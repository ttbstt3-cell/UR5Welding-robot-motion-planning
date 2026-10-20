// The only writable robot command in this executable is the generated set_tcp
// secondary program below. RTDE uses an empty input recipe and is output-only.
#include <ur_client_library/log.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Json = nlohmann::json;
using Pose = std::array<double, 6>;
using Clock = std::chrono::steady_clock;
constexpr double kStillVelocity = 0.005;
constexpr double kJointDrift = 0.1 * 3.14159265358979323846 / 180.0;
constexpr double kPositionTolerance = 1e-5;
constexpr double kRotationTolerance = 1e-7;
const std::vector<std::string> kRecipe{
    "timestamp", "actual_q", "actual_qd", "actual_TCP_pose", "tcp_offset",
    "robot_mode", "safety_mode", "runtime_state", "payload", "payload_cog"};

class StderrLog final : public urcl::LogHandler {
 public:
  void log(const char*, int, urcl::LogLevel, const char* message) override {
    std::cerr << "urcl: " << message << '\n';
  }
};

template <size_t N>
void require_finite(const std::array<double, N>& values, const std::string& name) {
  for (double value : values)
    if (!std::isfinite(value)) throw std::runtime_error(name + " contains a nonfinite value");
}

Pose read_pose(const Json& object, const std::string& key) {
  if (!object.contains(key) || !object.at(key).is_array() || object.at(key).size() != 6)
    throw std::runtime_error(key + " must contain exactly six numeric SI values");
  Pose pose{};
  for (size_t i = 0; i < pose.size(); ++i) {
    if (!object.at(key).at(i).is_number())
      throw std::runtime_error(key + " accepts numeric values only");
    pose[i] = object.at(key).at(i).get<double>();
  }
  require_finite(pose, key);
  return pose;
}

bool tcp_matches(const Pose& left, const Pose& right) {
  for (size_t i = 0; i < left.size(); ++i)
    if (std::abs(left[i] - right[i]) > (i < 3 ? kPositionTolerance : kRotationTolerance)) return false;
  return true;
}

bool expected_tcp_matches(const Pose& left, const Pose& right) {
  for (size_t i = 0; i < left.size(); ++i)
    if (std::abs(left[i] - right[i]) > 1e-7) return false;
  return true;
}

struct Snapshot {
  double timestamp = 0.0;
  Pose actual_q{}, actual_qd{}, actual_tcp_pose{}, tcp_offset{};
  int32_t robot_mode = 0, safety_mode = 0;
  uint32_t runtime_state = 0;
  double payload = 0.0;
  urcl::vector3d_t payload_cog{};

  Json json() const {
    return {{"timestamp", timestamp}, {"actual_q", actual_q}, {"actual_qd", actual_qd},
            {"actual_TCP_pose", actual_tcp_pose}, {"tcp_offset", tcp_offset},
            {"robot_mode", robot_mode}, {"safety_mode", safety_mode},
            {"runtime_state", runtime_state}, {"payload", payload}, {"payload_cog", payload_cog},
            {"units", {{"translation", "m"}, {"rotation", "rotation_vector_rad"},
                       {"joints", "rad"}, {"joint_velocity", "rad/s"}}},
            {"tcp_frame", "controller_tool_output_flange"}, {"reference_frame", "controller_base"}};
  }
};

template <typename T>
void get_field(const urcl::rtde_interface::DataPackage& package, const std::string& name, T& value) {
  if (!package.getData(name, value)) throw std::runtime_error("RTDE field unavailable: " + name);
}

Snapshot parse_snapshot(const urcl::rtde_interface::DataPackage& package) {
  Snapshot s;
  get_field(package, "timestamp", s.timestamp);
  get_field(package, "actual_q", s.actual_q);
  get_field(package, "actual_qd", s.actual_qd);
  get_field(package, "actual_TCP_pose", s.actual_tcp_pose);
  get_field(package, "tcp_offset", s.tcp_offset);
  get_field(package, "robot_mode", s.robot_mode);
  get_field(package, "safety_mode", s.safety_mode);
  get_field(package, "runtime_state", s.runtime_state);
  get_field(package, "payload", s.payload);
  get_field(package, "payload_cog", s.payload_cog);
  if (!std::isfinite(s.timestamp) || !std::isfinite(s.payload))
    throw std::runtime_error("RTDE scalar contains a nonfinite value");
  require_finite(s.actual_q, "actual_q");
  require_finite(s.actual_qd, "actual_qd");
  require_finite(s.actual_tcp_pose, "actual_TCP_pose");
  require_finite(s.tcp_offset, "tcp_offset");
  require_finite(s.payload_cog, "payload_cog");
  return s;
}

void require_stationary_stopped(const Snapshot& s) {
  if (s.robot_mode != 7) throw std::runtime_error("Robot must be RUNNING (robot_mode=7)");
  if (s.safety_mode != 1) throw std::runtime_error("Robot safety must be NORMAL (safety_mode=1)");
  if (s.runtime_state != 1)
    throw std::runtime_error("Stop the robot program manually before applying TCP (runtime_state must be STOPPED=1)");
  for (double velocity : s.actual_qd)
    if (std::abs(velocity) >= kStillVelocity) throw std::runtime_error("Robot is not stationary");
}

class OutputReader {
 public:
  explicit OutputReader(const std::string& ip)
      : client_(ip, notifier_, kRecipe, std::vector<std::string>{}, 20.0, false), package_(kRecipe) {
    if (!client_.init(1, std::chrono::milliseconds(250), 1, std::chrono::milliseconds(250)))
      throw std::runtime_error("RTDE initialization failed");
    if (!client_.start()) throw std::runtime_error("RTDE output stream did not start");
  }

  Snapshot next() {
    if (!client_.getDataPackage(package_, std::chrono::milliseconds(1000)))
      throw std::runtime_error("Timed out waiting for fresh RTDE output");
    Snapshot snapshot = parse_snapshot(package_);
    if (last_timestamp_ >= 0.0 && snapshot.timestamp <= last_timestamp_)
      throw std::runtime_error("RTDE timestamp did not advance");
    last_timestamp_ = snapshot.timestamp;
    return snapshot;
  }

  Snapshot stable() {
    const auto start_wall = Clock::now();
    Snapshot first = next(), current = first;
    require_stationary_stopped(first);
    while (current.timestamp - first.timestamp < 0.5 ||
           Clock::now() - start_wall < std::chrono::milliseconds(500)) {
      if (Clock::now() - start_wall > std::chrono::seconds(3))
        throw std::runtime_error("Stationary confirmation timed out");
      current = next();
      require_stationary_stopped(current);
      for (size_t i = 0; i < 6; ++i)
        if (std::abs(current.actual_q[i] - first.actual_q[i]) > kJointDrift)
          throw std::runtime_error("Joint angles changed during stationary confirmation");
      if (!tcp_matches(current.tcp_offset, first.tcp_offset))
        throw std::runtime_error("Active TCP changed during stationary confirmation");
    }
    return current;
  }

 private:
  urcl::comm::INotifier notifier_;
  urcl::rtde_interface::RTDEClient client_;
  urcl::rtde_interface::DataPackage package_;
  double last_timestamp_ = -1.0;
};

Json read_json_file(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read JSON file: " + path);
  return Json::parse(input);
}

struct Request {
  Pose target{}, expected{};
  std::string request_id, backup_path;
};

Request parse_request(const Json& object) {
  Request request;
  request.target = read_pose(object, "target_tcp_offset");
  request.expected = read_pose(object, "expected_current_tcp");
  double position_norm_squared = 0.0;
  for (size_t i = 0; i < 3; ++i) position_norm_squared += request.target[i] * request.target[i];
  if (position_norm_squared > 1.0)
    throw std::runtime_error("TCP XYZ norm exceeds 1 m; check metre units before applying");
  if (!object.contains("confirmed") || object.at("confirmed") != true)
    throw std::runtime_error("Manual confirmation is required");
  if (!object.contains("request_id") || !object.at("request_id").is_string() ||
      object.at("request_id").get<std::string>().empty())
    throw std::runtime_error("A nonempty request_id is required");
  if (!object.contains("backup_path") || !object.at("backup_path").is_string() ||
      object.at("backup_path").get<std::string>().empty())
    throw std::runtime_error("Existing before-write backup_path is required");
  request.request_id = object.at("request_id").get<std::string>();
  request.backup_path = object.at("backup_path").get<std::string>();
  for (size_t i = 3; i < 6; ++i)
    if (request.target[i] != request.expected[i])
      throw std::runtime_error("Position-only calibration must preserve all three rotation-vector values exactly");
  return request;
}

void require_backup(const Request& request) {
  Json backup = read_json_file(request.backup_path);
  if (backup.contains("snapshot")) backup = backup.at("snapshot");
  else if (backup.contains("before")) backup = backup.at("before");
  if (!expected_tcp_matches(read_pose(backup, "tcp_offset"), request.expected))
    throw std::runtime_error("Before-write backup does not match expected_current_tcp");
}

// All program characters are generated here from finite numeric values. The
// request cannot provide function names, raw script, extra commands or payload.
std::string secondary_program(const Pose& pose) {
  require_finite(pose, "target_tcp_offset");
  std::ostringstream program;
  program.imbue(std::locale::classic());
  program << std::setprecision(17) << "sec hmi_set_tcp():\n  set_tcp(p[";
  for (size_t i = 0; i < pose.size(); ++i) {
    if (i) program << ",";
    program << pose[i];
  }
  program << "])\nend\n";
  return program.str();
}

void require_preserved(const Snapshot& before, const Snapshot& after) {
  require_stationary_stopped(after);
  for (size_t i = 0; i < 6; ++i)
    if (std::abs(after.actual_q[i] - before.actual_q[i]) > kJointDrift)
      throw std::runtime_error("Joint position changed by more than 0.1 degrees after TCP write");
  if (std::abs(after.payload - before.payload) > 1e-6)
    throw std::runtime_error("Payload changed during TCP apply verification");
  for (size_t i = 0; i < 3; ++i)
    if (std::abs(after.payload_cog[i] - before.payload_cog[i]) > 1e-6)
      throw std::runtime_error("Payload CoG changed during TCP apply verification");
}

Json apply_tcp(const std::string& ip, const Request& request) {
  Json result{{"operation", "apply"}, {"request_id", request.request_id},
              {"success", false}, {"may_have_changed", false},
              {"persistence", "runtime_only"}, {"backup_path", request.backup_path},
              {"target_tcp_offset", request.target}};
  bool upload_attempted = false;
  try {
    require_backup(request);
    OutputReader reader(ip);
    Snapshot before = reader.stable();
    result["before"] = before.json();
    if (!expected_tcp_matches(before.tcp_offset, request.expected))
      throw std::runtime_error("Active robot TCP differs from expected_current_tcp; refresh and confirm again");
    for (size_t i = 3; i < 6; ++i)
      if (std::abs(before.tcp_offset[i] - request.target[i]) > 1e-10)
        throw std::runtime_error("Target orientation differs from the current robot TCP");

    urcl::comm::INotifier notifier;
    urcl::primary_interface::PrimaryClient primary(ip, notifier);
    primary.start(1, std::chrono::milliseconds(250));
    // Connection setup must not allow a stale precondition to authorize a write.
    before = reader.stable();
    result["before"] = before.json();
    if (!expected_tcp_matches(before.tcp_offset, request.expected))
      throw std::runtime_error("Robot TCP changed while opening the script interface");
    for (size_t i = 3; i < 6; ++i)
      if (std::abs(before.tcp_offset[i] - request.target[i]) > 1e-10)
        throw std::runtime_error("Robot TCP orientation changed while opening the script interface");
    result["expected_original_matched"] = true;
    upload_attempted = true;
    result["may_have_changed"] = true;
    // SEC upload has no execution acknowledgement. Use a single official
    // upload and advancing RTDE readback; do not call any stop/power APIs.
    if (!primary.sendScript(secondary_program(request.target)))
      throw std::runtime_error("Secondary program upload failed; verify active TCP before retrying manually");
    primary.stop();

    const auto deadline = Clock::now() + std::chrono::seconds(3);
    Snapshot after = before;
    size_t matching_samples = 0;
    double first_match_timestamp = -1.0;
    do {
      after = reader.next();
      result["after"] = after.json();
      require_preserved(before, after);
      if (tcp_matches(after.tcp_offset, request.target)) {
        if (matching_samples++ == 0) first_match_timestamp = after.timestamp;
        if (matching_samples >= 3 && after.timestamp - first_match_timestamp >= 0.1) {
          result["success"] = true;
          result["reason"] = "Active TCP verified by advancing RTDE readback; installation persistence was not changed";
          result["verification"] = {{"matching_samples", matching_samples},
                                    {"protocol", "advancing_rtde_tcp_offset_readback"},
                                    {"joint_drift_limit_deg", 0.1}, {"payload_unchanged", true},
                                    {"rotation_unchanged", true}};
          return result;
        }
      } else {
        matching_samples = 0;
        first_match_timestamp = -1.0;
      }
    } while (Clock::now() < deadline);
    throw std::runtime_error("RTDE did not confirm the requested TCP; no automatic retry or rollback was performed");
  } catch (const std::exception& error) {
    result["success"] = false;
    result["may_have_changed"] = upload_attempted;
    result["reason"] = error.what();
    return result;
  }
}

void expect_rejected(const Json& request, const std::string& label) {
  try { (void)parse_request(request); }
  catch (const std::exception&) { return; }
  throw std::runtime_error("Self-test did not reject " + label);
}

Json self_test() {
  Json request{{"target_tcp_offset", {0.01, 0.02, 0.38, -0.95, 2.45, 1.24}},
               {"expected_current_tcp", {0.015, 0.018, 0.379, -0.95, 2.45, 1.24}},
               {"confirmed", true}, {"request_id", "offline-self-test"}, {"backup_path", "offline.json"}};
  Request parsed = parse_request(request);
  const std::string code = secondary_program(parsed.target);
  if (code.rfind("sec hmi_set_tcp():\n  set_tcp(p[", 0) != 0 || code.substr(code.size() - 7) != "])\nend\n")
    throw std::runtime_error("Generated program is not the restricted secondary program");
  Json changed_rotation = request;
  changed_rotation["target_tcp_offset"][3] = 0.0;
  expect_rejected(changed_rotation, "rotation changes");
  Json injected = request;
  injected["target_tcp_offset"][0] = "0]); movej([0,0,0,0,0,0])";
  expect_rejected(injected, "script injection");
  Json unconfirmed = request;
  unconfirmed["confirmed"] = false;
  expect_rejected(unconfirmed, "unconfirmed requests");
  Json millimetres = request;
  millimetres["target_tcp_offset"][0] = 10.0;
  expect_rejected(millimetres, "millimetre units");
  Pose invalid = parsed.target;
  invalid[0] = std::numeric_limits<double>::infinity();
  bool invalid_rejected = false;
  try { (void)secondary_program(invalid); } catch (const std::exception&) { invalid_rejected = true; }
  if (!invalid_rejected) throw std::runtime_error("Nonfinite target was not rejected");
  Snapshot sample;
  sample.robot_mode = 7; sample.safety_mode = 1; sample.runtime_state = 1;
  require_stationary_stopped(sample);
  sample.runtime_state = 2;
  bool playing_rejected = false;
  try { require_stationary_stopped(sample); } catch (const std::exception&) { playing_rejected = true; }
  if (!playing_rejected) throw std::runtime_error("Running program was not rejected");
  return {{"operation", "self_test"}, {"success", true}, {"network_used", false},
          {"checks", {"numeric_only", "finite_values", "unchanged_rotation", "manual_confirmation",
                      "metre_units", "fixed_secondary_program", "stopped_program_guard"}}};
}

void print_help() {
  std::cout << "robot_tcp_helper --robot-ip IPv4 [--request request.json]\n"
               "robot_tcp_helper --self-test\n"
               "Default: output-only official UR RTDE snapshot.\n"
               "Request: confirmed=true, request_id, backup_path, target_tcp_offset[6], expected_current_tcp[6].\n"
               "Apply changes active TCP XYZ only, preserves rotation and payload; runtime persistence only.\n"
               "Robot must be NORMAL, RUNNING, stationary and robot program STOPPED.\n";
}
}  // namespace

int main(int argc, char** argv) {
  urcl::registerLogHandler(std::make_unique<StderrLog>());
  urcl::setLogLevel(urcl::LogLevel::WARN);
  try {
    std::string ip, request_path;
    bool test = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help" || argument == "-h") { print_help(); return 0; }
      if (argument == "--self-test") test = true;
      else if (argument == "--robot-ip" && i + 1 < argc) ip = argv[++i];
      else if (argument == "--request" && i + 1 < argc) request_path = argv[++i];
      else throw std::runtime_error("Unknown or incomplete command-line option: " + argument);
    }
    if (test) {
      if (!ip.empty() || !request_path.empty()) throw std::runtime_error("--self-test cannot be combined with network options");
      std::cout << self_test().dump() << '\n';
      return 0;
    }
    in_addr address{};
    if (ip.empty() || inet_pton(AF_INET, ip.c_str(), &address) != 1)
      throw std::runtime_error("A valid --robot-ip IPv4 address is required");
    if (!request_path.empty()) {
      Json result = apply_tcp(ip, parse_request(read_json_file(request_path)));
      std::cout << result.dump() << '\n';
      return result.at("success").get<bool>() ? 0 : 1;
    }
    OutputReader reader(ip);
    Snapshot first = reader.next();
    Snapshot snapshot = reader.next();
    if (snapshot.timestamp <= first.timestamp) throw std::runtime_error("Snapshot did not advance");
    std::cout << Json{{"operation", "read"}, {"success", true}, {"snapshot", snapshot.json()},
                     {"output_only", true}, {"input_recipe_empty", true}}.dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cout << Json{{"success", false}, {"may_have_changed", false}, {"reason", error.what()}}.dump() << '\n';
    return 1;
  }
}
