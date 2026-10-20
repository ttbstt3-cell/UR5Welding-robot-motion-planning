// Offline invocation of installed JTC interpolation library. No ROS node/init.
#include <joint_trajectory_controller/trajectory.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>
int main() {
  joint_trajectory_controller::Trajectory controller;
  double worst_q=0,worst_v=0,worst_a=0; int count=0;
  for(int e=0;e<25;++e) {
    trajectory_msgs::msg::JointTrajectoryPoint a,b,o;
    a.positions.resize(6); b.positions.resize(6);
    a.velocities.resize(6); b.velocities.resize(6);
    const double h=.2+.07*e;
    const int64_t hn=std::llround(h*1e9);
    for(int j=0;j<6;++j) {
      a.positions[j]=std::sin(.17*e+j); b.positions[j]=a.positions[j]+.1*std::cos(e+.3*j);
      a.velocities[j]=.03*std::cos(e+j); b.velocities[j]=.02*std::sin(e-j);
    }
    for(int k=0;k<=100;++k) {
      const int64_t tn=std::llround(hn*k/100.0);
      const double dt=hn*1e-9,u=double(tn)/hn;
      controller.interpolate_between_points(rclcpp::Time(0,0,RCL_ROS_TIME),a,
        rclcpp::Time(hn,RCL_ROS_TIME),b,rclcpp::Time(tn,RCL_ROS_TIME),o);
      for(int j=0;j<6;++j) {
        const double d=b.positions[j]-a.positions[j],v0=a.velocities[j],v1=b.velocities[j];
        const double q=a.positions[j]+(3*u*u-2*u*u*u)*d+dt*(u*u*u-2*u*u+u)*v0+dt*(u*u*u-u*u)*v1;
        const double v=(6*u-6*u*u)*d/dt+(3*u*u-4*u+1)*v0+(3*u*u-2*u)*v1;
        const double acc=(6-12*u)*d/(dt*dt)+(6*u-4)*v0/dt+(6*u-2)*v1/dt;
        worst_q=std::max(worst_q,std::abs(q-o.positions[j]));
        worst_v=std::max(worst_v,std::abs(v-o.velocities[j]));
        worst_a=std::max(worst_a,std::abs(acc-o.accelerations[j])); ++count;
      }
    }
  }
  const bool pass=worst_q<1e-12&&worst_v<1e-11&&worst_a<1e-10;
  std::cout<<"{\"pass\":"<<(pass?"true":"false")<<",\"joint_samples\":"<<count
    <<",\"max_q_difference\":"<<worst_q<<",\"max_v_difference\":"<<worst_v
    <<",\"max_a_difference\":"<<worst_a<<",\"ros_node_started\":false,\"robot_motion\":false}\n";
  return pass?0:1;
}
