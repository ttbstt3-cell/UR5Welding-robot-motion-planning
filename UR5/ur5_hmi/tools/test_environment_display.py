"""Offline environment display regression against exported collision geometry.

No ROS worker, publishers or robot commands. --vtk requires a local X display.
Snapshot fixtures copy actual exported colliders; they are display tests only.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse
import hashlib
import json
import os
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/ur5_environment_display_mpl")
from gui.scene_geometry import EnvironmentGeometry, environment_components


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=ROOT.parent / "environment_geometry_real.json")
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vtk", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = EnvironmentGeometry(args.manifest)
    assert env.components, "No actual exported environment components to test"
    records = []
    for component in env.components:
        record = {"name": component.name, "category": "Environment", "type": component.geometry_type,
                  "frame_id": "base", "pose_base": component.pose_base.tolist()}
        for key in ("size_m", "radius_m", "length_m", "vertices_m"):
            value = getattr(component, key)
            if value is not None:
                record[key] = value.tolist() if isinstance(value, np.ndarray) else value
        if component.faces is not None:
            record["faces"] = (component.faces + 1).tolist()
        records.append(record)
    snapshot = json.loads(args.snapshot.read_text())
    # An older actual robot/tool snapshot is extended only in this test copy.
    # MATLAB's freshly exported snapshot can also be supplied without changes.
    expected = snapshot.get("environment")
    environment_binding = snapshot.get("environment_geometry_load", {})
    if expected is not None:
        assert environment_binding.get("source_sha256") == env.sha256, "STALE environment snapshot source SHA"
        assert environment_binding.get("version") == env.data.get("version"), "STALE environment geometry version"
    max_matrix_error = max_position_error = 0.0
    if expected is not None:
        actual = environment_components(expected, "base")
        assert {c.name for c in actual} == {c.name for c in env.components}
        by_name = {c.name: c for c in env.components}
        for component in actual:
            source = by_name[component.name]
            assert component.geometry_type == source.geometry_type
            assert np.allclose(component.pose_base, source.pose_base, atol=1e-12, rtol=0)
            max_matrix_error = max(max_matrix_error, float(np.max(np.abs(component.pose_base-source.pose_base))))
            max_position_error = max(max_position_error, float(np.linalg.norm(component.pose_base[:3,3]-source.pose_base[:3,3])*1000))
            for key in ("size_m", "radius_m", "length_m", "vertices_m"):
                a, b = getattr(component, key), getattr(source, key)
                if a is not None or b is not None:
                    assert a is not None and b is not None
                    assert np.asarray(a).shape == np.asarray(b).shape
                    assert np.allclose(a, b, atol=1e-12, rtol=0), key
    else:
        snapshot["environment"] = records
        snapshot["environment_geometry_load"] = {"source_coverage": env.data.get("scene_coverage", {})}
    fixture = args.output / "display_snapshot_fixture.json"
    fixture.write_text(json.dumps(snapshot))
    trajectory = SimpleNamespace(metadata={"preview_display": {"snapshot_file": str(fixture),
        "snapshot_sha256": hashlib.sha256(fixture.read_bytes()).hexdigest()}, "collisionResult": []})
    from gui.preview_display import PreviewDisplayMixin
    probe = PreviewDisplayMixin()
    probe.set_preview_trajectory(trajectory)
    assert probe._preview_scene_snapshot is not None
    assert not probe._preview_environment_error
    assert len(probe._preview_environment_components) == len(env.components)
    result = {"manifest": str(args.manifest), "manifest_sha256": env.sha256,
              "environment_component_count": len(env.components), "snapshot_source": str(args.snapshot),
              "actual_matlab_snapshot_compared": expected is not None, "frame_id": "base", "robot_motion": "NO"}
    result["source_coverage_warning"] = env.coverage_warning
    result["environment_geometry_version"] = env.data.get("version")
    result["snapshot_environment_source_sha256"] = environment_binding.get("source_sha256")
    result["snapshot_environment_version"] = environment_binding.get("version")
    result["environment_binding_pass"] = expected is not None and environment_binding.get("source_sha256") == env.sha256
    result["matlab_hmi_environment_max_matrix_error"] = max_matrix_error if expected is not None else None
    result["matlab_hmi_environment_max_position_error_mm"] = max_position_error if expected is not None else None
    if env.data.get("scene_coverage", {}).get("safety_fence") is False:
        assert "安全栅栏" in probe._preview_environment_coverage_warning
    audit_path = args.snapshot.parent / "preview_display_audit.json"
    frames = []
    if expected is not None and audit_path.is_file():
        from gui.robot_view import UrdfKinematicModel
        frames = json.loads(audit_path.read_text())["frames"]
        audited = SimpleNamespace(metadata={**trajectory.metadata, "collisionResult": frames,
            "flangePosePath": [f["flange_pose"] for f in frames],
            "gunPosePath": [f["gun_pose"] for f in frames],
            "wirePosePath": [f["wire_pose"] for f in frames]})
        diagnostic = PreviewDisplayMixin(); diagnostic.model = UrdfKinematicModel()
        diagnostic.set_preview_trajectory(audited)
        diagnostic._preview_q = frames[0]["q_rad"]; diagnostic._preview_tcp_index = 0
        first = diagnostic.preview_frame_diagnostics()
        result["first_frame_collision_status"] = first["collision_status"]
        result["first_frame_collision_pairs"] = first["collision_pairs"]
        if first["collision_pairs"]:
            assert first["collision_status"] != "PASS"
    if args.vtk:
        from PyQt5.QtWidgets import QApplication
        from gui.vtk_robot_view import VtkRobotView
        import vtk
        app = QApplication.instance() or QApplication([])
        view = VtkRobotView(); view._timer.stop(); view.resize(1100, 1000); view.show()
        view.reload_environment_geometry(args.manifest)
        assert len(view._environment_actors) == len(env.components)
        live_q = np.deg2rad([10, -80, 20, -30, 40, 50]).tolist()
        view._live_q = live_q.copy()
        view.set_preview_trajectory(trajectory)
        assert len(view._preview_environment_actors) == len(env.components)
        def matrix(actor):
            value = actor.GetUserMatrix()
            return np.array([[value.GetElement(i, j) for j in range(4)] for i in range(4)])
        for component in env.components:
            actor = view._preview_environment_actors[component.name]
            assert np.allclose(matrix(actor), component.pose_base, atol=1e-12)
            if component.faces is not None:
                assert actor.GetMapper().GetInput().GetNumberOfPolys() == len(component.faces)
        for q in (np.zeros(6), np.deg2rad([30, -100, 60, -90, 80, 45])):
            view.set_preview_state(q.tolist(), 0)
            assert view._live_q == live_q
            for component in env.components:
                assert np.allclose(matrix(view._preview_environment_actors[component.name]), component.pose_base, atol=1e-12)
            assert all(not a.GetVisibility() for a in view._environment_actors.values())
        nominal = view._scene.workpiece_pose_base()
        changed_wp = nominal.copy(); changed_wp[:3, 3] += [.01, -.02, .03]
        view.set_workpiece_calibration_preview(changed_wp)
        for component in env.components:
            assert np.allclose(matrix(view._preview_environment_actors[component.name]), component.pose_base, atol=1e-12)
        # Reloading a new live binding must not replace an old preview snapshot.
        view.reload_environment_geometry(args.manifest)
        assert all(not a.GetVisibility() for a in view._environment_actors.values())
        view.set_visibility(environment=False)
        assert all(not a.GetVisibility() for a in view._preview_environment_actors.values())
        view.set_visibility(environment=True)
        assert all(a.GetVisibility() for a in view._preview_environment_actors.values())
        if frames:
            view.set_preview_trajectory(audited)
            view.set_preview_state(frames[0]["q_rad"], 0)
        app.processEvents(); view.fit_scene_view("iso"); view._refresh()
        image = vtk.vtkWindowToImageFilter(); image.SetInput(view.vtk_widget.GetRenderWindow()); image.Update()
        writer = vtk.vtkPNGWriter(); writer.SetFileName(str(args.output / "preview_environment.png"))
        writer.SetInputConnection(image.GetOutputPort()); writer.Write()
        # A review screenshot with all actual environment bounds. This does
        # not change the HMI's default robot/workpiece camera-fit behavior.
        actors = [*view._preview_environment_actors.values(),
                  *[value[0] for value in view._actors.values()], *view._workpiece_actors]
        bounds = np.array([actor.GetBounds() for actor in actors], dtype=float)
        all_bounds = [item for i in range(3) for item in (bounds[:,2*i].min(), bounds[:,2*i+1].max())]
        view.renderer.ResetCamera(*all_bounds); view.renderer.GetActiveCamera().Zoom(.9)
        view.vtk_widget.GetRenderWindow().Render(); image.Modified(); image.Update()
        writer.SetFileName(str(args.output / "environment_overview.png")); writer.Write()
        result["preview_environment_transforms"] = view.environment_display_diagnostics()
        view.clear_preview_state()
        assert all(not a.GetVisibility() for a in view._preview_environment_actors.values())
        assert all(a.GetVisibility() for a in view._environment_actors.values())
        result["vtk_actors_and_toggle_pass"] = True
        result["preview_environment_fixed_under_fk_and_workpiece_updates"] = True
        result["live_q_untouched"] = True
        view.close()
    (args.output / "display_test_report.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
