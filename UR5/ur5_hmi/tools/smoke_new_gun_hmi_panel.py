"""Build and exercise the gun UI with ROS worker startup disabled."""
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np
from PyQt5.QtWidgets import QApplication

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gui.main_window import MainWindow


def main():
    app=QApplication.instance() or QApplication(sys.argv)
    # Do not use run_hmi.sh or start the ROS worker during offline QA.
    with patch('gui.main_window.QThread.start') as start:
        win=MainWindow()
        assert start.call_count==1
    info=win.robot_view.gun_mount_adjustment()
    saved=json.loads((ROOT.parent/'real_gun_mount_calibration.json').read_text())
    assert np.allclose(info['T_flange_source'], saved['T_flange_source'], atol=1e-12)
    assert info['geometry_preview'] and info['source_sha256']
    assert len(win.gun_source_rpy_spins)==3
    old_live=win.robot_view._live_q
    checks=[]
    for scale in (1.0,1.25,1.5):
        win.apply_ui_scale(scale)
        win.gun_roll_spin.setValue(90)
        win.gun_translation_spins[0].setValue(1.0)
        win.gun_source_rpy_spins[1].setValue(10)
        app.processEvents()
        payload=win._gun_mount_payload('DRAFT')
        assert payload['source_sha256']==info['source_sha256']
        assert np.asarray(payload['T_flange_source']).shape==(4,4)
        assert win.robot_view._live_q is old_live
        assert win.worker.node is None
        assert win.robot_view._wire_actor is None
        checks.append({'scale':scale,'mount_controls':True,'draft_has_source_hash_and_full_transform':True,
                       'ROS_worker_not_started':True,'LIVE_state_unchanged':True})
    result={'checks':checks,'robot_motion':'NO','writes_to_calibration':False,'loaded_full_transform_matches_saved':True,
            'loaded_mount_status':str(info.get('status')),
            'loaded_roll_deg':float(info.get('roll_deg',0.0)),
            'loaded_translation_mm':info.get('translation_mm'),
            'loaded_nominal_transform':info.get('T_flange_source_nominal')}
    folder=ROOT.parent/'real_gun_geometry'
    (folder/'hmi_panel_smoke.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    win.close()
    os._exit(0)


if __name__=='__main__':main()
