"""Verify that ACCP core differs solely by the two environment-query adapters."""
from pathlib import Path
import hashlib
import json
root = Path('/home/ttbstt/jixiebi')
file = root / 'UR5/matlab仿真/replan_weld_actual_axis_adaptive_core.m'
current = file.read_text()
one_new = '''function ok=localFullStateFree(scene,q)
env=check_scene_environment_collision(scene,q);'''
one_old = '''function ok=localFullStateFree(scene,q)
try
    [env,~,~]=checkCollision(scene.robot,q,scene.environment, ...
        "IgnoreSelfCollision","on","Exhaustive","on");
catch
    env=checkCollision(scene.robot,q,scene.environment,"IgnoreSelfCollision","on");
end'''
two_new = '''function [ok,gap]=localStateFreeWithGap(scene,q)
gap=NaN;
[env,distEnv]=check_scene_environment_collision(scene,q);'''
two_old = '''function [ok,gap]=localStateFreeWithGap(scene,q)
gap=NaN;
try
    [env,distEnv]=checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on','Exhaustive','on');
catch
    [env,distEnv]=checkCollision(scene.robot,q,scene.environment, ...
        'IgnoreSelfCollision','on');
end'''
assert current.count(one_new) == current.count(two_new) == 1
restored = current.replace(one_new, one_old).replace(two_new, two_old)
previous_sha = 'ffae45d1f8415fc8a2bb6fbcf125504a06fb31d8bbbdb4802c14362a43aa09bd'
assert hashlib.sha256(restored.encode()).hexdigest() == previous_sha
unchanged_expected = {
    'UR5/scene_geometry.json': 'bf63a4706a52d677093fa34f6830288515d0c7faeb54520cbff70d47db9bafcd',
    'UR5/environment_geometry_real.json': '02f81207182e3275f7ce69a3aac7e06f2cd9c26fcddefeeef3ec7c00611dbc55',
    'UR5/real_tcp_calibration_active.json': '1c721be8b817e1400e6ca0d55ed0fde911c4c84b5535cce85482522d6bfb3390',
    'UR5/real_gun_geometry/real_gun_geometry.json': '1cb68d906d78c6a7cec7fe2b413d178302a5f79f3787a6a940bbad145e048b82',
    'UR5/workpiece_pose_real.json': '70185d4befa5825653e0c6041ed36ea444e9bdbc0a84befa0338e0f318045009',
    'UR5/real_robot_description_calibrated.urdf': '17eed053e2c4f6e5420d722c5256ea1acb20c38fea812162c02f811723adf4a4',
    'UR5/ur5.ttt': '1e2b295f479d96ea51489e7bea7425dc832748695b2c0941ce69315afd642644',
}
for path, sha in unchanged_expected.items():
    assert hashlib.sha256((root/path).read_bytes()).hexdigest() == sha, path
result = {'pass': True, 'accp_core_only_environment_adapter_calls_changed': True,
          'core_before_sha256': previous_sha,
          'core_current_sha256': hashlib.sha256(current.encode()).hexdigest(),
          'protected_geometry_and_tcp_sha256': unchanged_expected, 'robot_motion': 'NO'}
Path(__file__).with_name('source_scope_verification.json').write_text(json.dumps(result, indent=2))
print('SOURCE_SCOPE_PASS: ACCP core restores byte-for-byte prior hash after only two adapter substitutions')
