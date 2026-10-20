from pathlib import Path
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
tree=ET.parse(root/'rokae_description/xMateCR7_coppeliasim_fixed.urdf')
for mesh in tree.findall('.//mesh'):
 p=mesh.get('filename').replace('\\','/'); suffix=p[p.index('meshes/'):]; mesh.set('filename',str(root/'rokae_description'/suffix))
for trans in tree.findall('transmission'): tree.getroot().remove(trans)
tree.write(root/'rokae_description/CR7_linux.urdf',encoding='utf-8',xml_declaration=True)
src=Path('/home/ttbstt/jixiebi/ros2_ws/src/ur5_controller/ur5_controller/controller.py').read_text()
src=src.replace('UR5','CR7').replace('ur5','cr7').replace("self.declare_parameter('send_demo', True)","self.declare_parameter('send_demo', False)\n        self.declare_parameter('port', 23010)")
a=src.index('    JOINT_NAMES = [');b=src.index('\n    DEMO_TARGET',a)
src=src[:a]+"    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']\n"+src[b:]
src=src.replace("'/joint_trajectory'","'/cr7/joint_trajectory'").replace("'/joint_states'","'/cr7/joint_states'")
src=src.replace("RemoteAPIClient('127.0.0.1', 23000)","RemoteAPIClient('127.0.0.1', int(self.get_parameter('port').value))")
a=src.index('            self._joint_handles = list(');b=src.index('            if len(self._joint_handles)',a)
src=src[:a]+"            self._joint_handles = [self._sim.getObject('/CR7/joint' + str(i)) for i in range(1, 7)]\n"+src[b:]
src=src.replace('self._sim.setJointTargetPosition(handle, float(position))','self._sim.setJointPosition(handle, float(position))')
src=src.replace('        self._trajectory_plan = plan', '''        if any(not math.isfinite(v) for t, p in plan for v in [t] + p):
            self.get_logger().warning('Rejected nonfinite trajectory')
            return
        if self._sim is None or self._sim.getSimulationState() == self._sim.simulation_stopped:
            self.get_logger().warning('Start the CR7 simulation before publishing')
            return
        for _, positions in plan:
            for h, v in zip(self._joint_handles, positions):
                cyclic, interval = self._sim.getJointInterval(h)
                if not cyclic and not interval[0] <= v <= sum(interval):
                    self.get_logger().warning('Rejected joint limit violation')
                    return
        current = [self._sim.getJointPosition(h) for h in self._joint_handles]
        if plan[0][0] == 0 and max(abs(a-b) for a,b in zip(current,plan[0][1])) > 0.02:
            self.get_logger().warning('Trajectory start differs from current state')
            return
        if plan[0][0] > 0:
            plan.insert(0,(0.0,current))
        self._trajectory_plan = plan''')
src=src.replace('        elapsed = time.monotonic() - self._trajectory_start',"        if self._sim.getSimulationState() == self._sim.simulation_stopped:\n            self._trajectory_active = False\n            return\n        elapsed = time.monotonic() - self._trajectory_start")
(root/'scripts/cr7_controller.py').write_text(src)
