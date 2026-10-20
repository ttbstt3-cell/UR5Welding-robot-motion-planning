#!/usr/bin/env python3

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")

base = sim.getObject("/UR5")
connection = sim.getObject("/UR5/connection")
mount = sim.getObject("/UR5/connection/WeldingGunMount")
tcp = sim.getObject("/UR5/connection/WeldingGunMount/WeldTCP")


def print_pose(name, obj, ref):
    pose = sim.getObjectPose(obj, ref)

    print(name)
    print("position =", pose[:3])
    print("quaternion [qx qy qz qw] =", pose[3:])
    print()


print_pose("connection relative base", connection, base)
print_pose("mount relative connection", mount, connection)
print_pose("TCP relative mount", tcp, mount)
print_pose("TCP relative base", tcp, base)
