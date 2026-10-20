# xMate CR7 CoppeliaSim inertia fix

Generated files keep the original link/joint/mesh structure and preserve link masses. Inertia products are set to zero and diagonal values are adjusted so each link satisfies the rigid-body triangle inequalities.

| link | mass kg | ixx | iyy | izz |
|---|---:|---:|---:|---:|
| xMateCR7_base | 3.44074938 | 0.01764837 | 0.016617391 | 0.012016394 |
| xMateCR7_link1 | 3.44074938 | 0.01764837 | 0.016617391 | 0.012016394 |
| xMateCR7_link2 | 5.024380503 | 0.0136759927 | 0.01747453457 | 0.012535321 |
| xMateCR7_link3 | 2.439584706 | 0.026159338 | 0.024282179 | 0.011989835 |
| xMateCR7_link4 | 2.439584706 | 0.012 | 0.0221 | 0.014 |
| xMateCR7_link5 | 2.422757896 | 0.016 | 0.015 | 0.0213 |
| xMateCR7_link6 | 1.229800857 | 0.02 | 0.0212 | 0.02 |

CoppeliaSim import note: after import, keep the base fixed/static. Link1-link6 can be dynamic/respondable, and joints should be switched to force/torque mode with motors enabled for dynamic position control.
