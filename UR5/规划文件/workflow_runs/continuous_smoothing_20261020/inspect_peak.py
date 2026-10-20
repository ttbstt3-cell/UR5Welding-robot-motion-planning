"""Analytic derivative audit of immutable exported MAT; no robot connection."""
import json
from pathlib import Path
import numpy as np
from scipy.io import loadmat

here = Path(__file__).resolve().parent
source = here.parent / "continuous_refactor_20261019/staged_current/offline_optimized_trajectory.mat"
x = loadmat(source, simplify_cells=True)
q, t, v = x['qPath'], x['timeFromStart'].ravel(), x['velocities']
h = np.diff(t); d = np.diff(q, axis=0)
b = h[:, None] * v[:-1]
c = 3*d-2*b-h[:, None]*v[1:]
dd = -2*d+b+h[:, None]*v[1:]
a0, a1 = 2*c/h[:, None]**2, (2*c+6*dd)/h[:, None]**2
jerk = 6*dd/h[:, None]**3
labels = x['segmentLabel']
peaks = []
for e in np.argsort(np.max(np.abs(jerk), axis=1))[-10:][::-1]:
    j = int(np.argmax(np.abs(jerk[e])))
    peaks.append(dict(edge=int(e+1), joint=j+1, segment=str(labels[e+1]),
                      t=t[e:e+2].tolist(), dt=float(h[e]), dq=d[e].tolist(),
                      v0=v[e].tolist(), v1=v[e+1].tolist(),
                      a0=a0[e].tolist(), a1=a1[e].tolist(), jerk=jerk[e].tolist()))
result = dict(source=str(source), max_jerk=float(np.max(np.abs(jerk))),
              max_acceleration=float(np.max(np.abs([a0,a1]))),
              minimum_dt=float(h.min()), maximum_dt=float(h.max()), peaks=peaks,
              executionReady=False)
(here/'peak_python.json').write_text(json.dumps(result,indent=2))
for p in peaks[:5]:
    print(f"edge={p['edge']} J{p['joint']} {p['segment']} time={p['t']} dt={p['dt']:.9g} jerk={max(map(abs,p['jerk'])):.9g} dq={max(map(abs,p['dq'])):.9g}")
