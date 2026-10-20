"""Terminate only this fixture's process; inspect only its recorded workers."""
import json, os, signal, subprocess, time, tempfile
from pathlib import Path

here=Path(__file__).resolve().parent
out=Path(tempfile.mkdtemp(prefix='cancel_test_',dir=here))
with (out/'fixture.log').open('w') as log:
    proc=subprocess.Popen(['matlab','-batch',f"addpath('{here}'); cancellation_fixture('{out}')"],stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+60
    pidfile=out/'owned_pids.json'
    while not pidfile.exists() and proc.poll() is None and time.monotonic()<deadline:
        time.sleep(.2)
    assert pidfile.exists(), 'Fixture did not start: '+(out/'fixture.log').read_text()
    pids=json.loads(pidfile.read_text())
    pids['parent_pid']=int(pids['parent_pid'])
    pids['worker_pids']=list(map(int,pids['worker_pids']))
    owned=[pids['parent_pid']]+pids['worker_pids']
    def birth(pid):
        try: return Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()[19]
        except FileNotFoundError: return None
    starts={p:birth(p) for p in owned}
    assert all(starts.values()), 'Fixture processes missing before cancellation'
    clock=time.monotonic()
    proc.terminate()
    try: proc.wait(timeout=1)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=5)
    def alive(pid):
        try:
            stat=Path(f'/proc/{pid}/stat').read_text()
            fields=stat.split(') ',1)[1].split()
            return fields[0] != 'Z' and fields[19] == starts[pid]
        except FileNotFoundError: return False
    while any(alive(p) for p in owned) and time.monotonic()-clock<35:
        time.sleep(.2)
    remaining=[p for p in owned if alive(p)]
    result=dict(pass_all=not remaining, elapsed_after_termination=time.monotonic()-clock,
                parent_pid=pids['parent_pid'], worker_pids=pids['worker_pids'],
                remaining=remaining, robot_motion='NO')
    # No broad process killing: only exact PIDs created by this fixture.
    for p in remaining:
        os.kill(p,signal.SIGTERM)
    (out/'result.json').write_text(json.dumps(result,indent=2))
    (here/'cancellation_validation.json').write_text(json.dumps(dict(result,run_dir=str(out)),indent=2))
    print(result)
    assert result['pass_all'], 'Pool did not follow parent shutdown within budget'
