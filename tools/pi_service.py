"""Manage only this lab's Pi server; never signal an unrelated stale PID."""
import argparse,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PID=ROOT/'server.pid'
def running():
 try:
  p=int(PID.read_text())
  return p if b'Lab2.code.server' in Path(f'/proc/{p}/cmdline').read_bytes() else None
 except (OSError,ValueError):return None
def stop():
 p=running()
 if p:
  os.kill(p,signal.SIGTERM)
  for _ in range(60):
   if not running():return
   time.sleep(.1)
  raise RuntimeError('Server still shutting down; inspect server.log')
def start():
 if running():print('Already running:',running());return
 (ROOT/'Lab2/evidence').mkdir(parents=True,exist_ok=True)
 with (ROOT/'Lab2/evidence/server.log').open('a') as log:
  p=subprocess.Popen([str(ROOT/'.venv/bin/python'),'-m','Lab2.code.server'],cwd=ROOT,stdout=log,stderr=log,start_new_session=True)
 PID.write_text(str(p.pid));time.sleep(2)
 if p.poll() is not None:raise RuntimeError('Startup failed; inspect Lab2/evidence/server.log')
 print('Started:',p.pid)
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','stop','restart','status']);a=parser.parse_args()
 if a.action in ('stop','restart'):stop()
 if a.action in ('start','restart'):start()
 if a.action=='status':print('Running PID '+str(running()) if running() else 'Stopped')
