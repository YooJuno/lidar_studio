#!/usr/bin/env python3
"""Private Xvfb: visible missing-model state and slow-CPU playback recovery.

The protocol fixture tests scheduling; real neural inference is required by
the packaged Mac test using the original downloaded models.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import time
import traceback
from ui_gl_smoke import X11, capture
from ui_controls import Controls


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    out=Path(tempfile.mkdtemp(prefix='model-setup-',dir=args.output.resolve()))
    private=out/'private';(private/'build').mkdir(parents=True)
    repo=Path(__file__).resolve().parents[1]
    shutil.copytree(repo/'config',private/'config');(private/'python').mkdir()
    (private/'config/profiles/default.cfg').write_text('ground_removal_enabled=0\nclustering_enabled=0\ndetection_enabled=1\ndetection_model=tiny\nroi_enabled=0\n')
    scans=private/'sequence';scans.mkdir()
    for i in range(100):
        (scans/f'{i:06}.bin').write_bytes(b''.join(struct.pack('ffff',x+i*.01,y,0.,.5) for x in range(-8,9) for y in range(-8,9)))
    viewer=private/'build/lidar_studio';shutil.copy2(args.viewer.resolve(),viewer)
    env=dict(os.environ,XDG_CONFIG_HOME=str(out/'settings'),LIDAR_VIEWER_UI_TRACE='1',LIDAR_VIEWER_DETECTOR_TRACE='1')
    log_path=out/'viewer.log';result=dict(status='started',checks={});process=x=window=None
    def text():return log_path.read_text(errors='replace')
    def wait(condition,seconds=20):
        deadline=time.monotonic()+seconds
        while not condition():
            if process.poll() is not None:raise RuntimeError(text())
            if time.monotonic()>deadline:raise AssertionError('Timed out: '+text()[-2000:])
            time.sleep(.04)
    def check(name,value):
        result['checks'][name]=bool(value)
        if not value:raise AssertionError(name)
    try:
        x=X11()
        with log_path.open('wb') as log:
            process=subprocess.Popen([str(viewer),str(scans),'--2d','--play','--detect'],cwd=private,env=env,stdout=log,stderr=subprocess.STDOUT)
        wait(lambda:x.viewer_window() is not None);window=x.viewer_window();x.focus(window);x.resize(window,1500,1000)
        c=Controls(x,window,log_path);c.click('tab_Objects')
        rect=c.rect('object_status_missing')
        check('missing_model_notice_visible_without_scrolling',100<rect[1]<360)
        capture(x,window,out/'missing-model.png')
        config=out/'settings/lidar_viewer';config.mkdir(parents=True,exist_ok=True)
        (config/'detector.cfg').write_text(f'python={os.sys.executable}\nmodel_dir={private}\nprovider=cpu\n')
        (private/'python/detector_worker.py').write_text('''import struct,sys,time
source=sys.stdin.buffer;sink=sys.stdout.buffer
q=struct.Struct('<4sQIf');r=struct.Struct('<4sQIfI')
def exact(n):
 b=b''
 while len(b)<n:
  c=source.read(n-len(b))
  if not c:raise SystemExit()
  b+=c
 return b
sink.write(r.pack(b'LDR1',0,0,0,20)+b'CPUExecutionProvider');sink.flush()
while True:
 tag,revision,count,score=q.unpack(exact(q.size));exact(count*12);time.sleep(2.4)
 sink.write(r.pack(b'LDR1',revision,1,2400,0)+struct.pack('<9f',1,2,.8,4,2,1.6,0,.8,0));sink.flush()
''')
        c.click('object_enable');c.click('object_enable')
        c.rect('object_status_ready')
        started=time.monotonic();c.chord('s')
        saved=config/'profiles/default.cfg'
        wait(lambda:saved.exists(),seconds=1.5)
        check('UI_controls_respond_while_CPU_inference_is_pending',time.monotonic()-started<1.5)
        wait(lambda:bool(re.search(r'Detector frame \d+ revision \d+ boxes 1 full 1',text())),seconds=12)
        check('CPU_inference_over_two_seconds_reaches_playback',True)
        capture(x,window,out/'slow-cpu-detection.png')
        result['status']='passed'
    except Exception as error:
        result.update(status='failed',error=str(error),traceback=traceback.format_exc())
        if x and window and process and process.poll() is None:
            try:capture(x,window,out/'failure.png')
            except Exception:pass
    finally:
        if process and process.poll() is None:process.terminate();process.wait(timeout=8)
        if x:x.close()
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
