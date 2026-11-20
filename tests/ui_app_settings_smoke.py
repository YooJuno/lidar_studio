#!/usr/bin/env python3
"""Private Xvfb: application scale survives profile changes and never enters data profiles."""
import argparse
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import time
import traceback
from ui_gl_smoke import X11,capture,settings_inventory
from ui_controls import Controls


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();repo=Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True,exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix='app-settings-',dir=args.output.resolve()))
    private=output/'private';(private/'build').mkdir(parents=True)
    shutil.copytree(repo/'config',private/'config')
    base='ground_removal_enabled=0\nclustering_enabled=0\nroi_enabled=0\nroi_count=0\n'
    for name,scale in [('default',1),('gangnam',1.5)]:
        (private/f'config/profiles/{name}.cfg').write_text(base+f'ui_scale={scale}\n')
    sources=[]
    for name in ['cloud','gangnam']:
        folder=private/name;folder.mkdir();path=folder/'points.bin'
        path.write_bytes(b''.join(struct.pack('ffff',x,y,0,.5) for x in range(-10,11) for y in range(-10,11)));sources.append(path)
    viewer=private/'build/lidar_viewer';shutil.copy2(args.viewer.resolve(),viewer)
    env=dict(os.environ,XDG_CONFIG_HOME=str(output/'isolated'),LIDAR_VIEWER_UI_TRACE='1');before=settings_inventory(repo)
    process=x11=window=None;result={'status':'started','checks':{},'output':str(output)}
    def event(key,down):
        code=x11.x.XKeysymToKeycode(x11.display,x11.x.XStringToKeysym(key.encode()));x11.xt.XTestFakeKeyEvent(x11.display,code,down,0)
    def chord(key):
        event('Control_L',True);event(key,True);event(key,False);event('Control_L',False);x11.x.XSync(x11.display,False);time.sleep(.25)
    def check(label,value):
        result['checks'][label]=bool(value)
        if not value:raise AssertionError(label)
    try:
        x11=X11();subprocess.run(['xrdb','-merge'],input='Xft.dpi: 96\n',text=True,check=True)
        for index,path in enumerate(sources):
            log_path=output/f'viewer-{index}.log'
            with log_path.open('wb') as log:
                process=subprocess.Popen([str(viewer),str(path),'--2d'],cwd=private,env=env,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+15;window=None
                while not window:
                    if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Viewer did not open')
                    window=x11.viewer_window();time.sleep(.1)
                x11.focus(window);x11.resize(window,1600,960)
                controls=Controls(x11,window,log_path)
                if index==0:
                    left, _, right, _ = controls.rect('open_input')
                    scaled_open_width = (right - left) * 1.1
                    controls.click('app_settings')
                    controls.enter('app_ui_scale',1.1,ctrl_click=True)
                    controls.click('app_settings')
                deadline=time.monotonic()+5
                while True:
                    left,_,right,_=controls.rect('open_input')
                    # Measure a real rendered control, independent of the
                    # theme palette or the settings window's widget positions.
                    if abs((right-left)-scaled_open_width)<3:break
                    if time.monotonic()>deadline:raise AssertionError(f'profile_{index}_uses_global_scale: Open width {right-left}')
                    time.sleep(.05)
                picture=capture(x11,window,output/f'{index}-scale.png')
                check(f'profile_{index}_uses_global_scale',abs((right-left)-scaled_open_width)<3)
                chord('s')
                name='default' if index==0 else 'gangnam'
                user=Path(env['XDG_CONFIG_HOME'])/'lidar_viewer/profiles'/f'{name}.cfg'
                profile=user if user.is_file() else private/f'config/profiles/{name}.cfg'
                check(f'profile_{index}_does_not_store_app_scale','ui_scale=' not in profile.read_text())
                app=Path(env['XDG_CONFIG_HOME'])/'lidar_viewer/app_settings.cfg'
                prefs=dict(line.split('=',1) for line in app.read_text().splitlines() if '=' in line)
                check(f'profile_{index}_keeps_global_scale',abs(float(prefs['ui_scale'])-1.1)<.001)
                process.terminate();process.wait(timeout=5);process=None
        check('original_profiles_unchanged',before==settings_inventory(repo));result['status']='passed'
    except Exception as error:
        result.update(status='failed',error=str(error),traceback=traceback.format_exc())
        if x11 and window and process and process.poll() is None:
            try:capture(x11,window,output/'failure.png')
            except Exception:pass
    finally:
        if process and process.poll() is None:process.terminate();process.wait(timeout=5)
        if x11:x11.close()
        (output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
