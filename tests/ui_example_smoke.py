#!/usr/bin/env python3
"""Private Xvfb: real public HTTPS example download, auto-open and cache reuse."""
import argparse,json,os,re,shutil,subprocess,tempfile,time,traceback,hashlib
from pathlib import Path
from ui_gl_smoke import X11,capture,changed_pixels,settings_inventory
from ui_controls import Controls

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--viewer',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 repo=Path(__file__).resolve().parents[1];args.output.mkdir(parents=True,exist_ok=True);out=Path(tempfile.mkdtemp(prefix='example-',dir=args.output.resolve()))
 private=out/'private';(private/'build').mkdir(parents=True);shutil.copy2(args.viewer.resolve(),private/'build/lidar_studio');shutil.copytree(repo/'config',private/'config')
 env=dict(os.environ,XDG_CONFIG_HOME=str(out/'settings'),XDG_DATA_HOME=str(out/'data'),LIDAR_VIEWER_UI_TRACE='1',LIDAR_VIEWER_ROI_TRACE='1')
 result=dict(status='started',checks={},output=str(out));log_path=out/'viewer.log';before=settings_inventory(repo);process=x=window=None
 def text():return log_path.read_text(errors='replace')
 def wait(condition,seconds=75):
  end=time.monotonic()+seconds
  while not condition():
   if process.poll()is not None:raise RuntimeError(text())
   if time.monotonic()>end:raise AssertionError('Example UI timed out: '+text()[-1500:])
   time.sleep(.05)
 def check(name,condition):
  result['checks'][name]=bool(condition)
  if not condition:raise AssertionError(name)
 try:
  x=X11();subprocess.run(['xrdb','-merge'],input='Xft.dpi: 96\n',text=True,check=True)
  with log_path.open('wb')as log:process=subprocess.Popen([str(private/'build/lidar_studio')],cwd=private,env=env,stdout=log,stderr=subprocess.STDOUT)
  wait(lambda:x.viewer_window()is not None);window=x.viewer_window();x.focus(window);x.resize(window,1600,960);c=Controls(x,window,log_path)
  c.rect('open_input');time.sleep(.2);cache=out/'data/lidar_viewer/examples/driving-v1'
  check('no_network_or_download_at_startup',not cache.exists()and'Example ready'not in text())
  c.click('open_input');c.click('open_example');c.click('example_sensor');c.click('example_quick');c.click('example_start');wait(lambda:'Example ready |'in text())
  wait(lambda:'Opened point cloud:'in text()and' | 6 frames'in text())
  check('one_click_downloads_and_opens_six_scans',len(list(cache.glob('*.bin')))==6)
  check('first_open_fetches_public_files','Example ready | cached 0 / 6'in text())
  check('loading_uses_analysis_not_readonly_dataset','UI tab_Ground'in text()and'UI tab_Objects'in text()and'UI tab_Data'not in text())
  check('partial_files_never_become_scans',not list(cache.glob('*.part')))
  c.click('tab_Ground');x.move(window,1590,780);time.sleep(.4);a=capture(x,window,out/'01_downloaded.png');time.sleep(.23);b=capture(x,window,out/'02_playing.png')
  check('example_autoplays_without_blocking_controls',changed_pixels(a,b,(380,180,1540,750))>100)
  stamps={p.name:(p.stat().st_mtime_ns,hashlib.sha256(p.read_bytes()).hexdigest())for p in cache.glob('*.bin')}
  c.click('open_input');c.click('open_example');c.click('example_sensor');c.click('example_quick');c.click('example_start');wait(lambda:'Example ready | cached 6 / 6'in text())
  check('cached_open_does_not_download_or_rewrite',stamps=={p.name:(p.stat().st_mtime_ns,hashlib.sha256(p.read_bytes()).hexdigest())for p in cache.glob('*.bin')})
  c.chord('s');saved=out/'settings/lidar_viewer/profiles/example.cfg'
  if not saved.exists():saved=private/'config/profiles/example.cfg'
  check('example_has_its_own_saved_profile',saved.is_file()and'ground_mode=fugseg'in saved.read_text())
  check('existing_recording_profiles_untouched',before==settings_inventory(repo))
  result['status']='passed'
 except Exception as error:
  result.update(status='failed',error=str(error),traceback=traceback.format_exc())
  if x and window and process and process.poll()is None:
   try:capture(x,window,out/'failure.png')
   except Exception:pass
 finally:
  if process and process.poll()is None:process.terminate();process.wait(timeout=8)
  if x:x.close()
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 return 0 if result['status']=='passed'else 1
if __name__=='__main__':raise SystemExit(main())
