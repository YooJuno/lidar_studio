#!/usr/bin/env python3
"""Private Xvfb: fixed navigation, independent scroll, resizing and BIN timeline."""
import argparse,errno,json,os,re,shutil,struct,subprocess,tempfile,time,traceback
from pathlib import Path
from ui_controls import Controls
from ui_gl_smoke import X11,capture,changed_pixels,settings_inventory


def check_read_recovery(x, window, controls, clouds, out, log_path, check):
 bad_frame=clouds/'0002.bin'
 original=bad_frame.read_bytes()
 bad_frame.unlink()
 os.mkfifo(bad_frame)
 x.key(window,'Right')
 before=capture(x,window,out/'06_before_seek.png')
 controls.click('timeline',.75)
 # Relieve the blocked speculative read only after requesting a different frame.
 deadline=time.monotonic()+5
 while True:
  try:
   os.close(os.open(bad_frame,os.O_WRONLY|os.O_NONBLOCK));break
  except OSError as error:
   if error.errno!=errno.ENXIO or time.monotonic()>=deadline:raise
   time.sleep(.05)
 time.sleep(.4)
 bounds=(500,180,1540,750)
 after=capture(x,window,out/'07_valid_seek_after_prefetch_error.png')
 check('failed_prefetch_keeps_a_different_seek',changed_pixels(before,after,bounds)>80)
 check('speculative_failure_is_silent','Could not load frame' not in log_path.read_text(errors='replace'))
 bad_frame.unlink()
 bad_frame.write_bytes(b'invalid')
 offset=len(log_path.read_text(errors='replace'))
 controls.click('timeline',.18)
 controls.rect('dismiss_error',after=offset)
 time.sleep(.3)
 check('selected_bad_frame_stops_without_retry',log_path.read_text(errors='replace')[offset:].count('Could not load frame 3:')==1)
 controls.click('timeline',.75)
 time.sleep(.3)
 offset=len(log_path.read_text(errors='replace'))
 try:
  controls.rect('dismiss_error',timeout=.3,after=offset)
 except AssertionError:
  recovered=True
 else:
  recovered=False
 check('valid_seek_clears_frame_error',recovered)

 bad_frame.write_bytes(original)
 controls.click('playback_loop')  # The compact-view check saved this profile with Loop off.
 controls.rect('playback_loop_on')
 controls.click('playback_toggle')
 controls.rect('playback_playing')
 controls.click('open_input');controls.click('open_input_file')
 controls.rect('dismiss_error',after=offset)
 time.sleep(.5)
 offset=len(log_path.read_text(errors='replace'))
 check('failed_open_error_stays_visible_while_playing',bool(controls.rect('dismiss_error',after=offset)))
 check('failed_open_preserves_playback',bool(controls.rect('playback_playing',after=offset)))
 controls.click('dismiss_error')
 capture(x,window,out/'08_recovered_playback.png')


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--viewer',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 repo=Path(__file__).resolve().parents[1];a.output.mkdir(parents=True,exist_ok=True);out=Path(tempfile.mkdtemp(prefix='ux-',dir=a.output.resolve()))
 private=out/'private';(private/'build').mkdir(parents=True);shutil.copytree(repo/'config',private/'config')
 (private/'config/profiles/default.cfg').write_text('ground_removal_enabled=0\nclustering_enabled=0\ndetection_enabled=0\nroi_count=0\ncalibration_z_m=0\n')
 viewer=private/'build/lidar_viewer';shutil.copy2(a.viewer.resolve(),viewer);clouds=private/'sequence';clouds.mkdir()
 for i in range(12):
  points=[(x*.3,y*.3,0.,.1) for x in range(-20,21) for y in range(-20,21)]
  points.extend((i*.2+x*.1,2+y*.1,z*.1,.9) for x in range(8) for y in range(8) for z in range(15))
  (clouds/f'{i:04d}.bin').write_bytes(b''.join(struct.pack('<4f',*p) for p in points))
 env=dict(os.environ,XDG_CONFIG_HOME=str(out/'settings'),LIDAR_VIEWER_UI_TRACE='1');profile=out/'settings/lidar_viewer/app_settings.cfg'
 corrupt=out/'corrupt.bin';corrupt.write_bytes(b'invalid')
 picker=out/'picker';picker.mkdir()
 chooser=picker/'zenity';chooser.write_text('#!/bin/sh\nprintf "%s\\n" "$LIDAR_TEST_OPEN"\n');chooser.chmod(0o755)
 env.update(PATH=str(picker)+os.pathsep+env['PATH'],LIDAR_TEST_OPEN=str(corrupt))
 before=settings_inventory(repo);result={'status':'started','checks':{},'output':str(out)};process=x=window=None
 def check(label,v):
  result['checks'][label]=bool(v)
  if not v:raise AssertionError(label)
 def prefs():return dict(s.split('=',1) for s in profile.read_text().splitlines() if '=' in s)
 try:
  x=X11();subprocess.run(['xrdb','-merge'],input='Xft.dpi: 96\n',text=True,check=True)
  for run in range(3):
   log_path=out/f'viewer-{run}.log'
   with log_path.open('wb') as log:
    launch=[str(viewer),str(clouds),'--2d']
    if run<2:launch.append('--play')
    process=subprocess.Popen(launch,cwd=private,env=env,stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+20;window=None
    while not window:
     if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Viewer did not open')
     window=x.viewer_window();time.sleep(.1)
    x.focus(window);x.resize(window,1600,960);c=Controls(x,window,log_path)
    c.rect('tab_View');time.sleep(.15)
    if run==0:
     check('bin_folder_is_one_continuous_sequence','Source frames 0 11' in log_path.read_text())
     first=capture(x,window,out/'01_playing.png');time.sleep(.35);second=capture(x,window,out/'02_motion.png')
     check('bin_sequence_playback_moves',changed_pixels(first,second,(390,180,1540,750))>80)
     c.click('open_input');c.rect('open_input_file');c.rect('open_input_folder')
     capture(x,window,out/'02_open_menu.png');x.key(window,'Escape')
     check('one_open_menu_groups_file_and_folder_actions',True)
     c.click('tab_Ground');c.rect('ground_settings')
     capture(x,window,out/'02_ground_simple.png');c.click('ground_settings')
     capture(x,window,out/'02_ground_settings.png');c.click('ground_settings')
     c.rect('ground_average');c.rect('object_average')
     check('analysis_controls_separate_display_method_and_advanced_settings',True)
     x.resize(window,1600,720);c.click('tab_Align');tabs=c.rect('tab_Objects');x.wheel(window,200,500,-20)
     offset=len(log_path.read_text());check('navigation_does_not_scroll_with_settings',c.rect('tab_Objects',after=offset)==tabs)
     c.click('tab_Objects');c.rect('tracking_ids')
     check('can_switch_tabs_after_scrolling',True)
     x.resize(window,1600,960)
     c.click('tab_View');r=c.rect('inspector_resize');start=(int((r[0]+r[2])*.5),int((r[1]+r[3])*.5))
     c.drag(start,(start[0]+110,start[1]));time.sleep(.3)
     check('divider_resize_is_persisted',profile.exists() and 420<=float(prefs()['inspector_width'])<=440)
     resized=capture(x,window,out/'03_wider_panel.png');time.sleep(.35);later=capture(x,window,out/'04_motion_after_resize.png')
     check('resized_canvas_keeps_advancing',changed_pixels(resized,later,(550,180,1540,750))>80)
     offset=len(log_path.read_text());x.resize(window,800,600);time.sleep(.25)
     capture(x,window,out/'04_compact_window.png')
     navigation=['open_input','open_live','review_back','open_dataset','open_training','open_help','app_settings']
     transport=['source_selector','timeline','playback_toggle','playback_previous','playback_next','playback_loop','playback_speed']
     rects={}
     for name in navigation+['tab_Objects']+transport:
      left,top,right,bottom=c.rect(name,after=offset)
      check('compact_'+name,0<=left<right<=800 and 0<=top<bottom<=600)
      rects[name]=(left,top,right,bottom)
     check('compact_navigation_is_ordered_without_overlap',all(rects[a][2]<=rects[b][0] for a,b in zip(navigation,navigation[1:])))
     check('compact_source_has_its_own_row',rects['source_selector'][3]<=rects['timeline'][1])
     c.click('open_help');c.rect('open_about',after=offset);x.key(window,'Escape')
     offset=len(log_path.read_text());c.click('playback_toggle')
     check('compact_playback_action_is_reachable',bool(c.rect('playback_paused',after=offset)))
     c.click('playback_next');c.enter('playback_speed',12,ctrl_click=True)
     offset=len(log_path.read_text());c.click('playback_loop')
     check('compact_playback_options_are_reachable',bool(c.rect('playback_loop_off',after=offset)))
     x.resize(window,1600,960)
     c.chord('s');user=out/'settings/lidar_viewer/profiles/default.cfg'
     saved=user if user.exists() else private/'config/profiles/default.cfg'
     check('panel_width_is_not_a_data_setting','inspector_width=' not in saved.read_text())
    elif run==1:
     r=c.rect('inspector_resize');check('panel_width_is_restored_on_launch',440<r[0]<455)
     capture(x,window,out/'05_restored_width.png')
    else:
     c.rect('timeline')
     check_read_recovery(x,window,c,clouds,out,log_path,check)
    process.terminate();process.wait(timeout=5);process=None;window=None
  check('original_profiles_unchanged',before==settings_inventory(repo));result['status']='passed'
 except Exception as e:
  result.update(status='failed',error=str(e),traceback=traceback.format_exc())
  if process and process.poll() is None and window:
   try:capture(x,window,out/'failure.png')
   except Exception:pass
 finally:
  if process and process.poll() is None:process.terminate();process.wait(timeout=5)
  if x:x.close()
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
 return 0 if result['status']=='passed' else 1
if __name__=='__main__':raise SystemExit(main())
