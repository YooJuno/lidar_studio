#!/usr/bin/env python3
"""Private Xvfb: real GPU projection, nearest return, navigation and playback."""
import argparse,json,math,os,shutil,struct,subprocess,tempfile,time,traceback
from pathlib import Path
from ui_controls import Controls
from ui_gl_smoke import X11,capture,changed_pixels,settings_inventory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();repo=Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True,exist_ok=True);out=Path(tempfile.mkdtemp(prefix='panorama-',dir=args.output.resolve()))
    private=out/'private';(private/'build').mkdir(parents=True);shutil.copytree(repo/'config',private/'config')
    (private/'config/profiles/default.cfg').write_text(
        'ground_removal_enabled=0\nclustering_enabled=0\ndetection_enabled=0\nroi_count=0\n'
        'imu_level_enabled=0\ngrid_enabled=0\npoint_size=6\ncolor_mode=2\n'
        'calibration_roll_deg=17\ncalibration_pitch_deg=-11\ncalibration_yaw_deg=42\ncalibration_z_m=2.2\n'
        'camera_yaw_rad=.37\ncamera_pitch_rad=.21\ncamera_distance=29\n')
    viewer=private/'build/lidar_studio';shutil.copy2(args.viewer.resolve(),viewer)
    clouds=private/'sequence';clouds.mkdir()
    def point(bearing,elevation,distance):
        yaw=-math.radians(bearing);pitch=math.radians(elevation)
        return distance*math.cos(pitch)*math.cos(yaw),distance*math.cos(pitch)*math.sin(yaw),distance*math.sin(pitch),.5
    for frame in range(12):
        points=[point(column*360/512-180,-20+ring*35/31,35+8*math.sin(column*.025)+frame)
                for ring in range(32) for column in range(512)]
        # A far return comes LAST at exactly the same direction. The depth
        # buffer must retain the blue near return, not the orange far one.
        points.extend([point(30,0,10),point(30,0,60),(0,0,0,.5)])
        (clouds/f'{frame:04d}.bin').write_bytes(b''.join(struct.pack('<4f',*p) for p in points))
    env=dict(os.environ,XDG_CONFIG_HOME=str(out/'settings'),LIDAR_VIEWER_UI_TRACE='1')
    saved=out/'settings/lidar_viewer/profiles/default.cfg'
    before=settings_inventory(repo);result={'status':'started','checks':{},'output':str(out)}
    process=x=window=None
    def check(name,value):
        result['checks'][name]=bool(value)
        if not value:raise AssertionError(name)
    def save():
        c.chord('s')
        return dict(line.split('=',1) for line in saved.read_text().splitlines() if '=' in line)
    def view(prefs):
        return tuple(float(prefs['panorama_'+key]) for key in ['center_deg','elevation_deg','horizontal_fov','vertical_fov'])
    try:
        x=X11();subprocess.run(['xrdb','-merge'],input='Xft.dpi: 96\n',text=True,check=True)
        for run in range(2):
            log_path=out/f'viewer-{run}.log'
            with log_path.open('wb') as log:
                process=subprocess.Popen([str(viewer),str(clouds),'--panorama'],cwd=private,env=env,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+20;window=None
                while not window:
                    if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError(log_path.read_text(errors='replace'))
                    window=x.viewer_window();time.sleep(.05)
                x.focus(window);x.resize(window,1600,960);c=Controls(x,window,log_path);c.rect('camera_Panorama');time.sleep(.3)
                prefs=save()
                if run==1:
                    check('panorama_navigation_restored_on_launch',view(prefs)==last_view and prefs['projection_mode']=='panorama')
                    capture(x,window,out/'05_restored.png')
                    process.terminate();process.wait(timeout=5);process=None;break
                check('cli_and_auto_fit_use_sensor_elevation',prefs['projection_mode']=='panorama' and abs(view(prefs)[1]+2.5)<.01 and abs(view(prefs)[3]-39)<.01)
                image=capture(x,window,out/'01_distance.png')
                # ComputeUiLayout at 1600 x 960: canvas (352,72,1232,718).
                px=round(352+1232*(.5+30/360));py=round(72+718*(.5-2.5/39))
                pixel=image.getpixel((px,py))
                check('gpu_projection_undoes_alignment_and_keeps_nearest_return',pixel[2]>pixel[0]+40 and pixel[2]>100)
                c.click('panorama_color');x.key(window,'Home');x.key(window,'Down');x.key(window,'Return');time.sleep(.2)
                colored=capture(x,window,out/'02_point_colors.png');p=colored.getpixel((px,py))
                check('existing_point_colors_are_available',min(p[:3])>170)
                c.click('panorama_color');x.key(window,'Home');x.key(window,'Return');time.sleep(.1)
                x.wheel(window,1020,400,6);time.sleep(.2);zoom=save()
                check('wheel_zooms_without_moving_3d_camera',view(zoom)[2]<200 and abs(float(zoom['camera_3d_yaw_rad'])-.37)<1e-5)
                c.drag((950,430),(1120,475));moved=save()
                check('drag_pans_the_panorama',view(moved)!=view(zoom))
                x.key(window,'Right');time.sleep(.3);next_frame=save()
                check('next_frame_keeps_panorama_view',view(next_frame)==view(moved))
                c.click('camera_3D');three=save()
                check('three_d_bookmark_is_preserved',abs(float(three['camera_yaw_rad'])-.37)<1e-5 and abs(float(three['camera_pitch_rad'])-.21)<1e-5 and abs(float(three['camera_distance'])-29)<1e-4)
                c.click('camera_2D');c.drag((850,430),(920,460));top=save()
                c.click('camera_Panorama');check('returning_to_panorama_keeps_zoom',view(save())==view(moved))
                c.click('camera_2D');restored=save()
                check('two_d_bookmark_is_preserved',all(restored[k]==top[k] for k in ['camera_target_x','camera_target_y','camera_yaw_rad','camera_distance']))
                c.click('camera_Panorama');c.click('panorama_reset');reset=save()
                check('reset_fits_the_full_scan',view(reset)[2]==360 and abs(view(reset)[3]-39)<.01)
                x.key(window,'space');first=capture(x,window,out/'03_playing.png');time.sleep(.35)
                second=capture(x,window,out/'04_playing_later.png');x.key(window,'space')
                check('panorama_playback_continues',changed_pixels(first,second,(500,220,1500,730))>500)
                last_view=view(save())
                process.terminate();process.wait(timeout=5);process=None;window=None
        check('original_profiles_unchanged',settings_inventory(repo)==before);result['status']='passed'
    except Exception as error:
        result.update(status='failed',error=str(error),traceback=traceback.format_exc())
        if process and process.poll() is None and window:
            try:capture(x,window,out/'failure.png')
            except Exception:pass
    finally:
        if process and process.poll() is None:process.terminate();process.wait(timeout=5)
        if x:x.close()
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
