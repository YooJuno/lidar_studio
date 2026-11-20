#!/usr/bin/env python3
"""Private Xvfb: list removal never deletes data, reloads it, or loses selection."""
import argparse
import hashlib
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
from ui_gl_smoke import X11,capture,changed_pixels,settings_inventory
from ui_controls import Controls
from ui_direct_live_smoke import mid360_packet
from ui_playback_smoke import write_fixture


def write_pcap_fixture(path):
    with path.open('wb') as capture_file:
        capture_file.write(struct.pack('<IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 101))
        for i in range(40):
            payload = mid360_packet(i)
            udp = struct.pack('!HHHH', 56300, 56301, len(payload) + 8, 0)
            ipv4 = struct.pack('!BBHHHBBH4s4s', 0x45, 0, len(udp) + len(payload) + 20,
                0, 0, 64, 17, 0, b'\x7f\x00\x00\x01', b'\x7f\x00\x00\x01')
            packet = ipv4 + udp + payload
            capture_file.write(struct.pack('<IIII', 1, i * 10000, len(packet), len(packet)) + packet)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();repo=Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True,exist_ok=True)
    out=Path(tempfile.mkdtemp(prefix='source-removal-',dir=args.output.resolve()))
    private=out/'private';(private/'build').mkdir(parents=True);shutil.copytree(repo/'config',private/'config')
    (private/'config/profiles/rosbag.cfg').write_text('ground_removal_enabled=0\nclustering_enabled=0\ndetection_enabled=0\nroi_enabled=0\nroi_count=0\nimu_level_enabled=0\ncolor_mode=0\n')
    viewer=private/'build/lidar_viewer';shutil.copy2(args.viewer.resolve(),viewer)
    bags=out/'bags';a=bags/'sensors_20261102_120000_a';b=bags/'sensors_20261102_110000_b';c=bags/'unknown_recording'
    for bag in [a,b,c]:write_fixture(bag)
    def fingerprint(folder):return {str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.rglob('*') if p.is_file()}
    originals={p.name:fingerprint(p) for p in [a,b,c]}
    config=out/'settings/lidar_viewer';config.mkdir(parents=True)
    (config/'data_folders.cfg').write_text('folder "'+str(bags)+'"\n');manifest=config/'hidden_recordings.cfg'
    env=dict(os.environ,XDG_CONFIG_HOME=str(out/'settings'),LIDAR_VIEWER_CATALOG_TRACE='1',LIDAR_VIEWER_UI_TRACE='1',GSETTINGS_BACKEND='memory',NO_AT_BRIDGE='1')
    before=settings_inventory(repo);result={'status':'started','checks':{},'output':str(out)}
    x11=X11();process=window=logfile=None;deadline=time.monotonic()+105;logpath=out/'viewer-1.log'
    def logs():return logpath.read_text(errors='replace') if logpath.exists() else ''
    def wait(test):
        while not test():
            if process.poll() is not None:raise RuntimeError(logs())
            if time.monotonic()>deadline:raise TimeoutError('Source removal UI timed out')
            time.sleep(.05)
    def check(name,value):
        result['checks'][name]=bool(value)
        if not value:raise AssertionError(name)
    def row(path):
        matches=re.findall(r'Source row: "'+re.escape(str(path))+r'" \| select ([\d.-]+) ([\d.-]+) \| remove ([\d.-]+) ([\d.-]+) \| available ([01])',logs())
        return tuple(map(float,matches[-1])) if matches else None
    def click_row(path,remove=False):
        wait(lambda:row(path) is not None);time.sleep(.25);r=row(path)
        x11.click(window,round(r[2 if remove else 0]),round(r[3 if remove else 1]));time.sleep(.25)
    def menu():Controls(x11,window,logpath).click('source_selector');time.sleep(.2)
    def restore():
        wait(lambda:re.search(r'Restore removed UI: ([\d.-]+) ([\d.-]+)',logs()) is not None)
        time.sleep(.25);m=re.findall(r'Restore removed UI: ([\d.-]+) ([\d.-]+)',logs())[-1]
        x11.click(window,*map(lambda v:round(float(v)),m));time.sleep(.3)
    def start(number,arguments):
        nonlocal process,window,logfile,logpath
        logpath=out/f'viewer-{number}.log';logfile=logpath.open('wb')
        process=subprocess.Popen([str(viewer),*arguments],env=env,stdout=logfile,stderr=subprocess.STDOUT)
        wait(x11.viewer_window);window=x11.viewer_window();x11.focus(window);x11.resize(window,1600,960)
    def stop():
        process.terminate();process.wait(timeout=8);logfile.close();time.sleep(.15)
    try:
        start(1,[str(b),'--play','--2d'])
        wait(lambda:'Opened bag:' in logs());time.sleep(.3)
        capture(x11,window,out/'01_playing.png');menu();click_row(a,True)
        wait(lambda:manifest.is_file() and str(a) in manifest.read_text())
        check('trash_does_not_open_the_recording','Indexing ROS 2 bag "'+str(a)+'"' not in logs())
        check('removal_keeps_original_files',fingerprint(a)==originals[a.name])
        count=logs().count('Opened bag:');click_row(b)
        check('removing_earlier_row_preserves_active_selection',logs().count('Opened bag:')==count)
        menu();click_row(b,True);x11.key(window,'Escape');time.sleep(.2)
        active=capture(x11,window,out/'02_removed_active_keeps_playing.png');time.sleep(.35)
        later=capture(x11,window,out/'03_still_playing.png')
        check('removing_loaded_bag_keeps_playback',changed_pixels(active,later,(390,180,1540,750))>80 and
              bool(Controls(x11,window,logpath).rect('playback_playing',after=len(logs()))))
        menu();restore();check('restore_clears_exclusions','hidden ' not in manifest.read_text())
        count=logs().count('Opened bag:');click_row(b)
        check('restore_recovers_loaded_bag_selection',logs().count('Opened bag:')==count)
        # Simulate a removed/unplugged source without destroying any test data.
        c.rename(out/'unplugged_recording');menu();wait(lambda:row(c) is not None and row(c)[4]==0)
        capture(x11,window,out/'04_unavailable_source.png');click_row(c,True)
        check('unavailable_source_can_be_removed',str(c) in manifest.read_text())
        click_row(a,True);stop()
        start(2,[]);wait(lambda:'Bag list: 1 recordings' in logs());menu()
        check('removed_sources_stay_removed_after_restart',str(a) not in '\n'.join(line for line in logs().splitlines() if line.startswith('Source row:')))
        click_row(b,True);wait(lambda:str(b) in manifest.read_text())
        check('last_remaining_source_can_be_removed',all(str(path) in manifest.read_text() for path in [a,b,c]))
        restore();wait(lambda:row(b) is not None)
        check('empty_list_can_restore_removed_sources','hidden ' not in manifest.read_text())
        check('restore_does_not_load_a_recording','Indexing ROS 2 bag' not in logs())
        capture(x11,window,out/'05_restored_list.png')
        click_row(a,True);stop()
        reopened=subprocess.run([str(viewer),str(a),'--inspect'],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15)
        (out/'explicit-open.log').write_text(reopened.stdout)
        check('explicit_open_restores_a_removed_source',reopened.returncode==0 and str(a) not in manifest.read_text())
        check('all_original_data_bytes_preserved',fingerprint(a)==originals[a.name] and fingerprint(b)==originals[b.name] and fingerprint(out/'unplugged_recording')==originals[c.name])
        standalone=out/'single.pcap'
        recording=out/'sensors_20261120_120000_capture';recording.mkdir()
        write_pcap_fixture(standalone);write_pcap_fixture(recording/'lidar.pcap')
        for run, name, source in [(3,'file',standalone),(4,'directory',recording)]:
            start(run,[str(source),'--play','--2d'])
            wait(lambda:'Opened point cloud:' in logs());menu();click_row(source,True)
            x11.key(window,'Escape');menu();restore()
            wait(lambda:row(source) is not None)
            count=logs().count('Opened point cloud:')
            click_row(source)
            check('restoring_active_pcap_'+name+'_keeps_playback',
                  logs().count('Opened point cloud:')==count and
                  bool(Controls(x11,window,logpath).rect('playback_playing',after=len(logs()))))
            stop()
        check('original_profiles_unchanged',before==settings_inventory(repo));result['status']='passed'
    except Exception as error:
        result.update(status='failed',error=str(error),traceback=traceback.format_exc())
        if window and process and process.poll() is None:
            try:capture(x11,window,out/'failure.png')
            except Exception:pass
    finally:
        if process and process.poll() is None:stop()
        x11.close();(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
