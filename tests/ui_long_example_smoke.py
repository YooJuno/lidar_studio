#!/usr/bin/env python3
"""Private Xvfb: continuous multi-sensor examples, six-field conversion and shared cache.

Optional --prepared-cache supplies already verified XYZI scans to save bandwidth;
selected scans are always omitted to exercise the real HTTPS and conversion path.
Without it, this test downloads both example lengths (~1.84 GB total).
"""
import argparse, hashlib, json, os, shutil, subprocess, tempfile, time, traceback
from pathlib import Path
from ui_gl_smoke import X11, capture, settings_inventory
from ui_controls import Controls


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepared-cache', type=Path)
    parser.add_argument('--sensor', choices=['xt32', 'hdl32', 'at128'], default='xt32')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='long-example-', dir=args.output.resolve()))
    private = out / 'private'
    (private / 'build').mkdir(parents=True)
    shutil.copy2(args.viewer.resolve(), private / 'build/lidar_studio')
    shutil.copytree(repo / 'config', private / 'config')
    env = dict(os.environ, XDG_CONFIG_HOME=str(out / 'settings'), XDG_DATA_HOME=str(out / 'data'), LIDAR_VIEWER_UI_TRACE='1')
    result = dict(status='started', checks={}, output=str(out))
    log_path = out / 'viewer.log'
    before = settings_inventory(repo)
    process = x = window = None

    def text(): return log_path.read_text(errors='replace')

    def wait(condition, seconds=480):
        end = time.monotonic() + seconds
        while not condition():
            if process.poll() is not None: raise RuntimeError(text())
            if time.monotonic() > end: raise AssertionError('Long example UI timed out: ' + text()[-1800:])
            time.sleep(.05)

    def check(name, condition):
        result['checks'][name] = bool(condition)
        if not condition: raise AssertionError(name)

    def seed(folder, count, omitted):
        if not args.prepared_cache: return
        folder.mkdir(parents=True, exist_ok=True)
        for i, source in enumerate(sorted(args.prepared_cache.glob('*.bin'))[:count]):
            if i not in omitted and not (folder / source.name).exists():
                os.link(source, folder / source.name)

    try:
        x = X11()
        subprocess.run(['xrdb', '-merge'], input='Xft.dpi: 96\n', text=True, check=True)
        with log_path.open('wb') as log:
            process = subprocess.Popen([str(private / 'build/lidar_studio')], cwd=private, env=env, stdout=log, stderr=subprocess.STDOUT)
        wait(lambda: x.viewer_window() is not None)
        window = x.viewer_window(); x.focus(window); x.resize(window, 1600, 960)
        c = Controls(x, window, log_path)
        c.rect('open_input')
        cache_root = out / 'data/lidar_viewer/examples'
        check('startup_does_not_download', not cache_root.exists())
        c.click('open_input'); c.click('open_example'); c.click('example_sensor')
        check('all_three_sensors_are_available', all('UI example_' + sensor in text() for sensor in ['xt32', 'hdl32', 'at128']))
        capture(x, window, out / '00_sensors.png')
        c.click('example_' + args.sensor); c.rect('example_30s')
        check('opening_chooser_does_not_download', not cache_root.exists())
        capture(x, window, out / '01_lengths.png')
        short = cache_root / ('urbanv2x-' + args.sensor + '-30s-v1')
        seed(short, 300, {0, 150, 299})
        c.click('example_start')
        wait(lambda: ' | 300 frames' in text())
        check('opens_300_consecutive_scans', len(list(short.glob('*.bin'))) == 300)
        check('ground_and_object_tools_available', 'UI tab_Ground' in text() and 'UI tab_Objects' in text() and 'UI tab_Data' not in text())
        files = sorted(short.glob('*.bin'))
        check('timestamps_are_in_source_order', files[0].stem == {'xt32': '1751963625946723', 'hdl32': '1751963624922547', 'at128': '1786608303957690'}[args.sensor] and len({f.stem for f in files}) == 300)
        check('six_fields_normalized_to_xyzi', all(f.stat().st_size > 0 and f.stat().st_size % 16 == 0 for f in files))
        if args.prepared_cache:
            check('normalized_values_match_verified_source', all(hashlib.sha256(f.read_bytes()).digest() == hashlib.sha256((args.prepared_cache / f.name).read_bytes()).digest() for f in files))
        c.click('tab_Ground'); time.sleep(.4)
        capture(x, window, out / '02_300_frames.png')
        long = cache_root / ('urbanv2x-' + args.sensor + '-60s-v1')
        seed(long, 600, set(range(301)))
        c.click('open_input'); c.click('open_example'); c.click('example_60s'); c.click('example_start')
        wait(lambda: ' | 600 frames' in text())
        check('opens_one_minute_with_600_scans', len(list(long.glob('*.bin'))) == 600)
        check('changing_length_reuses_completed_scans', 'Example ready | cached 599 / 600' in text() if args.prepared_cache else 'Example ready | cached 300 / 600' in text())
        check('prefix_is_shared_without_duplicate_storage', (short / files[0].name).stat().st_ino == (long / files[0].name).stat().st_ino)
        check('no_partial_scans_left', not list(cache_root.rglob('*.part')))
        capture(x, window, out / '03_600_frames.png')
        stamps = {f.name: f.stat().st_mtime_ns for f in long.glob('*.bin')}
        c.click('open_input'); c.click('open_example'); c.click('example_start')
        wait(lambda: 'Example ready | cached 600 / 600' in text())
        check('cached_reopen_does_not_rewrite', stamps == {f.name: f.stat().st_mtime_ns for f in long.glob('*.bin')})
        c.chord('s'); time.sleep(.15)
        saved = out / ('settings/lidar_viewer/profiles/example-' + args.sensor + '.cfg')
        if not saved.exists(): saved = private / ('config/profiles/example-' + args.sensor + '.cfg')
        check('long_example_profile_is_independent', saved.is_file() and 'ground_mode=fugseg' in saved.read_text())
        c.click('open_input'); c.click('open_example'); c.rect('example_delete'); c.click('example_delete')
        wait(lambda: 'Example deleted | urbanv2x-' + args.sensor + '-60s-v1' in text())
        check('delete_removes_selected_download', not long.exists())
        check('delete_keeps_other_recording_length', len(list(short.glob('*.bin'))) == 300)
        check('deleted_recording_is_unloaded', 'Example unloaded | urbanv2x-' + args.sensor + '-60s-v1' in text())
        capture(x, window, out / '04_deleted.png')
        c.click('example_30s'); c.click('example_start')
        wait(lambda: 'Example ready | cached 300 / 300' in text())
        check('other_length_remains_playable_after_delete', short.is_dir())
        check('original_profiles_untouched', before == settings_inventory(repo))
        result['status'] = 'passed'
    except Exception as error:
        result.update(status='failed', error=str(error), traceback=traceback.format_exc())
        if x and window and process and process.poll() is None:
            try: capture(x, window, out / 'failure.png')
            except Exception: pass
    finally:
        if process and process.poll() is None: process.terminate(); process.wait(timeout=8)
        if x: x.close()
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__': raise SystemExit(main())
