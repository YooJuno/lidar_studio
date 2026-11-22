#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Install the optional LiDAR detector models in user storage."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
import ssl
import sys
import urllib.request
import venv

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'python'))
from lidar_detection.models import DEFAULT_MODELS, MODELS, sha256_file

MODEL_CONFIG_KEYS={'tiny':'model_dir','short_range':'short_range_model_dir','base':'base_model_dir',
    'transfusion':'transfusion_model_dir'}

def directories():
    home=Path.home()
    config=home/'.config/lidar_viewer'
    data_base=Path(os.environ.get('XDG_DATA_HOME',home/'.local/share'))
    data=(data_base if data_base.is_absolute() else home/'.local/share')/'lidar_viewer'
    if sys.platform=='win32':
        config=Path(os.environ.get('APPDATA',home/'AppData/Roaming'))/'LiDARViewer'
        data=Path(os.environ.get('LOCALAPPDATA',home/'AppData/Local'))/'LiDARViewer'
    elif sys.platform=='darwin':config=data=home/'Library/Application Support/LiDARViewer'
    # Match the viewer's explicit XDG_CONFIG_HOME override on every platform.
    if os.environ.get('XDG_CONFIG_HOME') and Path(os.environ['XDG_CONFIG_HOME']).is_absolute():
        config=Path(os.environ['XDG_CONFIG_HOME'])/'lidar_viewer'
    if os.environ.get('XDG_DATA_HOME') and Path(os.environ['XDG_DATA_HOME']).is_absolute():
        data=Path(os.environ['XDG_DATA_HOME'])/'lidar_viewer'
    return config,data/'detector'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=['all',*MODELS],default='all',help='all installs the standard tiny and short_range pair')
    parser.add_argument('--cuda',action='store_true',help='ONNX Runtime CUDA 12 + cuDNN 9 (Linux/Windows x64)')
    parser.add_argument('--runtime-python',type=Path,help='Use an existing isolated Python with numpy, onnx and onnxruntime; install no packages')
    parser.add_argument('--install-packages',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--progress-file',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.install_packages and not args.runtime_python:parser.error('--install-packages needs an app-owned --runtime-python')
    def report(message,received=0,total=0):
        print(message,flush=True)
        if args.progress_file:
            args.progress_file.parent.mkdir(parents=True,exist_ok=True)
            temporary=args.progress_file.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(message=message,bytes=received,total_bytes=total)),encoding='utf-8')
            temporary.replace(args.progress_file)
    if args.cuda and (sys.platform=='darwin' or platform.machine().lower() not in ('x86_64','amd64')):
        parser.error('CUDA runtime requires Linux or Windows x64')
    config,data=directories()
    config_file=config/'detector.cfg'
    previous={}
    if config_file.is_file():
        for line in config_file.read_text(encoding='utf-8').splitlines():
            key,separator,value=line.partition('=')
            if separator:previous[key]=value
    selected=DEFAULT_MODELS if args.model=='all' else (args.model,)
    requested_python=args.runtime_python.expanduser().absolute() if args.runtime_python else None
    configured_python=Path(previous['python']).expanduser().absolute() if previous.get('python') else None
    reuse_runtime=(args.model in ('base','transfusion') and not args.cuda and configured_python is not None
        and configured_python.is_file() and requested_python in (None,configured_python))
    runtime_python=configured_python if reuse_runtime else requested_python
    if not runtime_python and not (3,10)<=sys.version_info[:2]<=(3,13):
        parser.error('Use Python 3.10 through 3.13 for the pinned model runtime')
    data.mkdir(parents=True,exist_ok=True)
    provider='auto' if args.cuda or (reuse_runtime and previous.get('provider','auto')!='cpu') else 'cpu'
    managed_runtime=previous.get('managed_runtime')=='1' if reuse_runtime else args.install_packages
    if runtime_python:
        python=runtime_python
    else:
        runtime=data/'runtime'
        print('Creating isolated Python runtime:',runtime,flush=True)
        venv.EnvBuilder(with_pip=True,system_site_packages=False).create(runtime)
        python=runtime/('Scripts/python.exe' if sys.platform=='win32' else 'bin/python')
    if not runtime_python or (args.install_packages and not reuse_runtime):
        report('Installing private CPU runtime packages' if not args.cuda else 'Installing CUDA runtime packages')
        runtime_package='onnxruntime-gpu[cuda,cudnn]==1.23.2' if args.cuda else 'onnxruntime==1.23.2'
        subprocess.run([str(python),'-E','-s','-B','-m','pip','install','--disable-pip-version-check','numpy==2.2.6','onnx==1.20.1',runtime_package],check=True)
    # A reused custom runtime must see the same site packages as the worker.
    runtime_args=[str(python),'-B'] if reuse_runtime and not managed_runtime else [str(python),'-E','-s','-B']
    subprocess.run([*runtime_args,'-c','import numpy,onnx,onnxruntime'],check=True)
    context=ssl.create_default_context()
    # Standalone/python.org interpreters may have no OS CA bundle configured.
    # pip ships a maintained CA bundle; add its roots without disabling TLS checks.
    try:
        from pip._vendor import certifi
        context.load_verify_locations(cafile=certifi.where())
    except ImportError:pass
    python_root=ROOT/'python'
    for variant in selected:
        spec=MODELS[variant];model=data/spec.folder;model.mkdir(parents=True,exist_ok=True)
        for name,url,digest in spec.files:
            target=model/name
            if target.is_file() and sha256_file(target)==digest:continue
            message=f'Downloading {spec.title}: {name}'
            report(message)
            temporary=target.with_suffix('.download')
            try:
                request=urllib.request.Request(url,headers={'User-Agent':'LiDAR-Studio-model-setup'})
                with urllib.request.urlopen(request,timeout=120,context=context) as source,temporary.open('wb') as output:
                    total=int(source.headers.get('Content-Length',0));received=0
                    while True:
                        chunk=source.read(1024*1024)
                        if not chunk:break
                        output.write(chunk)
                        received+=len(chunk);report(message,received,total)
                if sha256_file(temporary)!=digest:raise RuntimeError('Model checksum mismatch: '+name)
                temporary.replace(target)
            finally:temporary.unlink(missing_ok=True)
        report('Validating '+spec.title)
        subprocess.run([*runtime_args,'-c',
            'import sys;sys.path.insert(0,sys.argv[1]);from lidar_detection.worker import create_detector;'
            'm=create_detector(sys.argv[2],sys.argv[3],sys.argv[4]);print("Validated:",m.spec.title,m.provider)',
            str(python_root),str(model),provider,variant],check=True)
    config.mkdir(parents=True,exist_ok=True)
    settings={'python':str(python)}
    for variant,key in MODEL_CONFIG_KEYS.items():
        # Installing a single variant must not replace paths for other models.
        previous_path=previous.get(key)
        if variant=='transfusion' and variant not in selected and not previous_path:
            continue
        default_path=str(data/MODELS[variant].folder)
        if variant=='short_range' and not previous_path and previous.get('model_dir'):
            previous_path=str(Path(previous['model_dir']).parent/MODELS[variant].folder)
        settings[key]=default_path if variant in selected else previous_path or default_path
    settings.update(provider=provider,managed_runtime=str(int(managed_runtime)))
    for key,value in previous.items():
        if key not in settings:settings[key]=value
    if any('\r' in value or '\n' in value for value in settings.values()):
        raise RuntimeError('Newlines in runtime paths are unsupported')
    text=''.join(f'{key}={value}\n' for key,value in settings.items())
    temporary=config_file.with_suffix('.cfg.tmp');temporary.write_text(text,encoding='utf-8');temporary.replace(config_file)
    report('Models ready')
    print('Ready. Open Objects > Enable objects (or Retry model). Configuration:',config/'detector.cfg')

if __name__=='__main__':main()
