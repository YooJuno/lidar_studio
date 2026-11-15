"""Installer transactions and TLS verification; no public download in unit tests."""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import ssl
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('studio_setup',ROOT/'tools/setup_detector.py')
setup=importlib.util.module_from_spec(spec);spec.loader.exec_module(setup)


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='studio-setup-')
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name);self.config=self.root/'settings';self.config.mkdir()
        self.data=self.root/'data';self.progress=self.root/'progress.json'
        digest=hashlib.sha256(b'validated model').hexdigest()
        self.models={name:SimpleNamespace(folder=name+'-model',title=name,
            files=[('model.onnx','https://models.example.test/'+name,digest)])for name in ('tiny','short_range')}
        self.run=self.enter(patch.object(setup.subprocess,'run'))
        self.enter(patch.object(setup,'directories',return_value=(self.config,self.data)))
        self.enter(patch.object(setup,'MODELS',self.models))
        self.enter(patch.object(sys,'argv',['setup','--runtime-python',str(self.root/'runtime space/python'),
            '--progress-file',str(self.progress)]))
        self.enter(contextlib.redirect_stdout(io.StringIO()))

    def enter(self,manager):
        value=manager.__enter__();self.addCleanup(manager.__exit__,None,None,None);return value

    def response(self,request,**kwargs):
        self.assertTrue(kwargs['context'].check_hostname)
        self.assertEqual(kwargs['context'].verify_mode,ssl.CERT_REQUIRED)
        source=io.BytesIO(b'validated model');source.headers={'Content-Length':'15'};return source

    def test_verified_install_publishes_config_last_and_reuses_cache(self):
        with patch.object(setup.urllib.request,'urlopen',side_effect=self.response) as download:
            setup.main();self.assertEqual(download.call_count,2)
            config=(self.config/'detector.cfg').read_text()
            self.assertIn('short_range_model_dir=',config)
            self.assertEqual(json.loads(self.progress.read_text())['message'],'Models ready')
            setup.main();self.assertEqual(download.call_count,2)
            self.assertEqual((self.config/'detector.cfg').read_text(),config)

    def test_failed_checksum_preserves_configuration_and_removes_partial(self):
        (self.config/'detector.cfg').write_text('previous working setup')
        def damaged(request,**kwargs):
            source=self.response(request,**kwargs)
            if request.full_url.endswith('short_range'):
                source=io.BytesIO(b'damaged');source.headers={}
            return source
        with patch.object(setup.urllib.request,'urlopen',side_effect=damaged):
            with self.assertRaisesRegex(RuntimeError,'checksum'):setup.main()
        self.assertEqual((self.config/'detector.cfg').read_text(),'previous working setup')
        self.assertFalse(list(self.data.rglob('*.download')))
        self.assertFalse((self.data/'short_range-model/model.onnx').exists())

    def test_validation_failure_does_not_replace_configuration(self):
        (self.config/'detector.cfg').write_text('previous working setup')
        self.run.side_effect=[None,RuntimeError('invalid original model')]
        with patch.object(setup.urllib.request,'urlopen',side_effect=self.response):
            with self.assertRaisesRegex(RuntimeError,'invalid original'):setup.main()
        self.assertEqual((self.config/'detector.cfg').read_text(),'previous working setup')

    def test_data_override_is_honored_on_macos(self):
        # Exercise the original function rather than the per-test installer fixture.
        original=self.directory_function
        with patch.object(setup.sys,'platform','darwin'),patch.dict(os.environ,
            XDG_CONFIG_HOME=str(self.root/'config override'),XDG_DATA_HOME=str(self.root/'data override')):
            config,data=original()
        self.assertEqual(config,self.root/'config override/lidar_viewer')
        self.assertEqual(data,self.root/'data override/lidar_viewer/detector')


InstallerTests.directory_function=staticmethod(setup.directories)
if __name__=='__main__':unittest.main()
