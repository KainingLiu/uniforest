"""Compare real legacy frames/actions on one additive firmware and fixed main."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.project_paths import PROJECT as ROOT, FIRMWARE
FIX=Path(__file__).parent/'firmware'
BASE=FIX/'additive_baseline'


@unittest.skipUnless((FIRMWARE).is_dir(),
                     'A-board source checks run on the development checkout')
class AdditiveFirmwareTests(unittest.TestCase):
    def compile_run(self,source,*,baseline=False,arguments=()):
        compiler=shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
        vcvars=None
        if compiler is None and os.name=='nt':
            compiler=shutil.which('cl')
            if compiler is None:
                vs=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio'
                candidates=sorted(vs.glob('*/*/VC/Auxiliary/Build/vcvars64.bat'))
                if candidates: vcvars,compiler=candidates[-1],'cl'
        if compiler is None: self.skipTest('host C compiler unavailable')
        work=ROOT/'work/firmware-host';work.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=work,prefix='additive-') as tmp:
            build=Path(tmp); exe=build/'harness.exe'
            includes=[FIX]
            if baseline: includes += [FIX/'protocol_baseline',BASE/'Core/Src',BASE/'Core/Inc']
            includes += [FIRMWARE / 'Core/Src',FIRMWARE / 'Core/Inc']
            msvc=Path(compiler).stem.lower()=='cl'
            if msvc:
                cmd=[compiler,'/nologo','/utf-8','/std:c11','/W1',
                     *([] if baseline else ['/DHOST_ADDITIVE=1']),
                     *(f'/I{p}' for p in includes),f'/Fe:{exe}',f'/Fo:{build / "harness.obj"}',str(FIX/source)]
                if vcvars:
                    batch=build/'build.cmd'
                    batch.write_text(f'@echo off\ncall "{vcvars}" >nul\nif errorlevel 1 exit /b\n'+subprocess.list2cmdline(cmd)+'\nexit /b %errorlevel%\n',encoding='utf-8')
                    cmd=[os.environ.get('COMSPEC','cmd.exe'),'/d','/c',str(batch)]
            else:
                cmd=[compiler,'-std=c11',*([] if baseline else ['-DHOST_ADDITIVE=1']),
                     *(f'-I{p}' for p in includes),str(FIX/source),'-o',str(exe)]
            result=subprocess.run(cmd,cwd=build,capture_output=True,text=True,errors='replace',timeout=60)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            result=subprocess.run([str(exe),*arguments],cwd=build,capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            return result.stdout

    def test_original_four_actions_and_stops_match_main(self):
        self.assertEqual(self.compile_run('legacy_action_trace.c'),
                         self.compile_run('legacy_action_trace.c',baseline=True))

    def test_original_protocol_frames_match_main(self):
        self.assertEqual(self.compile_run('protocol_harness.c'),
                         self.compile_run('protocol_harness.c',baseline=True))

    def test_same_firmware_new_to_old_handoff(self):
        self.assertIn('additive handoff passed',self.compile_run('protocol_harness.c',arguments=('handoff',)))

    def test_full_legacy_trace_after_enhanced_use_without_board_reset(self):
        self.assertEqual(self.compile_run('protocol_harness.c',arguments=('after',)),
                         self.compile_run('protocol_harness.c',baseline=True))

    def test_frozen_main_and_legacy_action_sources(self):
        manifest=json.loads((BASE/'manifest.json').read_text(encoding='utf-8'))
        for path,digest in manifest['files'].items():
            self.assertEqual(hashlib.sha256((BASE/path).read_bytes().replace(b'\r\n',b'\n')).hexdigest(),digest)
        for name in ('Core/Src/actions.c','Core/Inc/actions.h'):
            self.assertEqual((BASE/name).read_text(encoding='utf-8'),
                             (FIRMWARE/name).read_text(encoding='utf-8'))
        def stop_body(path):
            s=path.read_text(encoding='utf-8');a=s.index('void Motor3508_StopAll(void)')
            return s[a:s.index('\n}',a)+2]
        self.assertEqual(stop_body(BASE/'Core/Src/motor3508.c'),
                         stop_body(FIRMWARE/'Core/Src/motor3508.c'))
        p=FIX/'protocol_baseline'
        manifest=json.loads((p/'manifest.json').read_text(encoding='utf-8'))
        self.assertEqual(hashlib.sha256((p/'protocol.c').read_bytes().replace(b'\r\n',b'\n')).hexdigest(),manifest['sha256'])

    def test_all_presets_build_the_same_command_contract(self):
        doc=json.loads((FIRMWARE / 'CMakePresets.json').read_text(encoding='utf-8'))
        for preset in doc['configurePresets']:
            self.assertNotIn('UNIFOREST_EXECUTION_PROFILE',preset.get('cacheVariables',{}))
        cmake=(FIRMWARE / 'CMakeLists.txt').read_text(encoding='utf-8')
        self.assertNotIn('UNIFOREST_EXECUTION_ENHANCED=',cmake)
        self.assertIn('Core/Src/execution_session.c',cmake)
