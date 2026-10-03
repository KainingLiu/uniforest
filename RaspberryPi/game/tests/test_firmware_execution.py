"""Compile and exercise the real firmware motion code with a host HAL.

No robot, serial connection, ARM compiler, or flashing tool is involved. The
test skips explicitly when a supported host C compiler is unavailable.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


from tests.project_paths import PROJECT as ROOT, FIRMWARE
FIXTURES = Path(__file__).resolve().parent / "firmware"


@unittest.skipUnless((FIRMWARE).is_dir(),
                     'A-board source checks run on the development checkout')
class FirmwareExecutionTests(unittest.TestCase):
    def test_compiled_execution_state_machines(self):
        compiler = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
        vcvars = None
        if compiler is None and os.name == "nt":
            compiler = shutil.which("cl")
            if compiler is None:
                visual_studio = Path(os.environ.get("ProgramFiles(x86)",
                                     "C:/Program Files (x86)")) / "Microsoft Visual Studio"
                candidates = sorted(visual_studio.glob("*/*/VC/Auxiliary/Build/vcvars64.bat"))
                if candidates:
                    vcvars, compiler = candidates[-1], "cl"
        if compiler is None:
            self.skipTest("host C compiler unavailable; firmware execution harness not run")

        work = ROOT / "work" / "firmware-host"
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="test-", dir=work) as output:
            build = Path(output).resolve()
            # All generated files and automatic cleanup remain within this workspace.
            build.relative_to(work.resolve())
            executable = build / ("execution_harness.exe" if os.name == "nt" else "execution_harness")
            includes = [FIXTURES, FIRMWARE / "Core/Src", FIRMWARE / "Core/Inc"]
            source = FIXTURES / "execution_harness.c"
            is_msvc = Path(compiler).stem.lower() == "cl"
            if is_msvc:
                args = [compiler, "/nologo", "/utf-8", "/std:c11", "/W1",
                        *(f"/I{path}" for path in includes), f"/Fe:{executable}",
                        f"/Fo:{build / 'execution_harness.obj'}", str(source)]
                if vcvars:
                    batch = build / "compile.cmd"
                    batch.write_text(
                        f'@echo off\ncall "{vcvars}" >nul\n'
                        'if errorlevel 1 exit /b\n'
                        + subprocess.list2cmdline(args) + '\nexit /b %errorlevel%\n',
                        encoding="utf-8",
                    )
                    args = [os.environ.get("COMSPEC", "cmd.exe"), "/d", "/c", str(batch)]
            else:
                args = [compiler, "-std=c11", *(f"-I{path}" for path in includes),
                        str(source), "-o", str(executable)]
            compiled = subprocess.run(args, cwd=build, capture_output=True,
                                      text=True, errors="replace", timeout=60)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            checked = subprocess.run([str(executable)], cwd=build, capture_output=True,
                                     text=True, errors="replace", timeout=30)
            self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)
            self.assertIn("persistent stop passed", checked.stdout)


if __name__ == "__main__":
    unittest.main()
