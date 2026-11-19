"""run.py imports without yam_arena (run 6's Dream-1 trial stopped there) and finds it from the job environment."""

import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

ROOT = Path(__file__).resolve().parents[1]

BLOCK = textwrap.dedent("""
    import importlib.abc, sys
    class Block(importlib.abc.MetaPathFinder):
        def __init__(self, allowed):
            self.allowed = allowed
        def find_spec(self, name, path=None, target=None):
            if name == "yam_arena" or name.startswith("yam_arena."):
                for finder in sys.meta_path:
                    if finder is self or not hasattr(finder, "find_spec"):
                        continue
                    spec = finder.find_spec(name, path, target)
                    if spec is not None and self.allowed and str(spec.origin or "").startswith(self.allowed):
                        return spec
                raise ModuleNotFoundError("No module named 'yam_arena'")
            return None
""")


def run(code, env_extra=None):
    env = dict(os.environ, **(env_extra or {}))
    env["PYTHONPATH"] = str(ROOT)
    return subprocess.run([sys.executable, "-c", BLOCK + textwrap.dedent(code)], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=120)


def have_runtime():
    probe = subprocess.run([sys.executable, "-c", "import inspect_robots.guard, inspect_robots.logging"],
                           capture_output=True)
    return probe.returncode == 0


pytestmark = pytest.mark.skipif(not have_runtime(), reason="needs the inspect_robots runtime")


def test_run_module_imports_without_yam_arena():
    out = run("""
        sys.meta_path.insert(0, Block(None))
        import rdb_harmonium.run as run
        assert "rdb_harmonium.adapter" not in sys.modules and "yam_arena" not in sys.modules
        try:
            run.ensure_yam_arena()
        except ModuleNotFoundError as error:
            print("missing ok", "sys.path=" in str(error))
    """, {"YAM_UPSTREAM": "", "ARENA_DIR": ""})
    assert out.returncode == 0, out.stderr
    assert "missing ok True" in out.stdout


def test_yam_arena_found_from_the_job_environment(tmp_path):
    fake = tmp_path / "upstream"
    (fake / "yam_arena").mkdir(parents=True)
    (fake / "yam_arena" / "__init__.py").write_text("FAKE = True\n")
    out = run(f"""
        sys.meta_path.insert(0, Block({str(fake)!r}))
        import rdb_harmonium.run as run
        added = run.ensure_yam_arena()
        import yam_arena
        print("found", added == {str(fake)!r}, getattr(yam_arena, "FAKE", False))
    """, {"YAM_UPSTREAM": str(fake)})
    assert out.returncode == 0, out.stderr
    assert "found True True" in out.stdout
