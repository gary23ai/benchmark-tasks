import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
extra = [os.environ["YAM_ARENA"]] if os.environ.get("YAM_ARENA") else []     # a yam-isaac-lab-arena checkout
for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *extra):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
