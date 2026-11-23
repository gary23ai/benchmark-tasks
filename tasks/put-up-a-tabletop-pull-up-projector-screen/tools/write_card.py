"""Write task.json from one place. Usage: python tools/write_card.py --scores "Dream-1 run: ...; demo: ..." [--media M-...]"""

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scores", required=True)
    ap.add_argument("--media")
    args = ap.parse_args()
    card = json.loads((ROOT / "task.json").read_text())
    card["scores"] = args.scores
    if args.media:
        card["media"] = args.media
    (ROOT / "task.json").write_text(json.dumps(card, indent=2) + "\n")
    print({k: len(v) for k, v in card.items() if isinstance(v, str)})


if __name__ == "__main__":
    main()
