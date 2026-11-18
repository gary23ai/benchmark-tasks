"""The run script's bookkeeping, without a simulator or the Dream service: the request cap and the result file."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

import conftest  # noqa: F401

try:
    from rdb_cake_wire import fixed_scene as fixture, run
    from rdb_cake_wire.dream_audit import BudgetedDreamClient, DreamAudit
    from inspect_robots_dream import _station
except ImportError as error:  # pragma: no cover
    raise unittest.SkipTest(f"needs yam_arena, inspect_robots and inspect_robots_dream: {error}")


class FakeDream:
    """Stands in for dream.DreamClient: every reply is 48 rows of zeros with the grippers half open."""

    last_timing, session_id = None, "test"

    def __init__(self):
        self._policy = SimpleNamespace(_smoother=SimpleNamespace(reset=lambda: None))
        self.asked = 0

    def get_action(self):
        self.asked += 1
        chunk = np.zeros((_station.MAX_ROWS, 7))
        chunk[:, 6] = .5
        return {"q_l_chunk": chunk, "q_r_chunk": chunk.copy(), "extra": "kept"}


class RequestCapTest(unittest.TestCase):
    def test_cap_covers_a_full_trial_with_every_request_asked_twice(self):
        replies = -(-fixture.MAX_STEPS // run.DREAM_CHUNK_STEPS)
        self.assertEqual(run.DREAM_CHUNK_STEPS, _station.MAX_ROWS)
        self.assertEqual(replies, 34)
        self.assertEqual(run.DREAM_REQUEST_CAP, 4 * replies)        # the first trial needed 2 per reply and had 64

    def test_reaching_the_cap_holds_the_arms_instead_of_stopping_the_trial(self):
        with tempfile.TemporaryDirectory() as tmp:
            audit = DreamAudit(Path(tmp))
            client = BudgetedDreamClient(FakeDream(), 2, audit)
            target = np.linspace(.1, .8, 14)
            audit.last_target = target
            for _ in range(2):
                self.assertEqual(_station.rows(client.get_action()).shape, (_station.MAX_ROWS, 14))
            self.assertIsNone(client.capped_at)
            audit.tick = 96
            for _ in range(3):                                       # asked again and again: no error, no request
                held = client.get_action()
                np.testing.assert_allclose(_station.rows(held), np.tile(target, (_station.MAX_ROWS, 1)))
            self.assertEqual(held["extra"], "kept")
            self.assertEqual((client.calls, client.native.asked, client.capped_at), (2, 2, 96))
            audit.close()
            events = [json.loads(line) for line in (Path(tmp) / "dream-action-trace.jsonl").read_text().splitlines()]
            self.assertEqual([e["kind"] for e in events].count("request_cap_reached"), 1)

    def test_cap_before_any_reply_is_an_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            audit = DreamAudit(Path(tmp))
            audit.last_target = np.zeros(14)
            with self.assertRaises(RuntimeError):
                BudgetedDreamClient(FakeDream(), 0, audit).get_action()
            audit.close()


class ResultFileTest(unittest.TestCase):
    def result(self, status="success", score=.25, error=None):
        sample = SimpleNamespace(reduced={} if score is None else {fixture.SCORER: score}, error=error, termination_reasons=[])
        return SimpleNamespace(samples=[sample], status=status, error=None)

    def test_a_finished_trial_takes_the_scorers_score(self):
        self.assertEqual(run.outcome(self.result(), None, {"score": .75}), (.25, True, None))

    def test_a_trial_stopped_by_an_error_keeps_the_score_it_had_reached(self):
        score, completed, why = run.outcome(self.result("error", None, "PolicyError: boom"), None, {"score": .5})
        self.assertEqual((score, completed, why), (.5, False, "PolicyError: boom"))
        score, completed, why = run.outcome(None, RuntimeError("simulator died"), {"score": .25})
        self.assertEqual((score, completed, why), (.25, False, "RuntimeError: simulator died"))
        self.assertEqual(run.outcome(None, None, None), (None, False, "Inspect Robots produced no trial"))

    def test_the_result_file_is_written_before_a_failure_is_raised(self):
        source = Path(run.__file__).read_text()
        self.assertLess(source.index('"RESULT.json").write_text'), source.index("if not completed:"))
        self.assertIn("except Exception as error:", source)
        self.assertNotIn('raise RuntimeError("Inspect Robots produced no trial")', source)


if __name__ == "__main__":
    unittest.main()
