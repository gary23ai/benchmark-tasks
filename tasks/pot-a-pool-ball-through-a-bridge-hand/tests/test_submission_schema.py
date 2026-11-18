import hashlib,json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_card_and_recorded_spec():
    card=json.loads((ROOT/'task.json').read_text());ref=json.loads((ROOT/'REFERENCE-SPEC.json').read_text())
    assert {k:v for k,v in card.items() if k!='media'}==ref
    for k,n in [('title',120),('setup',4000),('fidelity',500),('context',1000),('scores',300)]:assert 0<len(card[k])<=n
    if 'media' in card:assert re.fullmatch('M-[A-Z0-9]+',card['media'])
def test_recorded_asset_and_scorer_identity():
    v=json.loads((ROOT/'validation.json').read_text())
    for key,name in [('scene_sha256','fixed_scene.py'),('asset_sha256','pool_table.usda'),('layout_sha256','layout.py'),('grader_sha256','grading.py')]:
        assert hashlib.sha256((ROOT/'rdb_pool_bridge'/name).read_bytes()).hexdigest()==v['recorded_hashes'][key]
