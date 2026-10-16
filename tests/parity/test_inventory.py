import importlib.util
import json
from pathlib import Path
import subprocess
import pytest

SCRIPT=Path(__file__).parents[2]/'scripts/migration/inventory_deployment.py'


def repo(path, files):
    path.mkdir()
    subprocess.run(['git','init','-q',str(path)],check=True)
    subprocess.run(['git','-C',str(path),'remote','add','origin','https://example.invalid/offline.git'],check=True)
    for name,content in files.items():
        f=path/name;f.parent.mkdir(parents=True,exist_ok=True);f.write_text(content)
    subprocess.run(['git','-C',str(path),'add','.'],check=True)
    subprocess.run(['git','-C',str(path),'-c','user.name=Offline Fixture','-c','user.email=fixture@example.invalid','commit','-qm','fixture'],check=True)
    return path


def test_inventory_hashes_consumers_provenance_and_dirty_detection(tmp_path):
    spec=importlib.util.spec_from_file_location('inventory',SCRIPT)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    source=repo(tmp_path/'source',{'deployment/franky_runtime/example.py':'import numpy\nfrom franka_motion import config\n',
        'deployment/config/example.yaml':'runtime: fake\noutput: /home/example/logs\n',
        'other/client.py':'import franky_experiment\n'})
    fork=repo(tmp_path/'fork',{'src/controller.cpp':'// offline fixture\n'})
    before=m.build(source,fork)
    assert len(before['entries'])==2
    entry=next(e for e in before['entries'] if e['path'].endswith('example.py'))
    assert entry['imports']==['franka_motion','numpy']
    assert any(c['path']=='other/client.py' for c in before['consumer_references'])
    assert before['source']['dirty_status']==''
    (source/entry['path']).write_text('import math\n')
    after=m.build(source,fork)
    assert after['source']['dirty_status']
    assert next(e for e in after['entries'] if e['path']==entry['path'])['sha256']!=entry['sha256']


def test_captured_manifest_covers_first_slice_and_external_dependencies():
    manifest=json.loads((SCRIPT.parents[2]/'deployment/migration/source-inventory.json').read_text())
    paths={e['path'] for e in manifest['entries']}
    for p in ('deployment/franky_runtime/run_m4_hardware.py',
              'deployment/franky_runtime/franky_experiment/motion_demos/hardware.py',
              'source/franka_motion/franka_motion/kinematics.py',
              'tests/motion_control/test_m4_hardware.py'):
        assert p in paths
    assert 'franky' in manifest['external_repositories']
    assert 'franka_ros2' in manifest['external_repositories']
    for e in manifest['entries']:
        assert len(e['sha256'])==64 and e['destination'] and e['disposition']
