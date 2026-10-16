#!/usr/bin/env python3
"""Read-only source inventory. No imports from inventoried projects or hardware."""
import argparse
import ast
import hashlib
import json
import re
import subprocess
from pathlib import Path


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def destination(path):
    if 'hardware_control_audit/' in path or 'model_audit/' in path:
        return 'legacy-archive', 'archive', 'historical evidence; selected fixtures may be copied with provenance'
    if path.startswith('deployment/reference_governor/'):
        if '/isaaclab.py' in path or 'isaaclab_adapter' in path:
            return 'isaac-tasks/controllers', 'extract', 'simulation-only governor integration'
        return 'franka-runtime/components/reference_governor', 'extract', 'optional numerical package; do not enable in M4'
    if path.startswith('source/franka_motion/'):
        return 'franka-runtime/common/motion; robot-learning/learning; isaac-tasks', 'split', 'remove runtime dependency on franka-motion; classify numerics versus experiment/simulation'
    if path.startswith('tests/'):
        return 'matching-owner/tests', 'split', 'parity tests follow implementation owner'
    if path.startswith('deployment/franky_runtime/'):
        if any(t in path for t in ('policy_worker.py', 'summarize_', 'report_', 'plot_', 'prepare_m4_comparisons.py')):
            return 'robot-learning/learning', 'extract', 'inference or experiment analysis'
        if any(t in path for t in ('coordinator.py', '/runner.py', '/core.py', '/path_runtime.py')):
            return 'franka-runtime; robot-learning/contracts; robot-learning/learning', 'split', 'separate native authority, portable semantics and experiment scheduling'
        return 'franka-runtime', 'extract', 'runtime integration/configuration/docs; historical approvals are not transferable'
    if path.startswith('deployment/franka_policy_bundle/') or path.startswith('deployment/schemas/'):
        return 'robot-learning/contracts; robot-learning/learning; franka-runtime', 'split', 'portable semantics, artifact tooling and runtime admission'
    if path.startswith('scripts/'):
        return 'robot-learning/learning; franka-runtime', 'split', 'export/analysis versus commissioning; preserve CLI consumers'
    if path.endswith('.md'):
        return 'legacy-archive', 'archive', 'retain history; extract current instructions selectively'
    return 'robot-learning/learning; franka-runtime', 'split', 'task assets versus physical profiles'


def build(source, franky, ros=None):
    tracked = git(source, 'ls-files').splitlines()
    selected = [p for p in tracked if p.startswith(('deployment/', 'source/franka_motion/', 'scripts/deployment/', 'tests/deployment/', 'tests/motion_control/'))]
    # Include external consumers, without claiming text search resolves dynamic imports.
    texts = {}
    for p in tracked:
        f = source / p
        if f.is_file() and f.suffix in ('.py', '.md', '.yaml', '.yml', '.toml', '.json', '.txt', '.cpp', '.hpp'):
            texts[p] = f.read_text(errors='replace')
    consumers = []
    for p, s in texts.items():
        hits = [{'line': i, 'tokens': sorted(set(re.findall(r'franka_motion|franky_experiment|franka_policy_bundle|franka_governor|deployment/franky_runtime|deployment/reference_governor', line)))}
                for i, line in enumerate(s.splitlines(), 1)
                if re.search(r'franka_motion|franky_experiment|franka_policy_bundle|franka_governor|deployment/franky_runtime|deployment/reference_governor', line)]
        if hits:
            consumers.append({'path': p, 'sha256': digest(source/p), 'references': hits})
    entries = []
    for p in selected:
        f = source / p
        if not f.is_file():
            raise ValueError(f'Missing tracked extraction input: {p}')
        owner, disposition, rationale = destination(p)
        deps = []
        definitions = []
        if f.suffix == '.py':
            tree = ast.parse(texts.get(p, f.read_text()), filename=p)
            definitions = [n.name for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))]
            deps = sorted(set('.'*n.level + (n.module or '') for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)) |
                          set(a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names))
        s = texts.get(p, '')
        entries.append({'path': p, 'sha256': digest(f), 'bytes': f.stat().st_size,
                        'destination': owner, 'disposition': disposition, 'rationale': rationale,
                        'imports': deps, 'definitions': definitions,
                        'cli_entry_point': "__name__" in texts.get(p, '') and "__main__" in texts.get(p, ''),
                        'host_path_lines': [i for i,l in enumerate(s.splitlines(),1) if re.search(r'/home/|/media/|/opt/ros/',l)],
                        'configuration_consumer': p.endswith(('.yaml','.yml','.json','.toml'))})
    def repository(root):
        return {'commit': git(root,'rev-parse','HEAD'), 'dirty_status': git(root,'status','--porcelain'),
                'origin': git(root,'remote','get-url','origin')}
    external = {'franky': {**repository(franky), 'disposition': 'pin-separate-fork',
        'files': [{'path': p, 'sha256': digest(franky/p)} for p in git(franky,'ls-files').splitlines()
                  if (franky/p).is_file() and p.startswith(('src/','include/','tests/','CMakeLists','pyproject','setup','docs/'))]}}
    if ros:
        external['franka_ros2'] = {**repository(ros), 'disposition': 'defer-optional-backend',
                                  'files': [{'path': p, 'sha256': digest(ros/p)} for p in git(ros,'ls-files').splitlines() if (ros/p).is_file()]}
    return {'schema_version': 1, 'hardware_accessed': False, 'baseline_changed': False,
            'source': repository(source), 'scope': ['deployment','source/franka_motion','scripts/deployment','tests/deployment','tests/motion_control'],
            'entries': entries, 'consumer_references': consumers, 'external_repositories': external,
            'limitations': ['Owner assignments containing multiple destinations require function-level splitting before extraction.',
                'Import/text scan is conservative; dynamic imports and out-of-repository consumers require explicit follow-up.',
                'No existing motion approval or hardware qualification transfers with these sources.']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--franky', type=Path, required=True)
    p.add_argument('--ros', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--verify', action='store_true')
    a = p.parse_args()
    result = build(a.source, a.franky, a.ros)
    if a.verify:
        if json.loads(a.output.read_text()) != result:
            raise SystemExit('Inventory differs: inspect revisions, dirty status, consumers and file hashes')
        print(f'Verified {len(result["entries"])} extraction inputs and external provenance')
    else:
        with a.output.open('x') as f:
            json.dump(result, f, indent=2, sort_keys=True)
            f.write('\n')
        print(f'Inventoried {len(result["entries"])} inputs; {len(result["consumer_references"])} consumer files')


if __name__ == '__main__':
    main()
