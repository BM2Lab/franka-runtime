"""Data-only transfer admission; verification never imports bundle code."""
import hashlib
import json
from pathlib import Path


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def verify_motion_bundle(root, expected_manifest_sha256, *, allow_legacy_cache=False):
    """Anchor identity and exact file inventory. Hashes are not authorization."""
    root = Path(root).resolve(strict=True)
    manifest_path = root / 'manifest.json'
    if manifest_path.is_symlink() or sha256(manifest_path) != expected_manifest_sha256:
        raise ValueError('Transfer manifest hash mismatch')
    manifest = json.loads(manifest_path.read_text(), object_pairs_hook=_unique)
    if not isinstance(manifest, dict):
        raise ValueError('Bundle manifest must be an object')
    files = manifest.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('Nonempty bundle file inventory required')
    for name, digest in files.items():
        p = Path(name)
        if (not isinstance(name, str) or p.is_absolute() or '..' in p.parts or
                p.as_posix() != name or name == 'manifest.json' or
                not isinstance(digest, str) or len(digest) != 64 or
                any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('Invalid bundle file path/digest')
    actual = set()
    for p in root.rglob('*'):
        if p.is_symlink():
            raise ValueError('Bundle symlinks are not permitted')
        if p.is_file() and p != manifest_path:
            actual.add(p.relative_to(root).as_posix())
    ignored = set()
    if allow_legacy_cache and manifest.get('artifact_format') is None:
        ignored = {name for name in actual-set(files)
                   if Path(name).parent.as_posix() == 'franka_policy_bundle/__pycache__'
                   and Path(name).stem.split('.')[0] in ('__init__','contract','bundle')
                   and name.endswith('.pyc')}
    if set(files) != actual-ignored:
        raise ValueError('Bundle file inventory mismatch')
    for name, digest in files.items():
        if sha256(root/name) != digest:
            raise ValueError(f'Bundle integrity failure: {name}')
    return manifest
