"""Repack a tested source build with byte-identical dev66 dependency payloads.

Project candidate only, Linux-host wheel (no manylinux certification claim).
No install, network access or mutation of the inputs. Requires explicit patchelf.
"""
import argparse
import base64
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile

VERSION='2.0.1.dev67+gdroidd6.libfranka.0.21.2'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('base-wheel','base-pin','build-receipt','build-dir','patchelf','output-dir'):
        p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=True)
    base=json.loads(a.base_pin.read_text());receipt=json.loads(a.build_receipt.read_text())
    if sha(a.base_wheel)!=base['wheel_sha256']:raise ValueError('Base wheel hash mismatch')
    with tempfile.TemporaryDirectory(dir=a.output_dir) as tmp:
        root=Path(tmp)
        with zipfile.ZipFile(a.base_wheel) as z:
            for name in z.namelist():
                (root/name).resolve().relative_to(root.resolve())
            z.extractall(root)
        for name,h in base['files'].items():
            if sha(root/name)!=h:raise ValueError(f'Base payload mismatch: {name}')
        for name in ('libfranky.so','_franky.cpython-312-x86_64-linux-gnu.so'):
            src=a.build_dir/name
            if sha(src)!=receipt['native_files'][str(src)]:raise ValueError('Build receipt mismatch')
            (root/'franky'/name).write_bytes(src.read_bytes())
        mapping={'libfranka.so.0.21':'libfranka-bfc9e82d.so.0.21.2','libruckig.so':'libruckig-5b3ec43e.so'}
        for name in ('libfranky.so','_franky.cpython-312-x86_64-linux-gnu.so'):
            binary=root/'franky'/name
            needed=subprocess.check_output([str(a.patchelf),'--print-needed',str(binary)],text=True).splitlines()
            for old,new in mapping.items():
                if old in needed:
                    subprocess.run([str(a.patchelf),'--replace-needed',old,new,str(binary)],check=True)
            subprocess.run([str(a.patchelf),'--set-rpath','$ORIGIN:$ORIGIN/../franky_control.libs',str(binary)],check=True)
        # Preserve every bundled dependency byte-for-byte, including libfranka/Ruckig.
        for name,h in base['files'].items():
            if name.startswith('franky_control.libs/') and sha(root/name)!=h:
                raise ValueError(f'Dependency changed: {name}')
        old=next(root.glob('*.dist-info'));info=root/f'franky_control-{VERSION}.dist-info';old.rename(info)
        meta=info/'METADATA';meta.write_text(meta.read_text().replace('Version: '+base['franky_version'],'Version: '+VERSION))
        wheel=info/'WHEEL';lines=[s for s in wheel.read_text().splitlines() if s and not s.startswith(('Tag:', 'Generator:'))]
        lines.append('Generator: franka-runtime-droid-candidate-repack')
        wheel.write_text('\n'.join(lines+['Tag: cp312-cp312-linux_x86_64'])+'\n')
        (info/'RECORD').unlink()
        records=[]
        for f in sorted(root.rglob('*')):
            if f.is_file():
                data=f.read_bytes();records.append((f.relative_to(root).as_posix(),'sha256='+base64.urlsafe_b64encode(hashlib.sha256(data).digest()).decode().rstrip('='),str(len(data))))
        records.append((info.name+'/RECORD','',''));buf=io.StringIO();csv.writer(buf,lineterminator='\n').writerows(records)
        (info/'RECORD').write_text(buf.getvalue())
        output=a.output_dir/f'franky_control-{VERSION}-cp312-cp312-linux_x86_64.whl'
        with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED) as z:
            for f in sorted(root.rglob('*')):
                if f.is_file():z.write(f,f.relative_to(root).as_posix())
        native={name:sha(root/name) for name in base['files']}
        pin=dict(schema_version=1,family='server10',franky_version=VERSION,wheel_sha256=sha(output),files=native,
                 qualification='OFFLINE CANDIDATE; physical commissioning not performed',base_wheel_sha256=base['wheel_sha256'],
                 build_receipt_sha256=sha(a.build_receipt),dependency_payloads_preserved=True,
                 packaging='Repacked tested source build into dev66 Python/dependency payload; native ELF linkage patched; Linux-host wheel')
        (a.output_dir/'droid-native-pin.json').write_text(json.dumps(pin,indent=2)+'\n')
        print(output)


if __name__=='__main__':main()
