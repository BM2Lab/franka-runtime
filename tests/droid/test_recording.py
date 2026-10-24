import json
from pathlib import Path
import threading
import time
import pytest
from franka_runtime.droid.recording import SessionRecorder


def recorder(tmp_path,**kwargs):
    settings=dict(metadata={'test':True},queue_capacity=32,max_record_bytes=4096,max_event_bytes=100_000,
                  min_free_bytes=1,fsync_interval_s=.01)
    settings.update(kwargs)
    return SessionRecorder(tmp_path/'session',**settings)


def test_passive_until_started(tmp_path):
    r=recorder(tmp_path);assert not r.directory.exists() and r._thread is None
    r.start();assert r.emit('state',{'q':[0.]*7},timestamp_ns=100)
    status=r.close({'stop_confirmed':True},timeout_s=1.)
    assert status['sealed'] and status['writer_joined']
    records=[json.loads(s) for s in (r.directory/'events.jsonl').read_text().splitlines()]
    assert records[0]['sequence']==1 and records[0]['payload']['q']==[0.]*7
    manifest=json.loads((r.directory/'manifest.json').read_text())
    assert manifest['status']=='sealed' and manifest['written']==1 and manifest['terminal']['stop_confirmed']
    with pytest.raises(FileExistsError):recorder(tmp_path).start()


def test_overflow_latches_and_never_seals_success(tmp_path):
    r=recorder(tmp_path,queue_capacity=1)
    entered=threading.Event();release=threading.Event();original=r._run
    def blocked():entered.set();release.wait(1.);original()
    r._run=blocked;r.start();assert entered.wait(1.)
    assert r.emit('a',{},timestamp_ns=1)
    assert not r.emit('b',{},timestamp_ns=2)
    release.set();status=r.close({'stop_confirmed':False},timeout_s=1.)
    assert not status['sealed'] and 'overflow' in status['error']
    manifest=json.loads((r.directory/'manifest.json').read_text())
    assert manifest['status']=='failed' and manifest['rejected']==1


@pytest.mark.parametrize('payload,expected', [({'bad':float('nan')},'compliant'),({'large':'x'*5000},'record_size_limit')])
def test_bad_record_is_not_silent(tmp_path,payload,expected):
    r=recorder(tmp_path);r.start()
    assert not r.emit('bad',payload,timestamp_ns=1)
    s=r.close({},timeout_s=1.)
    assert not s['sealed'] and expected in s['error']


def test_total_file_bound(tmp_path):
    r=recorder(tmp_path,max_record_bytes=4096,max_event_bytes=4096);r.start()
    for i in range(100):
        if not r.emit('a',{'text':'x'*100},timestamp_ns=i):break
    assert 'event_file_size_limit' in r.fault
    r.close({},timeout_s=1.)
    assert (r.directory/'events.jsonl').stat().st_size<=4096


def test_disk_failure_and_unjoined_writer_are_reported(tmp_path):
    r=recorder(tmp_path);r.start()
    # The writer's final manifest fsync/rename must not be treated as success.
    def failed(m):raise OSError('simulated_disk_failure')
    r._atomic_manifest=failed
    result=r.close({},timeout_s=1.)
    assert not result['sealed'] and 'disk_failure' in result['error']


def test_close_timeout_does_not_claim_join_or_complete(tmp_path):
    r=recorder(tmp_path);release=threading.Event();original=r._run
    def blocked():release.wait(1.);original()
    r._run=blocked;r.start()
    result=r.close({'stop_confirmed':False},timeout_s=.01)
    assert not result['writer_joined'] and not result['sealed']
    assert json.loads((r.directory/'manifest.json').read_text())['status']=='open'
    release.set();r._thread.join(1.)
    assert json.loads((r.directory/'manifest.json').read_text())['status']=='failed'


def test_integrity_audit_detects_tampering(tmp_path):
    from franka_runtime.droid.recording import inspect_recording
    r=recorder(tmp_path);r.start();r.emit('native_sample',{'tick':1},timestamp_ns=1);r.close({},timeout_s=1.)
    assert inspect_recording(r.directory)['kinds']=={'native_sample':1}
    p=r.directory/'events.jsonl';p.write_text(p.read_text().replace('"tick":1','"tick":2'))
    with pytest.raises(ValueError,match='digest'):inspect_recording(r.directory)
