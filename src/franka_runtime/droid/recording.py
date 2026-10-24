"""Bounded non-RT JSONL recording; imports and construction open nothing.

Only the explicitly started writer touches disk. A successful close means all
accepted records were fsynced and a terminal manifest was atomically installed.
An open/failed recording is never labelled complete. No camera/token recording.
"""
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import threading
import time


def encode(value):
    return (json.dumps(value,separators=(',',':'),sort_keys=True,allow_nan=False)+'\n').encode()


def native_sample(sample):
    fields=('tick','monotonic_ns','publication_sequence','fault','stop_fault','dropped',
            'stopping','stop_finished','stop_at_rest')
    data={k:getattr(sample,k) for k in fields if hasattr(sample,k)}
    for k in ('q','dq','reference','raw','tracking_raw','outgoing'):
        if hasattr(sample,k):data[k]=[float(x) for x in getattr(sample,k)]
    r=sample.consumed
    data['consumed']={k:int(getattr(r,k)) for k in ('row_sequence','publication_sequence','tick','monotonic_ns')}
    return data


class SessionRecorder:
    def __init__(self,directory,*,metadata,queue_capacity,max_record_bytes,max_event_bytes,
                 min_free_bytes,fsync_interval_s):
        for v in (queue_capacity,max_record_bytes,max_event_bytes,min_free_bytes):
            if type(v) is not int or v<=0:raise ValueError('Explicit positive recording bounds required')
        if max_record_bytes>max_event_bytes:raise ValueError('Record bound exceeds event-file bound')
        if isinstance(fsync_interval_s,bool) or not math.isfinite(fsync_interval_s) or fsync_interval_s<=0:
            raise ValueError('Positive fsync interval required')
        self.directory=Path(directory);self.metadata=json.loads(encode(metadata))
        if len(encode(metadata))>max_record_bytes:raise ValueError('Recording metadata too large')
        self.queue=queue.Queue(queue_capacity);self.max_record_bytes=max_record_bytes
        self.max_event_bytes,self.min_free_bytes=max_event_bytes,min_free_bytes
        self.interval=fsync_interval_s;self._lock=threading.Lock();self._closing=threading.Event()
        self._thread=None;self._error=None;self._accepted=self._written=self._bytes=self._rejected=0
        self._summary=None;self._sealed=False

    @property
    def fault(self):
        with self._lock:return self._error

    def _fail(self,error):
        with self._lock:self._error=self._error or str(error)

    def start(self):
        if self._thread is not None or self._closing.is_set():raise RuntimeError('Recorder is single-use')
        parent=self.directory.parent
        if not parent.is_dir():raise ValueError('Recording parent must already exist')
        if shutil.disk_usage(parent).free<self.min_free_bytes+self.max_event_bytes:
            raise ValueError('Insufficient free space for bounded recording plus reserve')
        self.directory.mkdir(mode=0o700) # Exclusive: never overwrite another session.
        manifest=dict(schema_version=1,status='open',metadata=self.metadata)
        self._atomic_manifest(manifest)
        self._thread=threading.Thread(target=self._run,name='droid-recording',daemon=True)
        self._thread.start()

    def emit(self,kind,payload,*,timestamp_ns):
        """Nonblocking queue admission; False permanently faults the recording."""
        try:
            if not isinstance(kind,str) or not kind or type(timestamp_ns) is not int or timestamp_ns<0:
                raise ValueError('Invalid event kind/time')
            if is_dataclass(payload):payload=asdict(payload)
            with self._lock:
                if self._thread is None or self._closing.is_set() or self._error:
                    raise ValueError('Recorder not active')
                record=encode(dict(schema_version=1,sequence=self._accepted+1,kind=kind,
                                   timestamp_ns=timestamp_ns,payload=payload))
                if len(record)>self.max_record_bytes:raise ValueError('record_size_limit')
                if self._bytes+len(record)>self.max_event_bytes:raise ValueError('event_file_size_limit')
                try:self.queue.put_nowait(record)
                except queue.Full:raise ValueError('recording_queue_overflow') from None
                self._accepted+=1;self._bytes+=len(record)
            return True
        except Exception as error:
            with self._lock:
                self._error=self._error or str(error);self._rejected+=1
            return False

    def _atomic_manifest(self,manifest):
        data=encode(manifest)
        if len(data)>self.max_record_bytes:raise ValueError('Manifest exceeds record bound')
        temp=self.directory/'manifest.tmp'
        with temp.open('wb') as f:f.write(data);f.flush();os.fsync(f.fileno())
        os.replace(temp,self.directory/'manifest.json')
        fd=os.open(self.directory,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)

    def _run(self):
        digest=hashlib.sha256();last_sync=time.monotonic()
        try:
            with (self.directory/'events.jsonl').open('xb') as f:
                while not self._closing.is_set() or not self.queue.empty():
                    try:record=self.queue.get(timeout=min(self.interval,.05))
                    except queue.Empty:record=None
                    if record is not None:
                        f.write(record);digest.update(record)
                        with self._lock:self._written+=1
                    if time.monotonic()-last_sync>=self.interval:
                        f.flush();os.fsync(f.fileno());last_sync=time.monotonic()
                f.flush();os.fsync(f.fileno())
            with self._lock:
                result=dict(schema_version=1,status='failed' if self._error else 'sealed',metadata=self.metadata,
                    accepted=self._accepted,written=self._written,rejected=self._rejected,event_bytes=self._bytes,
                    events_sha256=digest.hexdigest(),error=self._error,terminal=self._summary)
            self._atomic_manifest(result)
            with self._lock:self._sealed=self._error is None
        except Exception as error:self._fail(error)

    def close(self,summary,*,timeout_s):
        if isinstance(timeout_s,bool) or not math.isfinite(timeout_s) or timeout_s<=0:
            raise ValueError('Bounded positive recorder close timeout required')
        encoded=encode(summary)
        if len(encoded)>self.max_record_bytes//2:raise ValueError('Terminal summary too large')
        with self._lock:
            self._summary=json.loads(encoded)
            self._closing.set()  # Atomic with producer admission; no late accepted records.
        if self._thread is not None:self._thread.join(timeout_s)
        if self._thread is None or self._thread.is_alive():self._fail('recording_close_unconfirmed')
        return self.status()

    def status(self):
        with self._lock:return dict(sealed=self._sealed and self._error is None,error=self._error,
            accepted=self._accepted,written=self._written,rejected=self._rejected,
            writer_joined=self._thread is not None and not self._thread.is_alive())


def from_physical_config(config,*,clock_id,contract_sha256):
    """Passive recorder construction; start explicitly before device admission."""
    d=config.require_approved()
    if contract_sha256 not in d['contract']['allowed_sha256'] or not isinstance(clock_id,str) or not clock_id:
        raise ValueError('Recording contract/clock identity mismatch')
    settings=dict(d['recording'])
    directory=config.path(settings.pop('directory'));settings.pop('close_timeout_s')
    return SessionRecorder(directory,metadata=dict(clock_id=clock_id,configuration_sha256=config.sha256,
        contract_sha256=contract_sha256,native_pin_sha256=d['artifacts']['native_pin_sha256']),**settings)


def inspect_recording(directory,*,max_record_bytes=1_048_576):
    """Streaming integrity audit. Unsealed files are evidence, never success."""
    directory=Path(directory)
    with (directory/'manifest.json').open('rb') as f:
        raw=f.read(max_record_bytes+1)
    if len(raw)>max_record_bytes:raise ValueError('Manifest size bound exceeded')
    manifest=json.loads(raw);digest=hashlib.sha256();count=0;kinds={};size=0
    with (directory/'events.jsonl').open('rb') as f:
        while True:
            raw=f.readline(max_record_bytes+1)
            if not raw:break
            if len(raw)>max_record_bytes or not raw.endswith(b'\n'):raise ValueError('Oversized or truncated event')
            record=json.loads(raw)
            if record['sequence']!=count+1:raise ValueError('Recording sequence gap/reordering')
            count+=1;size+=len(raw);digest.update(raw);kinds[record['kind']]=kinds.get(record['kind'],0)+1
    sealed=manifest.get('status')=='sealed'
    if manifest.get('status') in ('sealed','failed'):
        if count!=manifest['written'] or digest.hexdigest()!=manifest['events_sha256']:
            raise ValueError('Event-file digest/count mismatch')
        if count!=manifest['accepted'] or size!=manifest['event_bytes']:
            raise ValueError('Accepted event loss/size mismatch')
    return dict(status=manifest['status'],sealed=sealed,event_count=count,event_bytes=size,kinds=kinds,
                terminal=manifest.get('terminal'),error=manifest.get('error'))


def main():
    import argparse
    p=argparse.ArgumentParser(description='Inspect a bounded DROID session recording; no device access')
    p.add_argument('directory',type=Path);args=p.parse_args()
    print(json.dumps(inspect_recording(args.directory),indent=2))


if __name__=='__main__':main()
