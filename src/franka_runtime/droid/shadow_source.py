"""Read-only sidecar process, explicitly started; importing opens nothing."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time


class ReadOnlyStateSource:
    def __init__(self,helper,sha256,address,*,hz=50,server_version=10,stderr_path,max_stream_lag_ns=40_000_000):
        self.helper=Path(helper).resolve();self.sha256=sha256;self.address=address
        self.hz,self.server_version,self.stderr_path=hz,server_version,Path(stderr_path)
        self._lock=threading.Lock();self._latest=None;self._fault=None
        self.process=self.thread=self._stderr=None;self._closing=False
        self._arm_last=self._hand_last=None
        self._anchors={};self.max_stream_lag_ns=max_stream_lag_ns

    @property
    def fault(self):
        with self._lock:return self._fault

    def snapshot(self):
        with self._lock:
            if self._latest is None:raise ValueError('Waiting for first read-only FCI/Hand sample')
            return dict(self._latest)

    def _accept(self,value):
        # Counters/timestamps may not be refreshed by replaying old SDK packets.
        # Only the arm counter is in milliseconds (measured 1000/s); the Hand's SDK
        # counter is not (measured about 20/s at 10 Hz state, 2026-10-24), so the Hand
        # gets monotonic/restamp checks here and host-age freshness in the backend.
        for group in ('arm','hand'):
            pair=(value[group+'_counter_ms'],value[group+'_ns'])
            if any(type(x) is not int or x<0 for x in pair):raise ValueError('Invalid SDK counter/time')
            millisecond_counter=group=='arm'
            anchor=self._anchors.setdefault(group,pair)
            if millisecond_counter and (pair[1]-anchor[1])-(pair[0]-anchor[0])*1_000_000>self.max_stream_lag_ns:
                raise ValueError('SDK packet stream lags host clock')
            previous=getattr(self,'_'+group+'_last')
            if previous is not None:
                if pair[0]<previous[0] or pair[1]<previous[1]:raise ValueError('Regressed read-only state')
                if pair[0]==previous[0] and pair[1]!=previous[1]:raise ValueError('SDK packet restamped without advancing')
                if millisecond_counter and pair[0]>previous[0]:
                    lag=(pair[1]-previous[1])-(pair[0]-previous[0])*1_000_000
                    if lag>100_000_000:raise ValueError('SDK packet stream lags host clock')
                if not millisecond_counter and pair[0]>previous[0] and pair[1]==previous[1]:
                    raise ValueError('Hand counter advanced without a new read timestamp')
            setattr(self,'_'+group+'_last',pair)
        with self._lock:self._latest=value

    def start(self):
        if self.process is not None or self._closing:raise RuntimeError('Read-only source is single-use')
        if os.environ.get('LD_LIBRARY_PATH') or os.environ.get('LD_PRELOAD'):
            raise ValueError('Clear loader overrides before launching pinned reader')
        if hashlib.sha256(self.helper.read_bytes()).hexdigest()!=self.sha256:raise ValueError('Reader hash mismatch')
        self._stderr=self.stderr_path.open('xb')
        self.process=subprocess.Popen([str(self.helper),self.address,str(self.server_version),str(self.hz)],
            stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=self._stderr)
        def read():
            try:
                while True:
                    line=self.process.stdout.readline(65537)
                    if not line:
                        if not self._closing:raise RuntimeError('Read-only FCI reader exited; inspect reader stderr')
                        break
                    if len(line)>65536 or not line.endswith(b'\n'):raise ValueError('Invalid reader frame')
                    value=json.loads(line);self._accept(value)
            except Exception as e:
                with self._lock:self._fault=repr(e)
        self.thread=threading.Thread(target=read,name='droid-readonly-state',daemon=True);self.thread.start()

    def wait_initial(self,timeout_s):
        end=time.monotonic()+timeout_s
        while time.monotonic()<end:
            if self.fault:raise RuntimeError(self.fault)
            try:return self.snapshot()
            except ValueError:time.sleep(.005)
        raise RuntimeError('Initial read-only state deadline exceeded')

    def stop(self):
        self._closing=True
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=1.)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1.)
            if self.thread is not None:self.thread.join(1.)
            self.process.stdout.close()
        if self._stderr is not None:self._stderr.close()

    @property
    def joined(self):
        return self.process is not None and self.process.poll() is not None and self.thread is not None and not self.thread.is_alive()
