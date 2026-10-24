"""Explicit commissioning adapters. No network, vendor import or implicit engine.

Recorded identity is operator evidence, not a live Desk query. Collision checking
requires an externally vetted engine/model, never a point-only fallback.
"""
import hashlib
import json
import math
from pathlib import Path
from contracts.sessions import payload_sha256

IDENTITY_FIELDS={'schema_version','source','address','model','firmware_version','fci_version',
                 'arm_serial','control_serial','clock_id','observed_ns'}


class RecordedIdentityReader:
    def __init__(self,path,*,sha256):
        self.path,self.sha256=Path(path),sha256

    def __call__(self,address):
        with self.path.open('rb') as f:data=f.read(65537)
        if len(data)>65536 or hashlib.sha256(data).hexdigest()!=self.sha256:
            raise ValueError('Identity evidence size/hash mismatch')
        v=json.loads(data)
        if not isinstance(v,dict) or set(v)!=IDENTITY_FIELDS or type(v['schema_version']) is not int or v['schema_version']!=1:
            raise ValueError('Invalid identity evidence schema')
        for key in IDENTITY_FIELDS-{'schema_version','fci_version','observed_ns'}:
            if not isinstance(v[key],str) or not v[key].strip():raise ValueError('Missing identity field: '+key)
        if type(v['fci_version']) is not int or v['fci_version']<=0 or type(v['observed_ns']) is not int or v['observed_ns']<=0:
            raise ValueError('Invalid identity protocol/time')
        if v['address']!=address:raise ValueError('Identity evidence address mismatch')
        return v  # Preserve original capture time and boot clock; never restamp.


class BoundCollisionChecker:
    def __init__(self,config,engine):
        d=config.validated();w=d['workspace']
        expected=dict(model_sha256=d['artifacts']['collision_model_sha256'],urdf_sha256=w['model_sha256'],
                      joint_names=tuple(w['joint_names']),tool_sha256=payload_sha256(d['tool']))
        for key,value in expected.items():
            if getattr(engine,key,None)!=value:raise ValueError('Collision engine binding mismatch: '+key)
        if not isinstance(getattr(engine,'provenance',None),str) or not engine.provenance.strip():
            raise ValueError('Collision engine provenance required')
        if not callable(getattr(engine,'check_configuration',None)):raise ValueError('Collision engine query missing')
        self.artifact_sha256=expected['model_sha256'];self.engine=engine

    def check_configuration(self,q):
        q=tuple(q)
        if len(q)!=7 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in q):
            raise ValueError('Seven finite encoder/target joints required')
        if self.engine.check_configuration(q) is not True:
            raise ValueError('Collision engine rejected configuration or returned no explicit clearance')
