"""Streaming M4 log checks; no replayed force/torque is ever transmitted."""
import json
import math
from pathlib import Path
import numpy as np
from ..common.motion.kinematics import UrdfKinematicsProvider


def replay_servo(path, model_path):
    model=UrdfKinematicsProvider(model_path)
    summary={'hardware_executed':False,'historical_replay':True,'rows':0,'native_faults':{},
             'valid_fk_rows':0,'prediction_rows':0,'max_fk_position_error_m':0.,
             'max_filtered_prediction_error_nm':0.,'stop_finished_seen':False,
             'nonfinite_rows':0}
    last_tick=-1;last_timestamp=-1
    alpha=.001/(.001+1/(2*math.pi*100))
    with Path(path).open() as stream:
        while True:
            line=stream.readline(1_048_577)
            if not line:break
            if len(line)>1_048_576:raise ValueError('Oversized servo record')
            row=json.loads(line,parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON constant')))
            tick=row['tick'];stamp=row['monotonic_ns']
            if type(tick) is not int or type(stamp) is not int or tick<=last_tick or stamp<last_timestamp or stamp<0:
                raise ValueError('Nonmonotonic servo records')
            last_tick,last_timestamp=tick,stamp;summary['rows']+=1
            if row.get('fault'):
                key=str(row['fault']);summary['native_faults'][key]=summary['native_faults'].get(key,0)+1
            if row.get('nonfinite_fields'):summary['nonfinite_rows']+=1
            if row.get('ee_feedback_valid') is True and row.get('pose') is not None:
                pose=np.asarray(row['pose'],dtype=float)
                if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('Invalid recorded pose')
                p,_,_=model.evaluate(row['q'])
                summary['max_fk_position_error_m']=max(summary['max_fk_position_error_m'],float(np.linalg.norm(p-pose[:3,3])))
                summary['valid_fk_rows']+=1
            if all(row.get(k) is not None for k in ('torque_previous','slew','filtered_prediction')):
                old,slew,pred=(np.asarray(row[k],dtype=float) for k in ('torque_previous','slew','filtered_prediction'))
                if any(v.shape!=(7,) for v in (old,slew,pred)):raise ValueError('Invalid torque shape')
                if all(np.isfinite(v).all() for v in (old,slew,pred)):
                    summary['max_filtered_prediction_error_nm']=max(summary['max_filtered_prediction_error_nm'],float(np.max(np.abs(pred-(old+alpha*(slew-old))))))
                    summary['prediction_rows']+=1
            summary['stop_finished_seen'] |= row.get('stop_finished') is True
    if not summary['rows']:raise ValueError('Empty servo log')
    summary['numerical_checks_passed']=(summary['max_fk_position_error_m']<=1e-5 and summary['max_filtered_prediction_error_nm']<=1e-10)
    summary['qualification']='Historical numerical consistency only; not physical stop qualification'
    return summary
