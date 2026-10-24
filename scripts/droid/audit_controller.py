"""Offline source-expression parity, NOT a native callback or hardware test.

Needs NumPy, g++ and Eigen headers; never imports Franky/Torch or opens FCI.
Downloads are separate. --sources is the saved sources.json directory.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
from types import SimpleNamespace

import numpy as np

FWD = 'fairo/polymetis/polymetis/python/torchcontrol/modules/feedback.py'
CPP = 'src/motion/joint_impedance_base.cpp'
GAINS = dict(Kq=[40,30,50,25,35,25,10], Dq=[4,6,5,5,3,2,1],
             Kx=[400,400,400,15,15,15], Dx=[37,37,37,2,2,2])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def upstream_forward(source):
    """Execute just upstream's unchanged forward body using ndarray operators.

    No upstream imports/constructor/TorchScript are executed. Remove annotations
    only; assert the supported arithmetic syntax rather than executing arbitrary code.
    """
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'HybridJointSpacePD')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'forward')
    for a in fn.args.args:
        a.annotation = None
    fn.returns = None
    allowed = (ast.FunctionDef, ast.arguments, ast.arg, ast.Expr, ast.Constant,
               ast.Assign, ast.Name, ast.Store, ast.Load, ast.BinOp, ast.MatMult,
               ast.Add, ast.Sub, ast.Attribute, ast.Return)
    if any(not isinstance(n, allowed) for n in ast.walk(fn)):
        raise ValueError('Upstream forward syntax changed; review before running')
    scope = {'__builtins__': {}}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), '<pinned HybridJointSpacePD.forward>', 'exec'), scope)
    return scope['forward']


def oracle(case):
    q, dq, qref, J = (np.array(case[k]) for k in ('q','dq','qref','J'))
    Kq,Dq,Kx,Dx = (np.diag(case[k]) for k in ('Kq','Dq','Kx','Dx'))
    return (Kq+J.T@Kx@J)@(qref-q) - (Dq+J.T@Dx@J)@dq


def generate_cases():
    rng=np.random.default_rng(615)
    out=[]
    for i in range(32):
        q=rng.uniform(-1,1,7);dq=rng.uniform(-.2,.2,7)
        e=rng.uniform(-.25,.25,7);J=rng.normal(0,.3,(6,7))
        name=f'coupled-{i:02d}'
        if i==0: name='zero'; e[:]=0;dq[:]=0
        if i==1: name='j7-only';e[:6]=0;dq[:]=0
        if i==2: name='translation-only';J[3:]=0
        if i==3: name='rotation-only';J[:3]=0
        if i==4: name='rank-deficient';J[3:]=J[:3]
        if i==5: name='zero-jacobian';J[:]=0
        if i==6: name='clip-boundary';e[:]=.5
        if i==7: name='outside-clip';e[:]=.8
        c=dict(name=name,q=q.tolist(),dq=dq.tolist(),qref=(q+e).tolist(),J=J.tolist(),**GAINS)
        c['droid_feedback_nm']=oracle(c).tolist()
        out.append(c)
    return out


def run_audit(sources, franky, work, fixture):
    work.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((sources/'sources.json').read_text())
    for entry in manifest:
        # Saved paths are receipts, not commands. Resolve only inside supplied root.
        path=(sources/entry['file']).resolve()
        path.relative_to(sources.resolve())
        if digest(path)!=entry['sha256']: raise ValueError(f'Source changed: {path}')
    forward=upstream_forward((sources/FWD).read_text())
    source=(franky/CPP).read_text()
    expr=re.search(r'tau_d =\s*(current_stiffness_\.cwiseProduct\(q_error\).*?torque_feedforward);',source,re.S)
    if expr is None: raise ValueError('Franky expression changed; review extraction')
    # Compile the unmodified expression from the audited source, with supplied J
    # and effective gains. This does NOT execute the installed wheel or its model.
    program='''#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
using V=Eigen::Matrix<double,7,1>;
using M=Eigen::Matrix<double,6,6>;
int main(){ int count; std::cin>>count;
for(int n=0;n<count;++n){ V q_error,dq_error,current_stiffness_,current_damping_,torque_feedforward=V::Zero();
Eigen::Matrix<double,6,7> jacobian; struct {M stiffness,damping;} shaping;
for(auto* v:{&q_error,&dq_error,&current_stiffness_,&current_damping_}) for(int i=0;i<7;++i) std::cin>>(*v)[i];
shaping.stiffness.setZero();shaping.damping.setZero();
for(int i=0;i<6;++i) std::cin>>shaping.stiffness(i,i);
for(int i=0;i<6;++i) std::cin>>shaping.damping(i,i);
for(int i=0;i<6;++i) for(int j=0;j<7;++j) std::cin>>jacobian(i,j);
V tau_d=EXPRESSION;
for(int i=0;i<7;++i) std::cout<<std::setprecision(17)<<tau_d[i]<<' ';std::cout<<'\\n';}}
'''.replace('EXPRESSION',expr[1])
    cpp=work/'feedback_expression.cpp';cpp.write_text(program)
    exe=work/'feedback_expression'
    subprocess.run(['g++','-std=c++17','-O2','-I/usr/include/eigen3',str(cpp),'-o',str(exe)],check=True)
    cases=generate_cases();rows=[]
    for c in cases:
        e=np.clip(np.array(c['qref'])-c['q'],-.5,.5)
        rows.append(np.concatenate([e,-np.array(c['dq']),c['Kq'],c['Dq'],c['Kx'],c['Dx'],np.array(c['J']).ravel()]))
    data=str(len(rows))+'\n'+'\n'.join(' '.join(map(str,row)) for row in rows)+'\n'
    native=np.fromstring(subprocess.check_output([str(exe)],input=data,text=True),sep=' ').reshape(-1,7)
    maxerr=0.;clipdiff=0.
    for c,tau in zip(cases,native):
        gain=SimpleNamespace(Kq=np.diag(c['Kq']),Kqd=np.diag(c['Dq']),Kx=np.diag(c['Kx']),Kxd=np.diag(c['Dx']))
        q,dq,qr,J=(np.array(c[k]) for k in ('q','dq','qref','J'))
        up=forward(gain,q,dq,qr,np.zeros(7),J)
        np.testing.assert_allclose(up,c['droid_feedback_nm'],rtol=1e-12,atol=1e-12)
        # Franky's error_clip applies before both joint and shaped proportional terms.
        expected=forward(gain,q,dq,q+np.clip(qr-q,-.5,.5),np.zeros(7),J)
        np.testing.assert_allclose(tau,expected,rtol=1e-12,atol=1e-12)
        if c['name']=='outside-clip': clipdiff=float(np.max(np.abs(up-tau)))
        else: maxerr=max(maxerr,float(np.max(np.abs(up-tau))))
        c['franky_clipped_feedback_nm']=tau.tolist()
    assert clipdiff>1
    result=dict(schema_version=1,scope='supplied-J algebraic feedback only; no wheel callback/model/processing execution',
                franky_revision=subprocess.check_output(['git','-C',str(franky),'rev-parse','HEAD'],text=True).strip(),
                source_hashes={CPP:digest(franky/CPP),FWD:digest(sources/FWD)},sources=manifest,
                max_unclipped_difference_nm=maxerr,outside_clip_difference_nm=clipdiff,cases=cases)
    fixture.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('scope','max_unclipped_difference_nm','outside_clip_difference_nm')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('sources','franky-source','work-dir','output'): p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run_audit(a.sources,a.franky_source,a.work_dir,a.output)
