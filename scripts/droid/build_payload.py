"""Build a per-rig payload (libfranka "load") file from operator measurements.

The payload is everything mounted on the end effector that Desk's end-effector profile
does not include (e.g. a wrist camera and its mount). Component masses are weighed;
the combined COM is measured in the TCP frame and converted to the flange frame with the
F_T_EE of a read-only state capture. Components without their own COM are lumped at
that point. Inertia is the optional camera body box about the COM (point masses add
none at a shared point); it is small and only needs to be plausible.
The runtime sets this load at connect and verifies the robot reports it.
"""
import argparse
import hashlib
import json
from pathlib import Path


def build(*,name,components,com_tcp_m,state,box_m=None,box_component=None,basis):
    if not components or any(m<=0 for _,m in components):raise ValueError('Positive component masses required')
    s=json.loads(Path(state).read_text());c=s['F_T_EE']  # column-major 4x4
    R=[[c[col*4+row] for col in range(3)] for row in range(3)];t=[c[12],c[13],c[14]]
    com=[sum(R[i][j]*com_tcp_m[j] for j in range(3))+t[i] for i in range(3)]
    mass=sum(m for _,m in components)
    inertia=[[0.]*3 for _ in range(3)]
    if box_m is not None:
        m=dict(components)[box_component];a,b,h=box_m
        for i,v in enumerate((b*b+h*h,a*a+h*h,a*a+b*b)):inertia[i][i]=m*v/12
    return dict(schema_version=1,name=name,m_load=round(mass,6),F_x_Cload=[round(v,6) for v in com],
                I_load=[round(inertia[row][col],9) for col in range(3) for row in range(3)],  # column-major
                components=[dict(name=n,mass_kg=m) for n,m in components],com_tcp_m=list(com_tcp_m),
                inertia_model=None if box_m is None else dict(box_m=list(box_m),component=box_component,frame='flange axes'),
                F_T_EE_source=dict(path=str(Path(state).resolve()),sha256=hashlib.sha256(Path(state).read_bytes()).hexdigest()),
                basis=basis)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name',required=True)
    p.add_argument('--component',action='append',required=True,metavar='NAME=KG')
    p.add_argument('--com-tcp-m',type=float,nargs=3,required=True)
    p.add_argument('--state',type=Path,required=True,help='read-only robot state JSON (F_T_EE)')
    p.add_argument('--box-m',type=float,nargs=3,help='camera body box size for its inertia')
    p.add_argument('--box-component')
    p.add_argument('--basis',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error(f'{a.output} exists')
    components=[(n,float(m)) for n,m in (x.split('=',1) for x in a.component)]
    value=build(name=a.name,components=components,com_tcp_m=a.com_tcp_m,state=a.state,box_m=a.box_m,
                box_component=a.box_component,basis=a.basis)
    a.output.write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value,indent=2))


if __name__=='__main__':main()
