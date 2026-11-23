"""Prime a stock aquarium siphon and interrupt flow at the bucket band."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera,Lens
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from dynamic_task_aquarium_siphon_prime_and_stop.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
TX,TY=.390,.172
TANK_X,TANK_Y=.242,.222
BASE_Z=TOP+.007
INITIAL_LEVEL=TOP+.123
BUCKET_X,BUCKET_Y=.416,-.215
BUCKET_R=.080
BUCKET_H=.106
BUCKET_BED=TOP+.004
OUTLET=np.array((BUCKET_X+.044,BUCKET_Y+.040,TOP+.043))
TUBE_H=.127
TUBE_R=.0157
TUBE_RESET=(TX-.022,TY-.024,BASE_Z+TUBE_H/2+.0004)
LOW_L,HIGH_L=.35,.45
TUBE_FLOW_X=TX+.003
GLASS=(.64,.76,.79)
PLASTIC=(.72,.84,.87)
BLUE=(.105,.32,.57)
WATER=(.16,.51,.77)
STONE=(.25,.22,.16)
TANK=Box('glass_aquarium',pos=(TX,TY,TOP+.076),size=(TANK_X,TANK_Y,.152),color=GLASS)
BUCKET=Box('marked_water_bucket',pos=(BUCKET_X,BUCKET_Y,TOP+BUCKET_H/2),
           size=(.167,.167,BUCKET_H),color=(.64,.77,.82))
INTAKE=Box('self_priming_intake_tube',pos=TUBE_RESET,size=(.034,.034,TUBE_H+.014),mass=.047,color=PLASTIC)
GRAVEL=tuple(Box(f'gravel_{i:02}',pos=(TX+.019+(i%5)*.022,TY-.063+(i//5)*.041,BASE_Z+.0075),
                 size=(.015,.015,.015),mass=.0045,color=STONE) for i in range(15))
OBJECTS=[TANK,BUCKET,INTAKE,*GRAVEL]


def wall(inner,outer,height,z,color,opacity):
    span=math.tau/22
    return tuple(Solid('tapered_panel',(inner,inner,outer-inner,height,span),(0.,0.,z),
                       roll=i*span,pitch=-math.pi/2,color=color,opacity=opacity) for i in range(22))


BODY_PARTS={
    TANK.name:(
        Solid('box',(TANK_X,TANK_Y,.007),(0.,0.,-.0725),color=(.28,.37,.40)),
        *(Solid('box',(.0045,TANK_Y,.145),(s*(TANK_X/2-.00225),0.,.0035),color=GLASS,opacity=.20)
          for s in (-1,1)),
        *(Solid('box',(TANK_X-.009,.0045,.145),(0.,s*(TANK_Y/2-.00225),.0035),color=GLASS,opacity=.20)
          for s in (-1,1)),
    ),
    BUCKET.name:(
        Solid('cylinder',(.082,.004),(0.,0.,-BUCKET_H/2+.002),color=PLASTIC,opacity=.53),
        *wall(BUCKET_R,.083,BUCKET_H-.004,.002,PLASTIC,.22),
        Solid('box',(.025,.012,.018),(.063,.047,.032),color=BLUE),
        Solid('box',(.018,.017,.006),(.044,.040,.041),color=BLUE),
    ),
    INTAKE.name:(
        *wall(.0138,TUBE_R,TUBE_H,0.,PLASTIC,.24),
        Solid('cylinder',(.0156,.007),(0.,0.,.056),color=PLASTIC,opacity=.40),
        Solid('cylinder',(.0057,.015),(0.,0.,.066),color=BLUE),
    ),
    **{g.name:(Solid('sphere',(.0075,),color=STONE),) for g in GRAVEL},
}
VISUAL_PARTS={
    TANK.name:tuple(Solid('box',(TANK_X-.008,.0007,.0036),(0.,s*(TANK_Y/2+.0001),.074),color=BLUE)
                    for s in (-1,1)),
    BUCKET.name:tuple(Solid('box',(.00060,.024,.0014),(-.083,0.,BUCKET_BED+level*1e-3/(math.pi*BUCKET_R**2)-BUCKET.pos[2]),
                            color=BLUE) for level in (LOW_L,HIGH_L)),
}
OBJECT_MATERIALS={TANK.name:'glass aquarium with a compact gravel bed',BUCKET.name:'transparent plastic bucket and stock siphon clip',
                  INTAKE.name:'Aqueon 5 inch self-priming intake tube',**{o.name:'inert stone gravel' for o in GRAVEL}}
SURFACE_FRICTION={INTAKE.name:(.76,.64),TANK.name:(.45,.34)}
CONTACT_OFFSETS_M={INTAKE.name:.0005,**{o.name:.0008 for o in GRAVEL}}
RIGID_SOLVER_ITERATIONS={INTAKE.name:(64,8)}

cam_p=np.array((-.310,-.420,.459))
cam_f=np.array((.033,-.155,-.018))-cam_p;cam_f/=np.linalg.norm(cam_f)
cam_r=np.cross(cam_f,(0.,0.,1.));cam_r/=np.linalg.norm(cam_r)
cam_rot=np.column_stack((cam_r,np.cross(cam_r,cam_f),-cam_f))
TASK_RECORD_CAMERA=(TANK.name,Camera(tuple(cam_p),tuple(matrix_to_quat_xyzw(cam_rot)),
                                     'opengl',Lens.from_focal(21.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.50
_primed=False
_cycles=0
_rise=False
_low=None
_peak=None
_litres=0.
_flow=0.
_invalid=False
_clear_since_priming=False
_lift_began=None
_graphics=None


def on_reset():
    global _primed,_cycles,_rise,_low,_peak,_litres,_flow,_invalid,_clear_since_priming,_lift_began
    _primed=_rise=_invalid=_clear_since_priming=False
    _cycles=0
    _low=_peak=_lift_began=None
    _litres=_flow=0.


def mouth(objects,context):
    r=quat_xyzw_to_matrix(objects[INTAKE.name].quat_xyzw)
    return context['body_origins'][INTAKE.name]-TUBE_H/2*r[:,2]


def waterline():
    return INITIAL_LEVEL-_litres*.001/((TANK_X-.009)*(TANK_Y-.009))


def observe(objects,context,dt):
    global _primed,_cycles,_rise,_low,_peak,_litres,_flow,_invalid,_clear_since_priming
    x,y,z=map(float,mouth(objects,context))
    inside=abs(x-TX)<TANK_X/2-.009 and abs(y-TY)<TANK_Y/2-.009
    submerged=inside and z<waterline()-.007
    if not submerged:
        if _primed:
            _clear_since_priming=True
        _primed=_rise=False
        _low=_peak=None
        _cycles=0
    elif not _primed:
        _low=z if _low is None else min(_low,z)
        if z-_low>=.018:
            _rise=True
        if _rise:
            _peak=z if _peak is None else max(z,_peak)
            if _peak-z>=.018:
                _cycles+=1
                _rise=False
                _low,_peak=z,None
                _primed=_cycles>=2
    head=max(0.,waterline()-OUTLET[2])
    hose_d,length,friction=.010,1.83,.038
    velocity=math.sqrt(2.*9.81*head/(1.+friction*length/hose_d))
    _flow=1000.*math.pi*(hose_d/2.)**2*velocity if _primed else 0.
    _litres+=_flow*dt
    _invalid=_invalid or _litres>HIGH_L or any(
        abs(context['body_origins'][o.name][0]-TX)>TANK_X/2-.004
        or abs(context['body_origins'][o.name][1]-TY)>TANK_Y/2-.004 for o in GRAVEL)


def success(objects):
    return bool(not _invalid and _clear_since_priming and LOW_L<=_litres<=HIGH_L and not _primed
                and mouth(objects,LIVE_CONTEXT)[2]>waterline()+.010)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .5 if _litres>=.18 else 0.


def update_visuals():
    global _graphics
    from pxr import Gf,UsdGeom,Vt
    if _graphics is None:
        import omni.usd
        stage=omni.usd.get_context().get_stage()
        (root,)=(p.GetPath() for p in stage.Traverse() if p.GetName()==TANK.name)
        hose=UsdGeom.BasisCurves.Define(stage,str(root)+'/PVC_hose')
        hose.CreateTypeAttr('linear');hose.CreateCurveVertexCountsAttr([20])
        hose.CreateWidthsAttr([.012]);hose.SetWidthsInterpolation('constant')
        hose.CreateDisplayColorAttr([Gf.Vec3f(.19,.37,.47)])
        fill=UsdGeom.Cylinder.Define(stage,str(root)+'/bucket_volume')
        fill.CreateRadiusAttr(BUCKET_R-.002);fill.CreateHeightAttr(1.)
        fill.CreateDisplayColorAttr([Gf.Vec3f(*WATER)]);fill.CreateDisplayOpacityAttr([.55])
        stream=UsdGeom.Cylinder.Define(stage,str(root)+'/outlet_flow')
        stream.CreateRadiusAttr(.0027);stream.CreateHeightAttr(1.)
        stream.CreateDisplayColorAttr([Gf.Vec3f(*WATER)])
        surface=UsdGeom.Cube.Define(stage,str(root)+'/aquarium_waterline')
        surface.CreateSizeAttr(1.)
        surface.CreateDisplayColorAttr([Gf.Vec3f(*WATER)])
        surface.CreateDisplayOpacityAttr([.17])
        surf_p=surface.AddTranslateOp();surface.AddScaleOp().Set(Gf.Vec3f(TANK_X-.011,TANK_Y-.011,.0011))
        _graphics=(hose,fill.AddTranslateOp(),fill.AddScaleOp(),stream,stream.AddTranslateOp(),stream.AddScaleOp(),surf_p)
    hose,fill_p,fill_s,stream,stream_p,stream_s,surf_p=_graphics
    base=np.asarray(TANK.pos);rotation=quat_xyzw_to_matrix(LIVE_OBJECTS[INTAKE.name].quat_xyzw)
    start=LIVE_CONTEXT['body_origins'][INTAKE.name]+(TUBE_H/2+.010)*rotation[:,2]
    p1=np.array((TX+.098,TY-.154,max(float(start[2])+.055,TOP+.206)))
    p2=np.array((BUCKET_X+.111,BUCKET_Y+.077,TOP+.161))
    points=[((1.-u)**3*start+3.*u*(1.-u)**2*p1+3.*u*u*(1.-u)*p2+u**3*OUTLET)-base
            for u in np.linspace(0.,1.,20)]
    hose.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float,p)) for p in points]))
    surf_p.Set(Gf.Vec3d(0.,0.,float(waterline()-base[2])))
    h=max(.00001,float(_litres*.001/(math.pi*(BUCKET_R-.002)**2)))
    fill_p.Set(Gf.Vec3d(*map(float,(BUCKET_X-base[0],BUCKET_Y-base[1],BUCKET_BED-base[2]+h/2.))))
    fill_s.Set(Gf.Vec3f(1.,1.,h))
    if _flow>0.:
        span=max(.001,float(OUTLET[2]-BUCKET_BED-h));stream.MakeVisible()
        stream_p.Set(Gf.Vec3d(*map(float,(OUTLET[0]-base[0],OUTLET[1]-base[1],OUTLET[2]-base[2]-span/2.))))
        stream_s.Set(Gf.Vec3f(1.,1.,span))
    else:
        stream.MakeInvisible()


GRIP_Z=float(TUBE_RESET[2]+.058)
START=(TUBE_RESET[0],TUBE_RESET[1],GRIP_Z)
HOVER=(START[0],START[1],GRIP_Z+.065)
RAISED=(START[0],START[1],GRIP_Z+.032)
LOWERED=(START[0],START[1],GRIP_Z+.004)
FLOW=(TUBE_FLOW_X,TY+.007,GRIP_Z+.027)
READ_START=7.4


def demo():
    return {'left':[
        Waypoint(1.35,HOVER,FINGER_OPEN,'approach the siphon intake'),
        Waypoint(.87,START,FINGER_OPEN,'reach the intake cap'),
        Waypoint(.65,START,-.010,'grasp the siphon intake'),
        Waypoint(.73,RAISED,-.010,'lift the immersed tube to prime'),
        Waypoint(.68,LOWERED,-.010,'lower the tube below water'),
        Waypoint(.73,RAISED,-.010,'lift the immersed tube again'),
        Waypoint(.68,LOWERED,-.010,'complete the second priming cycle'),
        Waypoint(1.2,FLOW,-.010,'keep the tube above the gravel'),
        Waypoint(25.,FLOW,-.010,'siphon to the mark then lift the intake clear'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    global _lift_began
    pitch=axis_angle_matrix(np.array((0.,1.,0.)),-.65)
    if arm=='left' and t>=READ_START:
        if _lift_began is None and _litres>=.374:
            _lift_began=t
        if _lift_began is not None:
            u=min(1.,max(0.,(t-_lift_began)/1.25))
            position=np.asarray(FLOW)+(0.,0.,.131*u*u*(3.-2.*u))
    return position,pitch@rotation,finger
