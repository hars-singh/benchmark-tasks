"""Lock a coarse syringe filter and dispense a measured clarified water sample."""

import math
from collections import deque

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_syringe_filter_chalk_water.geometry import Solid

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
X,Y=.354,-.026
BARREL_LOW=TOP+.098
BARREL_H=.074
BARREL_R=.0086
BARREL_INNER=.00725
PLUNGER_TOP=BARREL_LOW+BARREL_H+.055
TARGET_TRAVEL=.0480
MAX_TRAVEL=.056
AREA_ML_PER_M=math.pi*BARREL_INNER**2*1e6
BARREL_FLOOR=BARREL_LOW+.0025
STOPPER_SEAL_Z=BARREL_FLOOR+9.0/AREA_ML_PER_M
STOPPER_OFFSET=PLUNGER_TOP-(STOPPER_SEAL_Z+.002)
FILTER_Z=BARREL_LOW-.018
LOCK_TRAVEL=.00053
LEAD=.00255
DEAD_ML=.22
VIAL_R=.0114
VIAL_H=.048
GLASS=(.69,.78,.84)
PLASTIC=(.84,.87,.86)
GREY=(.25,.27,.29)
BLUE=(.10,.37,.63)
BROWN_FILTER=(.63,.39,.23)
OCHRE=(.67,.57,.36)
SUPPORT=Box('laboratory_syringe_clamp',pos=(X+.118,Y+.037,TOP+.013),size=(.135,.101,.025),mass=1.53,color=GREY)
BARREL=Box('ten_millilitre_luer_syringe',pos=(X,Y,BARREL_LOW+BARREL_H/2),size=(.025,.035,BARREL_H),
           mass=.020,color=PLASTIC)
PLUNGER=Box('syringe_plunger',pos=(X,Y,PLUNGER_TOP),size=(.020,.018,STOPPER_OFFSET+.006),mass=.010,color=PLASTIC)
SEAT=Box('luer_thread_travel_ring',pos=(X,Y,FILTER_Z+.0115),size=(.012,.012,.004),mass=.002,color=GREY)
FILTER=Box('five_micron_syringe_filter',pos=(X,Y,FILTER_Z),size=(.028,.028,.018),mass=.007,color=BROWN_FILTER)
VIAL=Box('marked_collection_vial',pos=(X,Y,TOP+VIAL_H/2),size=(.026,.026,VIAL_H),mass=.029,color=GLASS)
RACK=Box('single_position_vial_rack',pos=(X,Y,TOP+.012),size=(.060,.052,.024),
         mass=.12,color=(.25,.30,.33))
OBJECTS=[SUPPORT,BARREL,PLUNGER,SEAT,FILTER,VIAL,RACK]


def ring(inner,outer,height,z,color,opacity=None):
    step=math.tau/20
    return tuple(Solid('tapered_panel',(inner,inner,outer-inner,height,step),(0.,0.,z),
                       roll=i*step,pitch=-math.pi/2,color=color,opacity=opacity) for i in range(20))


BODY_PARTS={
    SUPPORT.name:(
        Solid('box',SUPPORT.size,color=GREY),
        Solid('cylinder',(.008,.151),(.030,.025,.081),color=GREY),
        Solid('box',(.110,.020,.015),(-.031,.010,.139),color=GREY),
        Solid('box',(.028,.023,.017),(-.095,-.028,.136),color=BLUE),
    ),
    BARREL.name:(
        *ring(BARREL_INNER,BARREL_R,BARREL_H,0.,PLASTIC,.26),
        Solid('cylinder',(.0081,.003),(0.,0.,-.036),color=PLASTIC),
        *ring(.0050,.0067,.006,-.040,PLASTIC),
        Solid('box',(.014,.035,.003),(0.,0.,.0365),color=PLASTIC),
    ),
    PLUNGER.name:(
        Solid('cylinder',(.0096,.0034),color=PLASTIC),
        Solid('box',(.006,.002,STOPPER_OFFSET),(0.,0.,-.5*STOPPER_OFFSET),color=PLASTIC),
        Solid('box',(.002,.006,STOPPER_OFFSET),(0.,0.,-.5*STOPPER_OFFSET),color=PLASTIC),
        Solid('cylinder',(.00719,.004),(0.,0.,-STOPPER_OFFSET),color=GREY),
    ),
    SEAT.name:ring(.0057,.0063,.004,0.,GREY),
    FILTER.name:(
        Solid('cylinder',(.0135,.008),color=BROWN_FILTER),
        Solid('cylinder',(.0054,.007),(0.,0.,.006),color=PLASTIC),
        *ring(.0017,.0032,.011,-.009,PLASTIC),
    ),
    VIAL.name:(
        *ring(VIAL_R,.013,VIAL_H-.003,.0015,GLASS,.23),
        Solid('cylinder',(.0128,.003),(0.,0.,-VIAL_H/2.+.0015),color=GLASS,opacity=.40),
    ),
    RACK.name:(
        *(Solid('box',(.013,.048,.024),(s*.021,0.,0.),color=GREY) for s in (-1,1)),
        *(Solid('box',(.029,.011,.024),(0.,s*.020,0.),color=GREY) for s in (-1,1)),
    ),
}
VISUAL_PARTS={
    BARREL.name:tuple(Solid('box',(.001,.006,.0005),(-.0084,0.,z),color=GREY)
                      for z in np.linspace(-.025,.027,11)),
    FILTER.name:tuple(Solid('box',(.004,.0009,.00028),(.010*math.cos(a),.010*math.sin(a),.0041),
                           yaw=a,color=PLASTIC) for a in (0.,2.094,4.188)),
    VIAL.name:(Solid('box',(.0005,.010,.0016),(-.013,0.,.001),color=BLUE),
               Solid('box',(.0005,.010,.0016),(-.013,0.,-.0013),color=BLUE)),
}
SUPPORT_JOINTS={SUPPORT.name:{'kind':'fixed'},RACK.name:{'kind':'fixed'}}
OBJECT_JOINTS={
    BARREL.name:{'kind':'fixed','parent':SUPPORT.name,'anchor':BARREL.pos},
    PLUNGER.name:{'kind':'prismatic','parent':BARREL.name,'axis':'Z','anchor':PLUNGER.pos,
                 'collision_enabled':False,
                 'lower':-MAX_TRAVEL,'upper':0.,
                 'drive':{'stiffness':520.,'damping':120.,'max_force':1.8}},
    SEAT.name:{'kind':'prismatic','parent':BARREL.name,'axis':'Z','anchor':SEAT.pos,
              'lower':-.00010,'upper':LOCK_TRAVEL+.00008,'drive':{'damping':48.,'max_force':15.}},
    FILTER.name:{'kind':'revolute','parent':BARREL.name,'axis':'Z','anchor':FILTER.pos,
                 'lower':-85.,'upper':3.,'drive':{'damping':.004,'max_force':.045},
                 'screw_coupling':{'carriage':SEAT.name,'travel_per_turn_m':-LEAD}},
}
SURFACE_FRICTION={PLUNGER.name:(.81,.72),FILTER.name:(.77,.68),VIAL.name:(.54,.47)}
CONTACT_OFFSETS_M={obj.name:.00023 for obj in OBJECTS}
RIGID_SOLVER_ITERATIONS={obj.name:(64,8) for obj in (BARREL,PLUNGER,SEAT,FILTER,VIAL)}
OBJECT_MATERIALS={SUPPORT.name:'ordinary bench laboratory clamp and steel base',
                  BARREL.name:'polymer BD Luer-Lok barrel',PLUNGER.name:'polymer plunger and rubber stopper',
                  SEAT.name:'Luer connector seat witness',FILTER.name:'Sartorius Minisart CA 5 micrometre filter',
                  VIAL.name:'borosilicate 15 mL marked vial',RACK.name:'ordinary single-position vial rack'}

camera_pos=np.array((.120,-.246,.279))
camera_forward=np.array((-.118,-.037,.091))-camera_pos
camera_forward/=np.linalg.norm(camera_forward)
camera_right=np.cross(camera_forward,(0.,0.,1.));camera_right/=np.linalg.norm(camera_right)
camera_rot=np.column_stack((camera_right,np.cross(camera_right,camera_forward),-camera_forward))
TASK_RECORD_CAMERA=(SUPPORT.name,Camera(tuple(camera_pos),tuple(matrix_to_quat_xyzw(camera_rot)),
                                        'opengl',Lens.from_focal(20.,640,360)))

LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.5
_flow_history=deque(maxlen=5)
_most_travel=0.
_passed_ml=0.
_spilled_ml=0.
_invalid=False
_levels=None
_compression_angle=None
_filter_drive=None
_plunger_drive=None
_dose_stop_time=None
_dose_stop_tcp=None


def on_reset():
    global _most_travel,_passed_ml,_spilled_ml,_invalid,_compression_angle
    global _dose_stop_time,_dose_stop_tcp
    _flow_history.clear()
    _most_travel=_passed_ml=_spilled_ml=0.
    _invalid=False
    _compression_angle=None
    _dose_stop_time=_dose_stop_tcp=None


def lock_degrees(state):
    q=state.quat_xyzw
    return -float(math.degrees(2.*math.atan2(q[2],q[3])))


def observe(objects,context,dt):
    global _most_travel,_passed_ml,_spilled_ml,_invalid,_compression_angle
    o=context['body_origins']
    travel=float(np.clip(PLUNGER_TOP-o[PLUNGER.name][2],0.,MAX_TRAVEL))
    discharged = max(0.,travel*AREA_ML_PER_M-DEAD_ML)-max(0.,_most_travel*AREA_ML_PER_M-DEAD_ML)
    discharged=max(0.,discharged)
    _most_travel=max(_most_travel,travel)
    sample_position=o[VIAL.name]
    angle=lock_degrees(objects[FILTER.name])
    seal_travel=o[SEAT.name][2]-SEAT.pos[2]
    if angle>=76. and seal_travel>=.00048:
        _compression_angle=max(_compression_angle or 0.,min(angle,84.))
    elif angle<61.:
        _compression_angle=None
    sealed=angle>=71. and seal_travel>=.00044
    upright=quat_xyzw_to_matrix(objects[VIAL.name].quat_xyzw)[2,2]>.972
    over_vial=math.dist(o[FILTER.name][:2],sample_position[:2])<=VIAL_R-.0015
    _flow_history.append(discharged/dt)
    if not upright or sum(_flow_history)/len(_flow_history)>3. or not np.isfinite(discharged):
        _invalid=True
    if sealed and upright and over_vial:
        _passed_ml+=discharged
    else:
        _spilled_ml+=discharged
    if _spilled_ml>.12 or _passed_ml>8.0:
        _invalid=True


def success(objects):
    plunger=LIVE_CONTEXT['body_origins'][PLUNGER.name]
    clear=all(math.dist((X,Y,FILTER_Z),tcp)>.100 and math.dist(plunger,tcp)>.067
              for tcp in LIVE_CONTEXT['tcp'].values())
    current_travel=PLUNGER_TOP-LIVE_CONTEXT['body_origins'][PLUNGER.name][2]
    return bool(not _invalid and 7.0<=_passed_ml<=8.0 and clear
                and current_travel>=_most_travel-.0010 and lock_degrees(objects[FILTER.name])>=71.
                and abs(objects[PLUNGER.name].lin_vel[2])<.0015)


def score(objects):
    if _invalid:
        return 0.
    return 1. if success(objects) else .5 if _passed_ml>=3.5 else 0.


def update_actuators():
    """Approximate the compression friction of the seated Luer taper."""
    global _filter_drive,_plunger_drive
    if _filter_drive is None:
        import omni.usd
        from pxr import UsdPhysics
        stage=omni.usd.get_context().get_stage()
        joint=next(prim for prim in stage.Traverse() if prim.GetName()==FILTER.name+'_joint')
        _filter_drive=UsdPhysics.DriveAPI(joint,'angular')
        joint=next(prim for prim in stage.Traverse() if prim.GetName()==PLUNGER.name+'_joint')
        _plunger_drive=UsdPhysics.DriveAPI(joint,'linear')
    # The dry stopper resists gravity before the compression stroke.
    if _compression_angle is not None and _most_travel>.0022:
        _plunger_drive.GetStiffnessAttr().Set(0.)
        _plunger_drive.GetDampingAttr().Set(185.)
        _plunger_drive.GetMaxForceAttr().Set(21.)
    else:
        _plunger_drive.GetStiffnessAttr().Set(520.)
        _plunger_drive.GetDampingAttr().Set(120.)
        _plunger_drive.GetMaxForceAttr().Set(1.8)
    if _compression_angle is None:
        _filter_drive.GetStiffnessAttr().Set(0.)
        _filter_drive.GetMaxForceAttr().Set(.045)
    else:
        _filter_drive.GetTargetPositionAttr().Set(float(-_compression_angle))
        _filter_drive.GetStiffnessAttr().Set(.175)
        _filter_drive.GetMaxForceAttr().Set(.090)


def update_visuals():
    global _levels
    from pxr import Gf,UsdGeom
    if _levels is None:
        import omni.usd
        stage=omni.usd.get_context().get_stage()
        handles=[]
        for obj,radius,color in ((VIAL,VIAL_R-.001,BLUE),(BARREL,BARREL_INNER-.0005,OCHRE)):
            (parent,)=(p for p in stage.Traverse() if p.GetName()==obj.name)
            level=UsdGeom.Cylinder.Define(stage,str(parent.GetPath())+'/visible_sample')
            level.CreateRadiusAttr(radius);level.CreateHeightAttr(1.)
            level.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            handles.append((level.AddTranslateOp(),level.AddScaleOp()))
        _levels=handles
    vial_h=max(.00002,_passed_ml*1e-6/(math.pi*(VIAL_R-.001)**2))
    remaining=max(.00002,.055-_most_travel)
    for (offset,stretch),z,h in zip(_levels,(-VIAL_H/2.+.003,-BARREL_H/2.+.004),(vial_h,remaining)):
        offset.Set(Gf.Vec3d(0.,0.,z+h/2.));stretch.Set(Gf.Vec3f(1.,1.,h))


PLUNGER_GRIP=(X,Y,PLUNGER_TOP+.004)
PRESS_TARGET=(X,Y,PLUNGER_GRIP[2]-TARGET_TRAVEL-.011)
LEFT_PARK=(.223,.147,PLUNGER_TOP+.029)
FILTER_GRIP=(X-.006,Y,FILTER_Z+.005)
TARGET_START=3.12
TARGET_TURN_S=1.84
TARGET_END=TARGET_START+TARGET_TURN_S+.60


def demo():
    return {
        'right':[
            Waypoint(1.2,(X-.052,Y-.041,FILTER_Z+.071),FINGER_OPEN,'approach the loose filter coupling'),
            Waypoint(1.12,FILTER_GRIP,FINGER_OPEN,'reach the brown filter rim'),
            Waypoint(.80,FILTER_GRIP,-.006,'grasp the filter rim'),
            Waypoint(TARGET_TURN_S,FILTER_GRIP,-.006,'seat the luer lock'),
            Waypoint(.94,FILTER_GRIP,FINGER_OPEN,'release the locked filter'),
            Waypoint(.58,(FILTER_GRIP[0],FILTER_GRIP[1]-.027,FILTER_GRIP[2]+.045),
                     FINGER_OPEN,'lift and withdraw above the collection vial'),
            Waypoint(1.0,(.246,-.208,FILTER_Z+.113),FINGER_OPEN,'move away from the vial'),
            Waypoint(12.,(.246,-.208,FILTER_Z+.113),FINGER_OPEN,'keep the collection area clear'),
        ],
        'left':[
            Waypoint(2.1,(X,Y,PLUNGER_TOP+.068),FINGER_OPEN,'approach the plunger'),
            Waypoint(3.2,(X,Y,PLUNGER_TOP+.068),FINGER_OPEN,'wait while the filter seats'),
            Waypoint(.8,PLUNGER_GRIP,FINGER_OPEN,'reach the thumb flange'),
            Waypoint(.70,PLUNGER_GRIP,-.006,'grasp the flange'),
            Waypoint(8.1,PRESS_TARGET,-.006,'press slowly to collect the marked volume'),
            Waypoint(.65,PRESS_TARGET,FINGER_OPEN,'release plunger'),
            Waypoint(1.2,LEFT_PARK,FINGER_OPEN,'retreat from the vial'),
            Waypoint(2.0,LEFT_PARK,FINGER_OPEN,'verify the collected water'),
        ],
    }


def demo_target(arm,phase,t,position,rotation,finger):
    global _dose_stop_time,_dose_stop_tcp
    if arm=='left':
        pitch=axis_angle_matrix(np.array((0.,1.,0.)),-.48)
        if phase=='press slowly to collect the marked volume' and 'body_origins' in LIVE_CONTEXT:
            if lock_degrees(LIVE_OBJECTS[FILTER.name])<71.:
                return PLUNGER_GRIP,pitch@rotation,FINGER_OPEN
            if _passed_ml>=7.47:
                if _dose_stop_time is None:
                    _dose_stop_time=t
                    _dose_stop_tcp=np.asarray(LIVE_CONTEXT['tcp']['left'])+(0.,0.,.002)
        if _dose_stop_time is not None:
            u=min(1.,max(0.,(t-_dose_stop_time-.65)/1.1))
            return _dose_stop_tcp+u*u*(3.-2.*u)*(np.asarray(LEFT_PARK)-_dose_stop_tcp),pitch@rotation,FINGER_OPEN
        return position,pitch@rotation,finger
    pitch=axis_angle_matrix(np.array((0.,1.,0.)),-1.02*min(1.,t/.9))
    if t>=TARGET_START:
        fraction=min(1.,(t-TARGET_START)/TARGET_TURN_S)
        unwind=min(1.,max(0.,(t-TARGET_END-1.10)/.75))
        yaw=-1.57*fraction*fraction*(3.-2.*fraction)*(1.-unwind)
        turn=axis_angle_matrix(np.array((0.,0.,1.)),yaw)
        centre=np.asarray(FILTER.pos)
        rotated_grip=centre+turn@(np.asarray(FILTER_GRIP)-centre)
        if phase in {'seat the luer lock','release the locked filter'}:
            position=rotated_grip
        elif phase=='lift and withdraw above the collection vial':
            rise=min(1.,max(0.,(t-TARGET_START-TARGET_TURN_S-.94)/.58))
            position=rotated_grip+rise*np.array((-.023,-.034,.044))
        return position,turn@pitch@grip_rotation(),finger
    return position,pitch@rotation,finger
