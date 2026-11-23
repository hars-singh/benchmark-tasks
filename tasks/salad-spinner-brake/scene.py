"""Dry wet lettuce by pumping a stock spinner and stop it with the brake button."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

from dynamic_task_salad_spinner_brake.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
X, Y = .399, .006
R = .133
BASKET_R = .111
PUMP_TRAVEL = .031
BRAKE_TRAVEL = .007
STARTING_WATER_G = 23.
DRY_SCORE_G = 16.
WHITE = (.80,.82,.80)
CLEAR = (.67,.75,.77)
GRAY = (.48,.55,.55)
DARK = (.09,.11,.10)
LEAF_COLORS = ((.27,.51,.12),(.36,.57,.15),(.33,.48,.102),(.47,.62,.19))

BOWL = Box('oxo_32480_clear_spinner_bowl', pos=(X,Y,TOP+.064), size=(2*R,2*R,.128),
           mass=.550, color=CLEAR)
ROTOR = Box('slotted_spinner_basket',pos=(X,Y,TOP+.065),size=(2*BASKET_R,2*BASKET_R,.102),
            mass=.17,color=WHITE)
LID = Box('removable_oxo_lid',pos=(X,Y,TOP+.140),size=(2*R,2*R,.022),mass=.328,color=WHITE)
PUMP = Box('salad_spinner_pump',pos=(X-.010,Y-.015,TOP+.175),size=(.064,.064,.029),mass=.046,color=WHITE)
BRAKE = Box('salad_spinner_brake',pos=(X+.012,Y+.079,TOP+.158),size=(.030,.027,.012),mass=.010,color=DARK)


def leaf_objects():
    leaves=[]
    for i in range(8):
        phase=math.tau*i/8
        distance=.051 if i%2 else .071
        xy=(X+distance*math.cos(phase),Y+distance*math.sin(phase))
        leaves.append(Box(f'lettuce_leaf_{i+1}',pos=(*xy,TOP+.028),size=(.047,.030,.0065),
                          yaw=phase+.31,mass=.011,color=LEAF_COLORS[i%4]))
    return leaves


LEAVES = tuple(leaf_objects())
OBJECTS = [BOWL,ROTOR,LID,PUMP,BRAKE,*LEAVES]
SUPPORT_JOINTS = {BOWL.name:{'kind':'fixed'}}
OBJECT_JOINTS = {
    ROTOR.name:{'kind':'revolute','parent':BOWL.name,'axis':'Z','anchor':ROTOR.pos,
                'drive':{'damping':.053,'max_force':.125}},
    PUMP.name:{'kind':'prismatic','parent':LID.name,'axis':'Z','anchor':PUMP.pos,'lower':-PUMP_TRAVEL,'upper':0.,
               'drive':{'stiffness':225.,'damping':6.0,'max_force':14.}},
    BRAKE.name:{'kind':'prismatic','parent':LID.name,'axis':'Z','anchor':BRAKE.pos,
                'lower':-BRAKE_TRAVEL,'upper':0.,'drive':{'stiffness':730.,'damping':4.0,'max_force':11.}},
}
SURFACE_FRICTION={BOWL.name:(.88,.70),LID.name:(.73,.58),ROTOR.name:(.56,.44),
                  **{leaf.name:(.78,.60) for leaf in LEAVES}}
CONTACT_OFFSETS_M={obj.name:.00055 for obj in OBJECTS}
RIGID_SOLVER_ITERATIONS={obj.name:(64,8) for obj in OBJECTS}


def wall(radius, top_radius, height, center_z, thickness, color, opacity=None):
    n=32
    return tuple(Solid('tapered_panel',(radius,top_radius,thickness,height,math.tau/n),
                       (0.,0.,center_z),pitch=-math.pi/2.,roll=math.tau*i/n,color=color,opacity=opacity)
                 for i in range(n))


def basket_ribs():
    pieces=[]
    for i in range(36):
        angle=math.tau*i/36
        xy=(.102*math.cos(angle),.102*math.sin(angle))
        pieces.append(Solid('box',(.0038,.0060,.087),(*xy,-.0038),yaw=angle,color=WHITE))
        pieces.append(Solid('box',(.012,.007,.004),(.109*math.cos(angle),.109*math.sin(angle),.042),
                           yaw=angle,color=WHITE))
    return tuple(pieces)


BODY_PARTS={
    BOWL.name:(Solid('cylinder',(.111,.006),(0.,0.,-.061),color=CLEAR,opacity=.68),
               *wall(.107,.128,.122,.001,.0033,CLEAR,.38)),
    ROTOR.name:(Solid('cylinder',(.088,.003),(0.,0.,-.041),color=WHITE),
                *basket_ribs()),
    LID.name:(Solid('cylinder',(.110,.006),(0.,0.,.002),color=CLEAR,opacity=.38),
              *wall(.124,.128,.017,-.004,.003,WHITE,.98),
              Solid('cylinder',(.047,.006),(0.,0.,.004),color=WHITE),
              Solid('box',(.024,.021,.004),(-.034,.042,.006),color=GRAY)),
    PUMP.name:(Solid('cylinder',(.032,.013),(0.,0.,.008),color=WHITE),
               Solid('cylinder',(.011,.020),(0.,0.,-.008),color=GRAY)),
    BRAKE.name:(Solid('cylinder',(.0145,.012),color=DARK),),
}
for leaf in LEAVES:
    BODY_PARTS[leaf.name]=(Solid('box',(.039,.028,.004),(0.,0.,.0003),color=leaf.color),
                           Solid('box',(.019,.019,.004),(.015,.004,.002),yaw=.22,color=leaf.color))
VISUAL_PARTS={
    ROTOR.name:tuple(Solid('box',(.012,.0020,.0006),(.091*math.cos(a),.091*math.sin(a),.0437),
                           yaw=a,color=(.43,.55,.39) if i%3 else (.20,.34,.21))
                     for i,a in enumerate(i*math.tau/24 for i in range(24))),
    **{leaf.name:(Solid('box',(.041,.0014,.0005),(0.,0.,.0026),color=(.62,.69,.32)),)
       for leaf in LEAVES},
}
OBJECT_MATERIALS={BOWL.name:'clear plastic OXO bowl and nonslip base',
                  LID.name:'removable white-rim OXO lid',
                  ROTOR.name:'native rotating polymer colander',
                  PUMP.name:'spring-return OXO pump knob',
                  BRAKE.name:'stock spring-return mechanical brake button',
                  **{leaf.name:'native lettuce leaf envelope' for leaf in LEAVES}}

cam=np.array((.296,.196,.235))
view=np.array((.009,.026,.091))-cam
view/=np.linalg.norm(view)
cz=np.cross(view,(0.,0.,1.))
cz/=np.linalg.norm(cz)
TASK_RECORD_CAMERA=(BOWL.name, Camera(tuple(cam),tuple(matrix_to_quat_xyzw(np.column_stack((cz,np.cross(cz,view),-view)))),
                                    'opengl',Lens.from_focal(24.,640,352)))


LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.50
_motor_target=_expelled=0.
_full_strokes=0
_pump_ready=True
_brake_seen=False
_brake_at=None
_first_brake_speed=0.
_in_contact=False
_drive=None
_water=None


def on_reset():
    global _motor_target,_expelled,_full_strokes,_pump_ready,_brake_seen,_brake_at,_first_brake_speed,_in_contact
    _motor_target=_expelled=0.
    _full_strokes=0
    _pump_ready=True
    _brake_seen=_in_contact=False
    _brake_at=None
    _first_brake_speed=0.


def observe(objects,context,dt):
    global _motor_target,_expelled,_full_strokes,_pump_ready,_brake_seen,_brake_at,_first_brake_speed,_in_contact
    origins=context['body_origins']
    lid=origins[LID.name]
    pump_depth=(PUMP.pos[2]-LID.pos[2])-(origins[PUMP.name][2]-lid[2])
    brake_depth=(BRAKE.pos[2]-LID.pos[2])-(origins[BRAKE.name][2]-lid[2])
    speed=abs(objects[ROTOR.name].ang_vel[2])
    _motor_target *= math.exp(-.35*dt)
    _pump_ready |= pump_depth<.005
    if _pump_ready and pump_depth>.025:
        _full_strokes+=1
        _pump_ready=False
        _motor_target=min(12.,_motor_target+6.1)
    if brake_depth>.0041 and speed>1.2:
        _brake_at=context['time_s']
        _first_brake_speed=max(_first_brake_speed,speed)
    _in_contact=brake_depth>.0041
    if _in_contact:
        _motor_target=0.
    if _brake_at is not None and speed<.25 and context['time_s']-_brake_at<1.1:
        _brake_seen=True
    _expelled=min(STARTING_WATER_G,_expelled+dt*max(0.,1.08*(speed-1.2)))


def update_actuators():
    global _drive
    if _drive is None:
        import omni.usd
        from pxr import UsdPhysics

        stage=omni.usd.get_context().get_stage()
        joint=next(prim for prim in stage.Traverse() if prim.GetName()==ROTOR.name+'_joint')
        _drive=UsdPhysics.DriveAPI(joint,'angular')
    _drive.GetTargetVelocityAttr().Set(float(math.degrees(_motor_target)))
    _drive.GetDampingAttr().Set(.24 if _in_contact else .053)
    _drive.GetMaxForceAttr().Set(.24 if _in_contact else .125)


def update_visuals():
    global _water
    from pxr import Gf, UsdGeom

    if _water is None:
        import omni.usd

        stage=omni.usd.get_context().get_stage()
        parent=next(p.GetPath() for p in stage.Traverse() if p.GetName()==BOWL.name)
        vessel=UsdGeom.Cylinder.Define(stage,str(parent)+'/drained_water')
        vessel.CreateRadiusAttr(.105)
        vessel.CreateHeightAttr(1.)
        vessel.CreateDisplayColorAttr([Gf.Vec3f(.306,.47,.46)])
        vessel.CreateDisplayOpacityAttr([.57])
        _water=(vessel.AddTranslateOp(),vessel.AddScaleOp())
    h=max(.00001, _expelled*1.e-6/(math.pi*.105**2))
    shift,scale=_water
    shift.Set(Gf.Vec3d(0.,0.,-.057+h/2))
    scale.Set(Gf.Vec3f(1.,1.,h))


def leaves_in_basket(objects):
    for item in LEAVES:
        pos=LIVE_CONTEXT['body_origins'][item.name]
        if math.hypot(pos[0]-X,pos[1]-Y)>BASKET_R-.004 or not TOP+.013<=pos[2]<=TOP+.123:
            return False
    return True


def success(objects):
    lid=LIVE_CONTEXT['body_origins'][LID.name]
    clear=all(math.dist(tcp,(X,Y,TOP+.158))>.114 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(_expelled>=DRY_SCORE_G and _full_strokes>=3 and _brake_seen
                and abs(objects[ROTOR.name].ang_vel[2])<.22 and math.dist(lid,LID.pos)<.009
                and leaves_in_basket(objects) and clear)


def score(objects):
    if not leaves_in_basket(objects):
        return 0.
    return 1. if success(objects) else .50 if _expelled>=DRY_SCORE_G and _full_strokes>=3 else 0.


PUMP_HIGH=np.asarray(PUMP.pos)+(0.,0.,.071)
PUMP_RELEASE=np.asarray(PUMP.pos)+(.002,0.,.029)
PUMP_PUSH=np.asarray(PUMP.pos)+(.002,0.,-.030)
BRAKE_HIGH=np.asarray(BRAKE.pos)+(0.,0.,.091)
BRAKE_PUSH=np.asarray(BRAKE.pos)+(.001,-.001,-.024)
PARK_RIGHT=(.244,-.258,TOP+.216)
PARK_LEFT=(.249,.285,TOP+.219)


def demo():
    pump=[Waypoint(1.55,tuple(PUMP_HIGH),-.005,'approach the broad pump')]
    for _ in range(5):
        pump.extend((
            Waypoint(.65,tuple(PUMP_PUSH),-.005,'press the pump through its stroke'),
            Waypoint(.25,tuple(PUMP_PUSH),-.005,'hold the pump at the stop'),
            Waypoint(.59,tuple(PUMP_RELEASE),-.005,'release the pump to spring back'),
        ))
    pump.extend((Waypoint(1.13,PARK_RIGHT,FINGER_OPEN,'clear the pump'),
                 Waypoint(6.5,PARK_RIGHT,FINGER_OPEN,'wait until the basket has stopped')))
    brake=[
        Waypoint(1.25,PARK_LEFT,FINGER_OPEN,'wait outside the spinning bowl'),
        Waypoint(8.15,PARK_LEFT,FINGER_OPEN,'allow the pump strokes to finish'),
        Waypoint(1.04,tuple(BRAKE_HIGH),-.005,'approach the brake'),
        Waypoint(.61,tuple(BRAKE_PUSH),-.005,'press the stock brake'),
        Waypoint(.69,tuple(BRAKE_PUSH),-.005,'hold until the basket stops'),
        Waypoint(1.05,PARK_LEFT,FINGER_OPEN,'withdraw from the brake'),
        Waypoint(3.9,PARK_LEFT,FINGER_OPEN,'leave the dry greens in the stopped basket'),
    ]
    return {'right':pump,'left':brake}


def demo_target(arm,phase,time,position,rotation,finger):
    return position,axis_angle_matrix(np.array((0.,1.,0.)),-.48)@grip_rotation(),finger
