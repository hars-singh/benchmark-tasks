"""Wax the inverted foot ring of a bisque cup on a slowly turning pottery wheel."""

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_pottery_foot_wax_band.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
X, Y = .421, .036
WHEEL_RADIUS = .1397
WHEEL_FACE = TOP + .135
FOOT_Z = WHEEL_FACE + .087
INNER, OUTER = .0325, .0405
SECTORS = 48
IRON = (.45, .51, .55)
HOUSING = (.89,.89,.86)
CLAY = (.83, .71, .57)
BRISTLE_COLOR = (.69, .60, .42)
WAX = (.27, .61, .49)
BASE = Box('tabletop_pottery_wheel_motor_and_basin', pos=(X,Y,TOP+.061), size=(.340,.366,.122),
           mass=7.2,color=HOUSING)
WHEEL = Box('powered_aluminium_pottery_head', pos=(X,Y,WHEEL_FACE-.009), size=(.2794,.2794,.018),
            mass=2.86,color=IRON)
CUP = Box('inverted_bisque_cup', pos=(X,Y,WHEEL_FACE+.042), size=(.106,.106,.087),
          mass=.295, color=CLAY)
BRUSH = Box('loaded_quarter_inch_flat_brush', pos=(.249,.271,TOP+.012),
            size=(.142,.019,.018), mass=.021, color=(.39,.23,.14))
TIP_OFFSET = np.array((.079,0.,0.))
TIP = Box('flat_wax_brush_bristles', pos=tuple(np.asarray(BRUSH.pos)+TIP_OFFSET),
          size=(.016,.0063,.003), mass=.0016, color=BRISTLE_COLOR)
REST = Box('brush_rest', pos=(.253,.275,TOP+.002), size=(.165,.062,.004),
           color=(.29,.34,.35))
OBJECTS = [BASE,WHEEL,CUP,REST,BRUSH,TIP]


def arc(inner, outer, z, height, phase, width, color):
    return Solid('tapered_panel',(inner,inner,outer-inner,height,width),(0.,0.,z),
                 roll=phase,pitch=-math.pi/2.,color=color)


BODY_PARTS = {
    BASE.name: (
        Solid('box',(.310,.310,.087),(0.,0.,-.014),color=HOUSING),
        Solid('cylinder',(.178,.016),(0.,0.,.035),color=HOUSING),
        *(arc(.178,.184,.054,.040,i*math.tau/32.,math.tau/32.,HOUSING) for i in range(32)),
        Solid('box',(.028,.066,.005),(-.117,-.128,.032),color=(.22,.30,.32)),
    ),
    WHEEL.name: (
        Solid('cylinder',(WHEEL_RADIUS,.017),(0.,0.,.0005),color=IRON),
        Solid('cylinder',(.124,.007),(0.,0.,-.0095),color=(.31,.35,.39)),
    ),
    CUP.name: (
        *(Solid('tapered_panel',(.048,.036,.005,.075,math.tau/24.),
                (0.,0.,-.004),roll=i*math.tau/24.,pitch=-math.pi/2.,color=CLAY)
          for i in range(24)),
        Solid('cylinder',(.042,.006),(0.,0.,.036),color=CLAY),
        *(arc(INNER,OUTER,.043,.004,i*math.tau/24.,math.tau/24.,CLAY) for i in range(24)),
    ),
    BRUSH.name: (
        Solid('capsule_x',(.0088,.103),(-.018,0.,0.),color=(.46,.28,.17)),
        Solid('box',(.031,.0085,.0062),(.055,0.,0.),color=(.64,.66,.65)),
    ),
}
VISUAL_PARTS = {
    WHEEL.name: (
        *(arc(r-.00030,r,.00912,.00018,i*math.tau/24.,math.tau/24.,(.33,.37,.42))
          for i in range(24) for r in (.073,.093,.117)),
        Solid('box',(.010,.0058,.00022),(.103,0.,.00918),color=(.71,.35,.17)),
    ),
    CUP.name: (
        Solid('box',(.007,.0012,.0003),(.036,0.,.04519),color=(.43,.31,.23)),
        *(Solid('sphere',(.0062,),(.052*math.cos(a),.052*math.sin(a),-.040),color=(.67,.48,.32))
          for a in (.36,.36+math.tau/3.,.36+2.*math.tau/3.)),
    ),
}
SUPPORT_JOINTS = {BASE.name:{'kind':'fixed'}}
OBJECT_JOINTS = {
    WHEEL.name: {'kind':'revolute','parent':BASE.name,'axis':'Z','anchor':WHEEL.pos,
                 'drive':{'damping':.31,'max_force':1.5,'target_velocity':180.}},
    CUP.name:{'kind':'fixed','parent':WHEEL.name,'anchor':CUP.pos,
              'break_force_N':13.,'break_torque_Nm':.24},
    TIP.name: {'kind':'revolute','parent':BRUSH.name,'axis':'Y',
               'anchor':tuple(np.asarray(TIP.pos)+(-.0074,0.,0.)),
               'lower':-27.,'upper':24.,
               'drive':{'stiffness':.011,'damping':.0008,'max_force':.028}},
}
SURFACE_FRICTION = {WHEEL.name:(.93,.79),CUP.name:(.66,.53),BRUSH.name:(1.05,.84),TIP.name:(.42,.30)}
CONTACT_OFFSETS_M = {CUP.name:.00024, TIP.name:.00014, BRUSH.name:.00030}
OBJECT_MATERIALS = {
    BASE.name:'full-size tabletop VEVOR pottery wheel housing and detachable basin',
    WHEEL.name:'279.4 mm powered VEVOR aluminium wheelhead',
    CUP.name:'bisque stoneware with a flat foot ring, held by three soft clay dabs',
    BRUSH.name:'wood and metal quarter-inch flat paintbrush',
    TIP.name:'flexible bristles loaded with water-based wax resist',
    REST.name:'ordinary brush rest',
}

camera = np.array((-.186,-.178,.319))
forward = np.array((-.008,.004,.146))-camera
forward /= np.linalg.norm(forward)
right = np.cross(forward,(0.,0.,1.))
right /= np.linalg.norm(right)
rotation = matrix_to_quat_xyzw(np.column_stack((right,np.cross(right,forward),-forward)))
TASK_RECORD_CAMERA = (BASE.name,Camera(tuple(camera),tuple(rotation),'opengl',Lens.from_focal(24.,640,360)))

LIVE_CONTEXT = {}
SUCCESS_HOLD_S = 1.
MAX_PARTIAL_SCORE = .5
_coated = np.zeros(SECTORS,dtype=bool)
_paint_prims = None
_smear = False
_parked = False
_steering = np.zeros(3)
_grasp_offset=None
_grasp_rotation=None


def on_reset():
    global _smear,_parked,_grasp_offset,_grasp_rotation
    _coated.fill(False)
    _steering.fill(0.)
    _smear = _parked = False
    _grasp_offset=_grasp_rotation=None


def brush_touch(objects,context):
    tip = np.asarray(context['body_origins'][TIP.name])
    r = quat_xyzw_to_matrix(objects[TIP.name].quat_xyzw)
    return tip+r@np.array((.0055,0.,-.0014))


def observe(objects,context,dt):
    global _smear,_parked
    centre = np.asarray(context['body_origins'][CUP.name])
    rot = quat_xyzw_to_matrix(objects[CUP.name].quat_xyzw)
    _smear |= np.linalg.norm(centre[:2]-(X,Y))>.005 or rot[2,2]<.985
    contact = brush_touch(objects,context)
    radial = rot.T@(contact-centre)
    radius = np.linalg.norm(radial[:2])
    tip_rotation=quat_xyzw_to_matrix(objects[TIP.name].quat_xyzw)
    bristle_bend=quat_xyzw_to_matrix(objects[BRUSH.name].quat_xyzw).T@tip_rotation
    pressure=abs(bristle_bend[2,0])>.027
    if pressure and .0435<radial[2]<.0485 and INNER-.001<radius<OUTER+.001 and rot[2,2]>.985:
        relative = rot.T@tip_rotation
        if abs(objects[CUP.name].ang_vel[2])>.13 and abs(relative[2,0])<.42:
            phase = math.atan2(radial[1],radial[0])%math.tau
            sector = phase*SECTORS/math.tau
            _coated[int(round(sector))%SECTORS] = True
            _coated[int(math.floor(sector))%SECTORS] = True
    if pressure and .041<radial[2]<.047 and (radius<INNER-.002 or OUTER+.002<radius<OUTER+.008):
        _smear=True
    work = (X,Y,FOOT_Z)
    _parked = math.dist(work,contact)>.075 and all(math.dist(work,tcp)>.120 for tcp in context['tcp'].values())


def update_visuals():
    global _paint_prims
    from pxr import Gf, UsdGeom
    if _paint_prims is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        cup = next(p for p in stage.Traverse() if p.GetName()==CUP.name)
        edges = np.linspace(0.,math.tau,SECTORS+1)
        _paint_prims=[]
        for i,(a,b) in enumerate(zip(edges,edges[1:])):
            mesh = UsdGeom.Mesh.Define(stage,str(cup.GetPath())+f'/wet_wax_{i:02d}')
            mesh.CreatePointsAttr([Gf.Vec3f(r*math.cos(t),r*math.sin(t),.04532)
                                   for r,t in [(INNER,a),(OUTER,a),(OUTER,b),(INNER,b)]])
            mesh.CreateFaceVertexCountsAttr([4])
            mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*WAX)])
            visibility=mesh.CreateVisibilityAttr('invisible')
            _paint_prims.append(visibility)
    for visible,painted in zip(_paint_prims,_coated):
        if painted and visible.Get()!='inherited':
            visible.Set('inherited')


def success(objects):
    return bool(not _smear and _parked and _coated.all())


def score(objects):
    if _smear:
        return 0.
    if success(objects):
        return 1.
    window=SECTORS//2
    covered=np.tile(_coated,2).astype(int)
    partial=np.max(np.convolve(covered,np.ones(window,dtype=int),mode='valid'))==window
    return .5 if partial else 0.


GRIP = np.asarray(BRUSH.pos)+(-.014,0.,.010)
CONTACT = np.array((X-(INNER+OUTER)/2.,Y,FOOT_Z-.0010))
TARGET = CONTACT - (TIP_OFFSET+(.0055,0.,-.0014)) + (0.,0.,.005)
LEFT_PITCH = axis_angle_matrix(np.array((0.,1.,0.)),-.35)
PINCH=0.


def demo():
    return {'left':[
        Waypoint(1.12,tuple(GRIP+(0.,0.,.065)),FINGER_OPEN,'approach the loaded brush'),
        Waypoint(.71,tuple(GRIP),FINGER_OPEN,'reach the brush handle'),
        Waypoint(.90,tuple(GRIP),PINCH,'grip the brush handle'),
        Waypoint(.98,tuple(GRIP+(0.,0.,.197)),PINCH,'lift the brush above the wheel'),
        Waypoint(1.30,tuple(TARGET+(0.,0.,.035)),PINCH,'bring the brush to the raised foot'),
        Waypoint(.78,tuple(TARGET),PINCH,'touch the foot ring with the bristles'),
        Waypoint(7.8,tuple(TARGET),PINCH,'hold the brush while the cup turns'),
        Waypoint(.75,tuple(TARGET+(0.,0.,.062)),PINCH,'lift the brush off the rotating cup'),
        Waypoint(1.3,(.222,.244,TOP+.174),PINCH,'show the wax ring with hands clear'),
    ]}


def demo_target(arm,phase,t,position,rotation,finger):
    global _grasp_offset,_grasp_rotation
    if arm=='left':
        default=LEFT_PITCH@rotation
        if 3.34<t<3.69 and 'tcp_rotation' in LIVE_CONTEXT:
            origin=np.asarray(LIVE_CONTEXT['body_origins'][BRUSH.name])
            if origin[2]>TOP+.073:
                grip_r=quat_xyzw_to_matrix(LIVE_OBJECTS[BRUSH.name].quat_xyzw)
                _grasp_offset=grip_r.T@(np.asarray(LIVE_CONTEXT['tcp']['left'])-origin)
                _grasp_rotation=grip_r.T@LIVE_CONTEXT['tcp_rotation']['left']
        drawing={'bring the brush to the raised foot','touch the foot ring with the bristles',
                 'hold the brush while the cup turns','lift the brush off the rotating cup'}
        if phase in drawing and _grasp_offset is not None:
            brush_r=axis_angle_matrix(np.array((0.,1.,0.)),.16)
            tip_local=TIP_OFFSET+(.0055,0.,-.0014)
            reference=CONTACT+brush_r@(_grasp_offset-tip_local)
            reference[0]-=.006
            reference[2]+=max(0.,position[2]-TARGET[2])
            if phase=='hold the brush while the cup turns':
                discrepancy=CONTACT-brush_touch(LIVE_OBJECTS,LIVE_CONTEXT)
                _steering[:]=np.clip(_steering+.09*discrepancy,(-.011,-.010,-.005),(.011,.010,.009))
                reference+=_steering+.49*np.clip(discrepancy,-.009,.009)
            weight=min(1.,max(0.,(t-3.71)/1.05))
            orientation=brush_r@_grasp_rotation
            return np.asarray(position)*(1.-weight)+reference*weight,orientation,finger
        return position,default,finger
    return position,rotation,finger
