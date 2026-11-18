"""Feed a pencil into a moving mounted sharpener while turning its crank.

The pencil, spring-loaded cutter face, crank and freely spinning knob are
ordinary contact bodies. A disclosed material-removal rule changes only the
visible pencil tip after eight loaded crank turns. No task body is teleported.
"""
import math
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY, FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from dynamic_task_moving_crank_sharpener.geometry import Solid
from dynamic_task_moving_crank_sharpener.motion import OscillatingPlatform
from dynamic_task_moving_crank_sharpener.scoring import _rotate

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TOOL = Box("tool", pos=(.245,-.04,TOP+.12), size=(.06,.07,.07), mass=.50, color=(.15,.30,.65))
STAGE = Box("stage", pos=(.245,-.04,TOP+.015), size=(.18,.14,.085), mass=5., color=(.28,.28,.30))
WHEEL = Box("wheel", pos=(.285,-.04,TOP+.12), size=(.008,.09,.09), mass=.080, color=(.75,.20,.15))
KNOB = Box("knob", pos=(.331,-.04,TOP+.085), size=(.030,.016,.016), mass=.008, color=(.12,.12,.14))
CUTTER = Box("cutter", pos=(.232,-.04,TOP+.12), size=(.003,.008,.008), mass=.005, color=(.45,.45,.48))
PENCIL = Box("pencil", pos=(.120,.10,TOP+.0335), size=(.140,.007,.007), mass=.006, color=(.95,.68,.10))
CRADLES = [Box("cradle_left",pos=(.055,.10,TOP+.015),size=(.016,.04,.03),color=(.28,.28,.30)),
           Box("cradle_right",pos=(.160,.10,TOP+.015),size=(.016,.04,.03),color=(.28,.28,.30))]
OBJECTS = [STAGE,TOOL,WHEEL,KNOB,CUTTER,PENCIL,*CRADLES]
HOLE = .009
BAR = (.070-HOLE)/2
SHIFT = (.070+HOLE)/4
BODY_PARTS = {
    "stage": (Solid("box",(.18,.14,.030)),Solid("box",(.025,.050,.055),(0,0,.0425))),
    "tool": (Solid("box",(.060,BAR,.070),(0,-SHIFT,0)),
             Solid("box",(.060,BAR,.070),(0,SHIFT,0)),
             Solid("box",(.060,HOLE,BAR),(0,0,-SHIFT)),
             Solid("box",(.060,HOLE,BAR),(0,0,SHIFT))),
    "wheel": (Solid("cylinder",(.008,.018),axis="X"),
              Solid("box",(.008,.012,.043),(0,0,-.0175)),
              Solid("cylinder",(.003,.040),(.020,0,-.035),axis="X")),
    "knob": (Solid("cylinder",(.008,.030),axis="X"),),
    "pencil": (Solid("cylinder",(.0035,.140),axis="X"),),
}
MOTIONS = {"stage":OscillatingPlatform(STAGE.pos,(0,.005,0),8.0)}
SUPPORT_JOINTS = {"stage":{"axis":"Y","lower":-.020,"upper":.020}}
OBJECT_JOINTS = {
    "tool":{"kind":"fixed","parent":"stage","anchor":TOOL.pos},
    "wheel":{"kind":"revolute","parent":"tool","anchor":WHEEL.pos,"axis":"X",
             "drive":{"damping":.020,"max_force":.08}},
    "knob":{"kind":"revolute","parent":"wheel","anchor":KNOB.pos,"axis":"X",
            "drive":{"damping":.0001,"max_force":.005}},
    "cutter":{"kind":"prismatic","parent":"tool","anchor":CUTTER.pos,"axis":"X",
              "lower":0.,"upper":.006,"drive":{"stiffness":500.,"damping":3.,"max_force":5.}},
}
TRACK_OBJECTS={"left":"pencil","right":"knob"}
TRACK_PHASES={"left":{"to pencil","descend pencil","grip pencil"},
              "right":{"to knob","descend knob","grip knob","hold knob","wait feed"}}
SIDE=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
GRIP_OFFSET=np.array((0,-.008,0))
SUCCESS_HOLD_S=1.0
MAX_PARTIAL_SCORE=.75
LIVE_CONTEXT={"tcp":{},"body_origins":{},"gripper":{}}
_previous=-math.pi/2
_loaded_rotation=0.
_loaded_by_end={1:0.,-1:0.}
_fed=False
_sharpened=False
_cut_end=1
_material_updated=False
_pencil_offset=None

def on_reset():
    global _previous,_loaded_rotation,_loaded_by_end,_fed,_sharpened,_material_updated,_pencil_offset,_cut_end
    _previous=-math.pi/2;_loaded_rotation=0.;_fed=False;_sharpened=False
    _material_updated=False;_pencil_offset=None
    _cut_end=1
    _loaded_by_end={1:0.,-1:0.}

def feed_state(objects,context):
    """Actual tip alignment and passive spring displacement, independent of policy."""
    root=context['body_origins']['pencil'];tool=context['body_origins']['tool']
    axis=np.asarray(_rotate((1,0,0),tuple(objects['pencil'].quat_xyzw)))
    # Both blunt ends look the same. Accept whichever end points into the tool,
    # rather than requiring the policy to infer an invisible body-frame sign.
    end_sign=1 if axis[0]>=0 else -1
    tip=root+end_sign*.070*axis
    depression=context['body_origins']['cutter'][0]-(tool[0]-.013)
    aligned=abs(axis[0])>math.cos(math.radians(5)) and np.linalg.norm(tip[1:]-tool[1:])<.0012
    cutter_face=context['body_origins']['cutter'][0]-.0015
    inserted=.229 <= tip[0] <= .242 and abs(tip[0]-cutter_face)<.0012
    # A 500 N/m passive spring displaced 2 mm gives a nominal 1 N axial load.
    return bool(aligned and inserted and .002<=depression<=.0065),float(depression),tip,end_sign

def observe(objects,context,dt):
    global _previous,_loaded_rotation,_fed,_sharpened,_pencil_offset,_cut_end
    delta=context['body_origins']['knob']-context['body_origins']['wheel']
    angle=math.atan2(delta[2],delta[1]);increment=(angle-_previous+math.pi)%(2*math.pi)-math.pi
    _previous=angle
    loaded,_,_,end_sign=feed_state(objects,context)
    if loaded:
        _fed=True
        _loaded_by_end[end_sign]=max(0.,_loaded_by_end[end_sign]-increment)
        _loaded_rotation=max(_loaded_by_end.values())
        if _loaded_by_end[end_sign]>=16*math.pi and not _sharpened:
            _sharpened=True;_cut_end=end_sign
    if 4.8<context['time_s']<7.0 and objects['pencil'].low[2]>TOP+.08:
        offset=context['body_origins']['pencil']-context['tcp']['left']
        _pencil_offset=offset if _pencil_offset is None else .9*_pencil_offset+.1*offset

def success(objects):
    p=objects['pencil'];root=LIVE_CONTEXT['body_origins']['pencil']
    axis=np.asarray(_rotate((1,0,0),tuple(p.quat_xyzw)))
    clear=all(np.linalg.norm(root-v)>.10 for v in LIVE_CONTEXT['tcp'].values())
    returned=abs(root[0]-.120)<.015 and abs(root[1]-.10)<.012 and TOP+.029<=p.low[2]<=TOP+.034
    return bool(_sharpened and returned and abs(axis[0])>math.cos(math.radians(10)) and p.resting() and clear)

def score(objects):
    if objects['pencil'].low[2]<TOP-.03:return 0.
    if success(objects):return 1.
    if _sharpened:return .75
    return .5 if _loaded_rotation>=8*math.pi else .25 if _fed else 0.

def update_materials():
    """Visible cutting response only; body pose, mass and collider stay untouched."""
    global _material_updated
    if not _sharpened or _material_updated:return
    import omni.usd
    from pxr import UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
    stage=omni.usd.get_context().get_stage()
    bodies=[p for p in stage.Traverse() if p.GetName()=='pencil' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies)!=1:raise RuntimeError('expected one pencil body')
    root=str(bodies[0].GetPath())+'/geometry'
    UsdGeom.Imageable(stage.GetPrimAtPath(root+'/solid_0')).MakeInvisible()
    for name,kind,length,radius,offset,color in [
        ('cut_shaft','cylinder',.120,.0035,(-_cut_end*.010,0,0),(.95,.68,.10)),
        ('cut_wood','cone',.020,.0035,(_cut_end*.060,0,0),(.72,.49,.26)),
        ('cut_graphite','cone',.003,.000525,(_cut_end*.0685,0,0),(.07,.07,.08))]:
        shape=(UsdGeom.Cylinder if kind=='cylinder' else UsdGeom.Cone).Define(stage,root+'/'+name)
        shape.CreateAxisAttr('X');shape.CreateHeightAttr(length);shape.CreateRadiusAttr(radius)
        shape.AddTranslateOp().Set(Gf.Vec3d(*offset))
        if _cut_end<0 and kind=='cone':shape.AddRotateZOp().Set(180.)
        material=UsdShade.Material.Define(stage,root+'/'+name+'_material')
        shader=UsdShade.Shader.Define(stage,root+'/'+name+'_material/shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
    _material_updated=True

def demo_rotation(arm,label,t,default):
    if arm!='right':return default
    start=ik.tcp_pose(np.array(READY),station.BASES['right'])[1]
    return ik.slerp_matrix(start,SIDE,ik.smoothstep(min(t/2,1.)))

def demo_target(arm,label,t,pos,rotation,finger):
    if arm=='right' and label=='crank':
        angle=-math.pi/2-(t-10.)*2*math.pi/4.
        pos=LIVE_CONTEXT['body_origins']['wheel']+np.array((.046,.035*math.cos(angle),.035*math.sin(angle)))+GRIP_OFFSET
    elif arm=='right' and label=='release knob':pos=LIVE_CONTEXT['body_origins']['knob']+GRIP_OFFSET
    elif arm=='left' and label in {'align pencil','feed pencil','hold feed','withdraw pencil'}:
        pos=pos+LIVE_CONTEXT['body_origins']['tool']-np.asarray(TOOL.pos)
        offset=LIVE_CONTEXT['body_origins']['pencil']-LIVE_CONTEXT['tcp']['left']
        pos=pos+np.clip(np.array((0,0,-PAD))-offset,-.015,.015)
        # Align the actual pencil tip, including its small tilt inside the pads.
        # Only the robot command changes; no object pose or joint is edited.
        axis=np.asarray(_rotate((1,0,0),tuple(LIVE_OBJECTS['pencil'].quat_xyzw)))
        axis*=1 if axis[0]>=0 else -1
        pos[1:]-=np.clip(.070*axis[1:],-.005,.005)
        pos[1:]+=np.clip(.15*LIVE_OBJECTS['tool'].lin_vel[1:],-.002,.002)
        if label=='hold feed':
            depression=LIVE_CONTEXT['body_origins']['cutter'][0]-(LIVE_CONTEXT['body_origins']['tool'][0]-.013)
            # Compensate arm compliance through a bounded Cartesian command.
            # The native passive spring still supplies the entire axial load.
            pos[0]+=np.clip(2*(.004-depression),-.002,.006)
    return pos,rotation,finger

def demo():
    # Thin pencils need pad overlap; the cube convention puts only pad tips
    # level with the pencil axis and misses its 7 mm cylindrical cross-section.
    pick=np.asarray(PENCIL.pos)+np.array((0,0,PAD-.008));hover=(.120,.10,TOP+.20)
    align=(.130,-.04,TOP+.12+PAD);feed=(.164,-.04,TOP+.12+PAD)
    knob=np.asarray(KNOB.pos)+GRIP_OFFSET;end=(KNOB.pos[0],WHEEL.pos[1]-.035+GRIP_OFFSET[1],WHEEL.pos[2])
    return {
      'left':[Waypoint(1.5,hover,OPEN,'to pencil'),Waypoint(1.5,tuple(pick),OPEN,'descend pencil'),
              Waypoint(.8,tuple(pick),CLOSED,'grip pencil'),Waypoint(1.2,hover,CLOSED,'lift pencil'),
              Waypoint(2.,align,CLOSED,'align pencil'),Waypoint(3.,feed,CLOSED,'feed pencil'),
              Waypoint(34.,feed,CLOSED,'hold feed'),Waypoint(2.,align,CLOSED,'withdraw pencil'),
              Waypoint(2.,hover,CLOSED,'return over cradle'),Waypoint(1.5,tuple(pick),CLOSED,'lower pencil'),
              Waypoint(.8,tuple(pick),OPEN,'release pencil'),Waypoint(1.5,hover,OPEN,'clear pencil')],
      'right':[Waypoint(2.,(knob[0],knob[1]-.045,knob[2]+.020),OPEN,'to knob'),
               Waypoint(1.5,tuple(knob),OPEN,'descend knob'),Waypoint(.8,tuple(knob),CLOSED,'grip knob'),
               Waypoint(.7,tuple(knob),CLOSED,'hold knob'),Waypoint(5.,tuple(knob),CLOSED,'wait feed'),
               Waypoint(32.8,end,CLOSED,'crank'),Waypoint(.8,end,OPEN,'release knob'),
               Waypoint(1.5,(.30,-.25,TOP+.16),OPEN,'withdraw'),Waypoint(12.,(.30,-.25,TOP+.16),OPEN,'done')],
    }
