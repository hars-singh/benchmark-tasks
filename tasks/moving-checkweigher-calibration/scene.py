"""Zero, span and verify a load cell while its pan travels on a rail carriage."""

from collections import deque
import json

import numpy as np
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_moving_checkweigher_calibration.geometry import Solid
from dynamic_task_moving_checkweigher_calibration.mechanisms import linear_stage
from dynamic_task_moving_checkweigher_calibration.motion import OscillatingPlatform


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
TRAVEL = .040
PERIOD = 7.
MINIMUM_SPEED = .012
STANDARD_KG = .300
CHECK_KG = .195
MAXIMUM_ERROR_KG = .006
PAN_TOP = TOP+.050
STAGE = Box('scale_carriage',pos=(.310,.071,TOP+.015),size=(.134,.114,.030),mass=2.4,
            color=(.13,.22,.32))
PAN = Box('weighing_pan',pos=(*STAGE.pos[:2],PAN_TOP-.004),size=(.098,.084,.008),mass=.115,
          color=(.51,.58,.64))
STANDARD = Box('reference_weight',pos=(.172,.188,TOP+.020),size=(.036,.032,.040),
               mass=STANDARD_KG,color=(.64,.53,.34))
CHECK = Box('verification_pack',pos=(.208,.276,TOP+.0175),size=(.047,.038,.035),
            mass=CHECK_KG,color=(.71,.53,.35))
PANEL = Box('scale_panel',pos=(.191,-.181,TOP+.017),size=(.102,.165,.034),mass=2.,
            color=(.13,.19,.24))
BUTTONS = {
    name:Box(f'{name}_button',pos=(PANEL.pos[0]-.016,PANEL.pos[1]+offset,TOP+.040),
             size=(.028,.030,.018),mass=.025,color=color)
    for name,offset,color in (
        ('zero',.045,(.19,.46,.80)), ('span',0.,(.86,.64,.16)), ('verify',-.045,(.25,.66,.35)),
    )
}
hardware, BODY_PARTS = linear_stage(STAGE,TRAVEL)
OBJECTS = [STAGE,PAN,STANDARD,CHECK,PANEL,*BUTTONS.values(),*hardware]
BODY_PARTS[STAGE.name] += tuple(
    Solid('box',(.018,.014,.012),(x,y,.021),color=(.21,.27,.32))
    for x in (-.030,.030) for y in (-.023,.023)
)
BODY_PARTS[PANEL.name] = (
    Solid('box',PANEL.size),
    Solid('box',(.024,.144,.005),(.029,0.,.0195),color=(.035,.059,.063)),
)
OBJECT_JOINTS = {
    PAN.name:{'kind':'fixed','parent':STAGE.name,'anchor':PAN.pos},
    **{button.name:{'kind':'prismatic','parent':PANEL.name,'anchor':button.pos,
                    'axis':'Z','lower':-.008,'upper':0.,
                    'drive':{'stiffness':650.,'damping':2.,'max_force':7.}}
       for button in BUTTONS.values()},
}
MOTIONS = {STAGE.name:OscillatingPlatform(STAGE.pos,(0.,TRAVEL,0.),PERIOD)}
SUPPORT_JOINTS = {STAGE.name:{'axis':'Y','lower':-.055,'upper':.055}}
MACHINE_VISUALS = {STAGE.name:{'kind':'linear_stage','amplitude':TRAVEL}}
CONTACT_PAIRS = {spec.name:(spec.name,PAN.name) for spec in (STANDARD,CHECK)}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,4) for obj in OBJECTS if obj.free}
SUCCESS_HOLD_S = .50
MAX_PARTIAL_SCORE = .50
LIVE_CONTEXT = {}
_zero = None
_scale = None
_reading = None
_mode = None
_samples = deque(maxlen=9)
_lamps = None
_display_state = None


def on_reset():
    global _zero, _scale, _reading, _mode, _lamps, _display_state
    _zero = _scale = _reading = _mode = _lamps = _display_state = None
    _samples.clear()


def on_pan(state, pan_origin, velocity):
    offset = state.pos-pan_origin
    return bool(abs(offset[0])<.024 and abs(offset[1])<.018
                and abs(state.low[2]-(pan_origin[2]+.004))<.003
                and np.linalg.norm(state.lin_vel-velocity)<.018
                and np.linalg.norm(state.ang_vel)<.5)


def observe(objects, context, dt):
    global _zero, _scale, _reading, _mode
    origins = context['body_origins']
    velocity = objects[STAGE.name].lin_vel
    pan_origin = origins[PAN.name]
    force = context['contact_forces']
    loads = {spec.name:float(force[spec.name][2]) for spec in (STANDARD,CHECK)}
    hands_clear = all(np.linalg.norm(tcp-pan_origin)>.066 for tcp in context['tcp'].values())
    mode = None
    if hands_clear and abs(velocity[1])>=MINIMUM_SPEED:
        occupied = [spec for spec in (STANDARD,CHECK) if on_pan(objects[spec.name],pan_origin,velocity)]
        if not occupied and all(abs(load)<.05 for load in loads.values()):
            mode = 'zero'
        elif len(occupied)==1 and loads[occupied[0].name]>1.0:
            mode = 'span' if occupied[0]==STANDARD else 'verify'
    if mode!=_mode or mode is None:
        _samples.clear()
    _mode = mode
    if mode is None:
        return
    # The electronics have an unknown gain and offset. Their input is the
    # simulator's signed native force on the load, not the authored body mass.
    _samples.append(37.+193.*sum(loads.values()))
    if len(_samples)<_samples.maxlen or np.ptp(_samples)>12.:
        return
    panel_shift = PANEL.pos[2]-origins[PANEL.name][2]
    pressed = [name for name,button in BUTTONS.items()
               if button.pos[2]-origins[button.name][2]-panel_shift>.004]
    if pressed!=[mode]:
        return
    counts = float(np.mean(_samples))
    if mode=='zero' and _zero is None:
        _zero = counts
    elif mode=='span' and _zero is not None and _scale is None and counts-_zero>120.:
        _scale = STANDARD_KG/(counts-_zero)
    elif mode=='verify' and _scale is not None and _reading is None:
        _reading = _scale*(counts-_zero)
    else:
        return
    print('NATIVE_CHECKWEIGHER '+json.dumps(dict(mode=mode,time_s=context['time_s'],
          samples=len(_samples),counts=counts,force_N=sum(loads.values()),zero=_zero,
          kg_per_count=_scale,verification_kg=_reading)),flush=True)


def success(objects):
    if _reading is None or abs(_reading-CHECK_KG)>MAXIMUM_ERROR_KG:
        return False
    standard,check = (objects[spec.name] for spec in (STANDARD,CHECK))
    return bool(np.linalg.norm(standard.pos-np.asarray(STANDARD.pos))<.018 and standard.resting()
                and on_pan(check,LIVE_CONTEXT['body_origins'][PAN.name],objects[STAGE.name].lin_vel)
                and all(np.linalg.norm(tcp-check.pos)>.065 for tcp in LIVE_CONTEXT['tcp'].values()))


def score(objects):
    if any(objects[spec.name].low[2]<TOP-.030 for spec in (STANDARD,CHECK)):
        return 0.
    if success(objects):
        return 1.
    return .50 if _scale is not None else .20 if _zero is not None else 0.


def update_materials():
    global _lamps, _display_state
    state = (_zero is not None,_scale is not None,
             _reading is not None and abs(_reading-CHECK_KG)<=MAXIMUM_ERROR_KG)
    if state==_display_state:
        return
    if _lamps is None:
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        stage = omni.usd.get_context().get_stage()
        (panel,) = (p for p in stage.Traverse() if p.GetName()==PANEL.name)
        root = str(panel.GetPath())+'/verification_indicators'
        _lamps = []
        for name,button in BUTTONS.items():
            material = UsdShade.Material.Define(stage,root+'/'+name+'_light')
            shader = UsdShade.Shader.Define(stage,root+'/'+name+'_light/shader')
            shader.CreateIdAttr('UsdPreviewSurface')
            emission = shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            geometry = UsdGeom.Cube.Define(stage,root+'/'+name+'_indicator')
            geometry.CreateSizeAttr(1.)
            geometry.AddTranslateOp().Set(Gf.Vec3d(.029,button.pos[1]-PANEL.pos[1],.0222))
            geometry.AddScaleOp().Set(Gf.Vec3f(.010,.019,.0004))
            UsdShade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(material)
            _lamps.append((emission,Gf.Vec3f(*button.color)))
    for active,(emission,color) in zip(state,_lamps,strict=True):
        emission.Set(color if active else .03*color)
    _display_state = state


def demo_target(arm, label, t, pos, rotation, finger):
    if arm!='left' or not LIVE_CONTEXT:
        return pos,rotation,finger
    if label.startswith('pan '):
        motion = LIVE_CONTEXT['body_origins'][PAN.name]-np.asarray(PAN.pos)
        lead = .10*LIVE_OBJECTS[STAGE.name].lin_vel[1]
        pos = pos+motion+np.array((0.,lead,0.))
    if label in {'rest lower reference_weight','rest release reference_weight'}:
        origin = LIVE_CONTEXT['body_origins'][STANDARD.name]
        tcp = LIVE_CONTEXT['tcp'][arm]
        if LIVE_CONTEXT['gripper'][arm]<.66 and np.linalg.norm(tcp-origin)<.055:
            pos = pos+np.r_[np.clip((tcp-origin)[:2],-.034,.034),0.]
    return pos,rotation,finger


def demo():
    hover_z = TOP+.159
    left = []
    for part,source,destination,first,second in (
        (STANDARD,STANDARD.pos,(*PAN.pos[:2],PAN_TOP+STANDARD.size[2]/2),'rest','pan'),
        (STANDARD,(*PAN.pos[:2],PAN_TOP+STANDARD.size[2]/2),STANDARD.pos,'pan','rest'),
        (CHECK,CHECK.pos,(*PAN.pos[:2],PAN_TOP+CHECK.size[2]/2),'rest','pan'),
    ):
        grab = (*source[:2],source[2]+PAD)
        release = (*destination[:2],destination[2]+PAD+.002)
        left.extend([
            Waypoint(1.25,(*grab[:2],hover_z),OPEN,f'{first} approach {part.name}'),
            Waypoint(.85,grab,OPEN,f'{first} align {part.name}'),
            Waypoint(.65,grab,CLOSED,f'{first} grasp {part.name}'),
            Waypoint(.8,(*grab[:2],hover_z),CLOSED,f'{first} lift {part.name}'),
            Waypoint(1.45,(*release[:2],hover_z),CLOSED,f'{second} carry {part.name}'),
            Waypoint(.85,release,CLOSED,f'{second} lower {part.name}'),
            Waypoint(.65,release,OPEN,f'{second} release {part.name}'),
            Waypoint(.80,(*release[:2],hover_z),OPEN,f'{second} clear {part.name}'),
        ])
        if part==STANDARD and second=='pan':
            left.append(Waypoint(2.5,(*release[:2],hover_z+.020),OPEN,'pan wait away from reference'))
    # Remain over the shelf while the other arm zeroes the empty pan.
    left[:0] = [Waypoint(2.0,(*STANDARD.pos[:2],hover_z),OPEN,'park above reference'),
                Waypoint(1.0,(*STANDARD.pos[:2],hover_z),OPEN,'wait for zero')]
    right = []
    for name,start,hold in (('zero',0.,1.35),('span',9.9,1.5),('verify',25.1,2.0)):
        button = BUTTONS[name]
        hover = (*button.pos[:2],TOP+.139)
        down = (*button.pos[:2],button.pos[2]+button.size[2]/2+PAD-.010)
        wait = start-sum(point.duration for point in right)
        if wait>0:
            right.append(Waypoint(wait,right[-1].pos,CLOSED,'keep hands clear'))
        right.extend([Waypoint(1.55,hover,CLOSED,f'approach {name}'),
                      Waypoint(.7,down,CLOSED,f'press {name}'),
                      Waypoint(hold,down,CLOSED,f'hold {name}'),
                      Waypoint(.8,hover,CLOSED,f'clear {name}')])
    return {'left':left,'right':right}
