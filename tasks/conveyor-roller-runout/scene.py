"""Measure a conveyor roller while turning its inspection fixture by hand."""

import json
import math

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD, READY
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_conveyor_roller_runout.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
AXIS = (.529, -.050, TOP+.121)
ECCENTRICITY_M = .00035
TOLERANCE_MM = .40
CRANK_RADIUS = .035
CRANK_X = AXIS[0]-.155
TURN_START_S = 5.
TURN_DURATION_S = 16.
SECONDS_PER_TURN = 10.
TIP_Z = AXIS[2]+.025+.00035+.0015
AXIAL_GRIP = np.array([[0.,0.,1.],[0.,-1.,0.],[1.,0.,0.]])
CRANK_OFFSET = np.array((-.009,0.,0.))
CRANK_OPEN = .30*OPEN

BASE = Box('check_stand', pos=(AXIS[0]-.009,AXIS[1],TOP+.010), size=(.23,.145,.020),
           mass=5., color=(.15,.25,.34))
ROLLER = Box('roller', pos=AXIS, size=(.15,.050,.050), mass=.365, color=(.58,.63,.67))
KNOB = Box('crank_grip', pos=(CRANK_X,AXIS[1],AXIS[2]-CRANK_RADIUS),
           size=(.030,.016,.016), mass=.009, color=(.11,.18,.23))
PLUNGER = Box('indicator_plunger', pos=(*AXIS[:2],TIP_Z), size=(.008,.008,.05),
              mass=.025, color=(.73,.75,.78))
PANEL = Box('result_panel', pos=(.16,.205,TOP+.016), size=(.180,.135,.032),
            mass=3.,color=(.16,.19,.22))
PASS = Box('pass_button', pos=(.16,.239,TOP+.038), size=(.030,.030,.018),
           mass=.032,color=(.11,.62,.34))
FAIL = Box('fail_button', pos=(.16,.171,TOP+.038), size=PASS.size,
           mass=PASS.mass,color=(.85,.30,.13))
OBJECTS = [BASE,ROLLER,KNOB,PLUNGER,PANEL,PASS,FAIL]

BODY_PARTS = {
    'check_stand': (
        Solid('box',(.23,.145,.020)),
        *(Solid('box',(.012,.054,.110),(x,0.,.065)) for x in (-.080,.092)),
        Solid('cylinder',(.004,.181),(.009,0.,.111),axis='X',color=(.39,.42,.46)),
        Solid('box',(.018,.015,.199),(.009,.061,.109)),
        Solid('box',(.018,.071,.015),(.009,.030,.193)),
        Solid('box',(.022,.024,.029),(.009,.003,.189),color=(.11,.15,.18)),
    ),
    'roller': (
        Solid('cylinder',(.025,.150),(0.,0.,ECCENTRICITY_M),axis='X'),
        Solid('cylinder',(.005,.081),(-.1075,0.,0.),axis='X'),
        Solid('box',(.006,.011,.040),(-.146,0.,-.016),color=(.19,.30,.45)),
    ),
    'crank_grip': (Solid('cylinder',(.008,.030),axis='X'),),
    'indicator_plunger': (Solid('sphere',(.0015,)),Solid('cylinder',(.0035,.048),(0.,0.,.025))),
    'result_panel': (
        Solid('box',(.180,.135,.032)),
        Solid('box',(.051,.075,.021),(.056,0.,.025),color=(.12,.16,.19)),
        Solid('box',(.040,.057,.001),(.056,0.,.036),color=(.02,.07,.06)),
    ),
}
OBJECT_JOINTS = {
    'roller': {'kind':'revolute','parent':'check_stand','anchor':AXIS,'axis':'X',
               'drive':{'stiffness':0.,'damping':.0018,'max_force':.060}},
    'crank_grip': {'kind':'revolute','parent':'roller','anchor':KNOB.pos,'axis':'X',
                   'drive':{'damping':.0001,'max_force':.005}},
    'indicator_plunger': {'kind':'prismatic','parent':'check_stand','anchor':PLUNGER.pos,
        'axis':'Z','lower':-.008,'upper':.003,
        'drive':{'stiffness':250.,'damping':1.,'max_force':1.5,'target':-.003}},
    **{button.name: {'kind':'prismatic','parent':'result_panel','anchor':button.pos,
        'axis':'Z','lower':-.008,'upper':0.,
        'drive':{'stiffness':1000.,'damping':3.,'max_force':9.,'target':0.}}
       for button in (PASS,FAIL)},
}
CONTACT_PAIRS = {'tip_roller':('indicator_plunger','roller')}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,0) for obj in OBJECTS if obj.free}
TRACK_OBJECTS = {'right':'crank_grip'}
TRACK_PHASES = {'right':{'reach crank','align crank','grip crank'}}
LIVE_CONTEXT = {}
SUCCESS_HOLD_S = .75
MAX_PARTIAL_SCORE = .5

_reading = None
_window = []
_decision = None
_last_angle = None
_angle = 0.
_display_shapes = None
_displayed_text = None


def on_reset():
    global _reading,_decision,_last_angle,_angle,_displayed_text,_display_shapes
    _reading = _decision = _last_angle = _displayed_text = _display_shapes = None
    _angle = 0.
    _window.clear()


def observe(objects, context, dt):
    global _reading,_decision,_last_angle,_angle
    origins=context['body_origins']
    q=objects['roller'].quat_xyzw
    a=2*math.atan2(q[0],q[3])
    if _last_angle is not None:
        delta=a-_last_angle
        _angle+=math.atan2(math.sin(delta),math.cos(delta))
    _last_angle=a
    if _reading is None:
        reaction=float(np.linalg.norm(context['contact_forces']['tip_roller']))
        if not .15<=reaction<=2. or abs(objects['roller'].ang_vel[0])>2.15:
            _window.clear()
        else:
            stroke=origins['indicator_plunger'][2]-origins['check_stand'][2]
            _window.append((_angle,stroke))
        if _window:
            values=np.asarray(_window)
            if np.ptp(values[:,0])>=math.tau:
                _reading=1000*float(np.ptp(values[:,1]))
    if _reading is None:
        return
    panel_drop=PANEL.pos[2]-origins['result_panel'][2]
    pressed=[button.name for button in (PASS,FAIL)
             if button.pos[2]-origins[button.name][2]-panel_drop>=.004]
    if pressed:
        _decision=pressed[0] if len(pressed)==1 else None


def success(objects):
    if _reading is None or _decision is None:
        return False
    desired=PASS.name if _reading<=TOLERANCE_MM else FAIL.name
    threshold_matches_fixture=(_reading<=TOLERANCE_MM)==(2000*ECCENTRICITY_M<=TOLERANCE_MM)
    return bool(threshold_matches_fixture and _decision==desired and abs(objects['roller'].ang_vel[0])<.12
                and all(np.linalg.norm(LIVE_CONTEXT['body_origins']['crank_grip']-tcp)>.065
                        for tcp in LIVE_CONTEXT['tcp'].values()))


def score(objects):
    return 1. if success(objects) else .5 if _reading is not None else 0.


# Seven-segment numerals on the test stand's actual display. Only measured
# displacement enters the readout. Its geometry is visual and has no collision.
_DIGITS = ('abcdef','bc','abdeg','abcdg','bcfg','acdfg','acdefg','abc','abcdefg','abcdfg')
_SEGMENTS = {'a':(0.,.005,.006,.0011),'b':(.003,.0025,.0011,.0042),
             'c':(.003,-.0025,.0011,.0042),'d':(0.,-.005,.006,.0011),
             'e':(-.003,-.0025,.0011,.0042),'f':(-.003,.0025,.0011,.0042),
             'g':(0.,0.,.006,.0011)}


def update_materials():
    global _display_shapes,_displayed_text
    if _reading is None or _reading>=10.:
        return
    text=f'{_reading:.2f}'.replace('.','')
    if text==_displayed_text:
        return
    import omni.usd
    from pxr import Gf,Sdf,UsdGeom,UsdShade

    if _display_shapes is None:
        print('NATIVE_ROLLER_MEASUREMENT '+json.dumps({'runout_mm':_reading,
              'time_s':LIVE_CONTEXT['time_s'],'samples':len(_window)}),flush=True)
        usd=omni.usd.get_context().get_stage()
        bases=[p for p in usd.Traverse() if p.GetName()=='result_panel']
        if len(bases)!=1:
            raise RuntimeError('roller fixture display body missing')
        root=str(bases[0].GetPath())+'/geometry/'
        material=UsdShade.Material.Define(usd,root+'readout_light')
        shader=UsdShade.Shader.Define(usd,root+'readout_light/shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.13,.90,.32))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        def draw(name,x,y,w,h):
            shape=UsdGeom.Cube.Define(usd,root+name)
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(.056+y,-x,.037))
            shape.AddScaleOp().Set(Gf.Vec3f(h,w,.0003))
            UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material)
            return UsdGeom.Imageable(shape.GetPrim())
        _display_shapes=[{name:draw(f'readout_{i}_{name}',x+dx,dy,w,h)
                          for name,(dx,dy,w,h) in _SEGMENTS.items()}
                         for i,x in enumerate((-.012,0.,.012))]
        draw('readout_decimal',-.006,-.005,.0012,.0012)
    for digit,shapes in zip(text,_display_shapes):
        for name,shape in shapes.items():
            if name in _DIGITS[int(digit)]:shape.MakeVisible()
            else:shape.MakeInvisible()
    _displayed_text=text


def demo_rotation(arm,label,t,default):
    if arm!='right':
        return default
    retreat_start = TURN_START_S+TURN_DURATION_S+.7
    if t>retreat_start:
        blend=ik.smoothstep(min((t-retreat_start)/2.,1.))
        return ik.slerp_matrix(AXIAL_GRIP,default,blend)
    start=ik.tcp_pose(np.array(READY),station.BASES['right'])[1]
    return ik.slerp_matrix(start,AXIAL_GRIP,ik.smoothstep(min(t/2.,1.)))


def _crank_point(angle):
    return np.array((CRANK_X,AXIS[1]+CRANK_RADIUS*math.cos(angle),
                     AXIS[2]+CRANK_RADIUS*math.sin(angle)))+CRANK_OFFSET


def demo_target(arm,label,t,pos,rotation,finger):
    if arm=='right' and label=='measure turn':
        scheduled = -(t-TURN_START_S)*math.tau/SECONDS_PER_TURN
        bounded = max(scheduled,_angle-.45)
        pos=_crank_point(-math.pi/2+bounded)
        pos+=LIVE_CONTEXT['body_origins']['roller']-np.array(AXIS)
    elif arm=='right' and label=='release crank':
        pos=LIVE_CONTEXT['body_origins']['crank_grip']+CRANK_OFFSET
    elif arm=='left' and label in {'report approach','press result','hold result','leave result'} and _reading is not None:
        if _reading<=TOLERANCE_MM:
            pos=pos.copy();pos[1]+=PASS.pos[1]-FAIL.pos[1]
    return pos,rotation,finger


def demo():
    grip=_crank_point(-math.pi/2)
    end=_crank_point(-math.pi/2-TURN_DURATION_S*math.tau/SECONDS_PER_TURN)
    clear=(.300,-.145,TOP+.205)
    inspect=(PANEL.pos[0]+.056,PANEL.pos[1],TOP+.195)
    hover=(FAIL.pos[0],FAIL.pos[1],TOP+.125)
    push=(FAIL.pos[0],FAIL.pos[1],FAIL.pos[2]+PAD-.0015)
    return {
        'right':[Waypoint(2.,(grip[0]-.047,grip[1],grip[2]),CRANK_OPEN,'reach crank'),
            Waypoint(1.5,tuple(grip),CRANK_OPEN,'align crank'),Waypoint(.8,tuple(grip),CLOSED,'grip crank'),
            Waypoint(.7,tuple(grip),CLOSED,'hold crank'),Waypoint(TURN_DURATION_S,tuple(end),CLOSED,'measure turn'),
            Waypoint(.7,tuple(end),CRANK_OPEN,'release crank'),Waypoint(2.,clear,OPEN,'clear crank'),
            Waypoint(13.,clear,OPEN,'wait record')],
        'left':[Waypoint(2.,(.13,.32,TOP+.18),OPEN,'park'),Waypoint(21.8,(.13,.32,TOP+.18),OPEN,'wait reading'),
            Waypoint(2.,inspect,OPEN,'view readout'),Waypoint(1.5,inspect,OPEN,'read indicator'),
            Waypoint(2.,hover,CLOSED,'report approach'),Waypoint(2.,push,CLOSED,'press result'),
            Waypoint(.5,push,CLOSED,'hold result'),Waypoint(1.5,hover,OPEN,'leave result')],
    }
