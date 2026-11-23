"""Control a V60 practice pour from a physically grasped kettle."""

import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from dynamic_task_pour_over_bed_wetting.geometry import Solid

SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
BED = np.array((.271, .025, TOP+.114))
PAD = (.351, .253, TOP+.004)
KETTLE_CENTER = (.351, .253, TOP+.064)
SPOUT = np.array((-.105, 0., .072))
EXIT_AXIS = np.array((-.970, 0., -.243))
HANDLE = np.array((.066, 0., .018))
GRIP_OFFSET = np.array((.004,0.,.034))
INLET_R = .036
LOAD_G = 220.
SERVER_BASE = TOP+.005
HANDLE_BLACK = (.077, .083, .090)
GLASS = (.62, .72, .74)
PAPER = (.88, .86, .78)
GROUNDS = (.30, .161, .083)
WATER = (.54, .65, .74)
WET = (.115, .053, .024)

KETTLE = Box('hario_air_drip_kettle', pos=KETTLE_CENTER, size=(.177,.078,.128),
             mass=.310, color=(.33,.42,.44))
MAT = Box('silicone_kettle_rest', pos=PAD, size=(.157,.170,.008), color=(.23,.35,.33))
SCALE = Box('gram_scale', pos=(float(BED[0]),float(BED[1]),TOP+.0025),
            size=(.126,.137,.005), color=HANDLE_BLACK)
SERVER = Box('clear_coffee_server', pos=(float(BED[0]),float(BED[1]),TOP+.0375),
             size=(.098,.091,.075), mass=.156, color=GLASS)
FILTER = Box('v60_01_dripper_and_paper_filter',pos=(float(BED[0]),float(BED[1]),TOP+.114),
             size=(.116,.116,.090), mass=.11, color=PAPER)
OBJECTS = [KETTLE,MAT,SCALE,SERVER,FILTER]


def ring(low_radius, high_radius, thickness, length, z, color, opacity=None):
    count = 20
    return tuple(Solid('tapered_panel',(low_radius,high_radius,thickness,length,math.tau/count),
                       (0.,0.,z),roll=math.tau*i/count,pitch=-math.pi/2,color=color,opacity=opacity)
                 for i in range(count))


def segment(start, end, radius, color):
    start,end = np.asarray(start),np.asarray(end)
    dx,dy,dz = end-start
    return Solid('capsule_x',(radius,max(.002,float(np.linalg.norm(end-start))-2*radius)),
                 tuple(map(float,(start+end)/2)),yaw=math.atan2(dy,dx),
                 pitch=-math.atan2(dz,math.hypot(dx,dy)),color=color)


spout_profile = ((-.036, .005, .015), (-.055, .014, .020),
                 (-.070, .031, .015), (-.085, .054, .010), (-.102, .069, .005))
handle_path = ((.036,0.,.028),(.058,0.,.035),tuple(HANDLE),
               (.062,0.,-.012),(.050,0.,-.031),(.036,0.,-.034))
BODY_PARTS = {
    KETTLE.name: (
        Solid('cylinder',(.036,.005),(0.,0.,-.054),color=(.27,.36,.37),opacity=.50),
        *ring(.034,.037,.002,.101,-.0018,(.34,.43,.45),.38),
        *(Solid('box', (.022, .0020, .075 - z), (x, side * half_width, (z + .070) / 2.),
                color=(.37,.47,.48), opacity=.26)
          for x,z,half_width in spout_profile for side in (-1,1)),
        *(Solid('box', (.022, 2. * half_width, .002), (x, 0., z),
                pitch=-.64, color=(.37,.47,.48), opacity=.26)
          for x,z,half_width in spout_profile),
        *(segment(a,b,.0075,(.24,.31,.33)) for a,b in zip(handle_path,handle_path[1:])),
        Solid('box',(.014,.016,.028),(.062,0.,.007),color=(.24,.31,.33)),
    ),
    SERVER.name: (
        Solid('cylinder',(.045,.004),(0.,0.,-.0305),color=GLASS,opacity=.40),
        *ring(.043,.036,.003,.064,.0015,GLASS,.32),
        segment((.039,0.,.017),(.061,0.,.013),.005,GLASS),
        segment((.061,0.,.013),(.058,0.,-.014),.005,GLASS),
        segment((.058,0.,-.014),(.044,0.,-.019),.005,GLASS),
    ),
    FILTER.name: (
        *ring(.012,.054,.003,.077,0.,PAPER),
        Solid('cylinder',(.056,.0038),(0.,0.,-.037),color=(.76,.78,.76)),
        Solid('cylinder',(.030,.002),(0.,0.,-.001),color=GROUNDS),
    ),
}
VISUAL_PARTS = {
    SCALE.name: (Solid('box',(.024,.034,.0003),(-.048,0.,.0027),color=(.28,.64,.51)),),
}
OBJECT_JOINTS = {
    FILTER.name: {'kind':'fixed','parent':SERVER.name,'anchor':FILTER.pos,
                  'collision_enabled':False},
}
SURFACE_FRICTION = {KETTLE.name:(.92,.79),MAT.name:(.76,.61),SERVER.name:(.61,.43)}
CONTACT_OFFSETS_M = {KETTLE.name:.00055,FILTER.name:.00033,SERVER.name:.00040}
OBJECT_MATERIALS = {KETTLE.name:'Hario VKA-35-TB translucent PCT resin kettle and loop handle',
                    MAT.name:'silicone kettle rest', SCALE.name:'gram scale',
                    SERVER.name:'clear glass coffee server', FILTER.name:'size 01 dripper and paper filter'}

cam = np.array((-.229,-.163,.301))
forward = np.array((.028,.006,.064))-cam
forward /= np.linalg.norm(forward)
right = np.cross(forward,(0.,0.,1.))
right /= np.linalg.norm(right)
rotation = np.column_stack((right,np.cross(right,forward),-forward))
TASK_RECORD_CAMERA = (SCALE.name, Camera(tuple(cam),tuple(matrix_to_quat_xyzw(rotation)),
                                        'opengl',Lens.from_focal(25.,640,360)))

SAMPLES = np.array([(0.,0.)]+[(r*math.cos(a),r*math.sin(a)) for r,count in ((.012,6),(.024,10))
                             for a in (i*math.tau/count for i in range(count))])
LIVE_CONTEXT = {}
SUCCESS_HOLD_S = 1.
MAX_PARTIAL_SCORE = .5
_wet = np.zeros(len(SAMPLES))
_dose = _served = _pond = _spill = 0.
_pulse_end = _bloom_at = _finish_at = None
_bloom_ok = False
_graphics = None
_impact = _nozzle = None
_stream_rate = 0.
_grasp_offset = None
_grasp_rotation = None
_aim_trim = np.zeros(2)


def on_reset():
    global _dose,_served,_pond,_spill,_pulse_end,_bloom_at,_finish_at,_bloom_ok,_impact,_nozzle,_stream_rate
    global _grasp_offset,_grasp_rotation
    _wet.fill(0.)
    _dose = _served = _pond = _spill = _stream_rate = 0.
    _pulse_end = _bloom_at = _finish_at = _impact = _nozzle = None
    _bloom_ok = False
    _grasp_offset = _grasp_rotation = None
    _aim_trim.fill(0.)


def landing(nozzle, direction, grams_per_second, level):
    speed = grams_per_second*.001/(1000.*math.pi*.0020**2)
    velocity = speed*direction
    drop = nozzle[2]-level
    if drop <= 0.:
        return nozzle.copy()
    flight = (velocity[2]+math.sqrt(velocity[2]**2+2.*9.81*drop))/9.81
    return np.asarray((nozzle[0]+flight*velocity[0],nozzle[1]+flight*velocity[1],level))


def stream(body, rot, available, level):
    nozzle = body+rot@SPOUT
    direction = rot@EXIT_AXIS
    tilt = math.atan2(float(rot[2,0]),float(rot[2,2]))
    empty_correction = .08*(1.-available/LOAD_G)
    rate = max(0.,min(12.,30.*(tilt-.43-empty_correction))) if available>0 else 0.
    return nozzle,landing(nozzle,direction,rate,level),rate


def observe(objects, context, dt):
    global _dose,_served,_pond,_spill,_pulse_end,_bloom_ok,_impact,_nozzle,_stream_rate
    clock = context['time_s']
    centre = np.asarray(context['body_origins'][FILTER.name])
    body = np.asarray(context['body_origins'][KETTLE.name])
    rot = quat_xyzw_to_matrix(objects[KETTLE.name].quat_xyzw)
    available = max(0.,LOAD_G-_dose-_spill)
    _nozzle,_impact,_stream_rate = stream(body,rot,available,centre[2])
    mass = min(available,_stream_rate*dt)
    local = _impact[:2]-centre[:2]
    if mass>0. and _nozzle[2]>centre[2]+.018 and np.linalg.norm(local)<=INLET_R:
        _dose += mass
        _pond += mass
        _wet[:] += mass*np.maximum(0.,1.-np.linalg.norm(SAMPLES-local,axis=1)/.015)
        if _dose <= 28.:
            _pulse_end = clock
    else:
        _spill += mass
    coverage = np.mean(_wet>=.40)
    if _pulse_end is not None and clock-_pulse_end>=24. and 20.<=_dose<=28. and coverage>=.82:
        _bloom_ok = True
    drain = min(_pond, dt*max(0.,min(13.,(_pond-16.)*.74)))
    _pond -= drain
    _served += drain
    if _pond>56.:
        _spill += _pond-56.
        _pond = 56.


def success(objects):
    centre = LIVE_CONTEXT['body_origins'][KETTLE.name]
    vessel = LIVE_CONTEXT['body_origins'][SERVER.name]
    upright = quat_xyzw_to_matrix(objects[KETTLE.name].quat_xyzw)[2,2]>.98
    parked = upright and math.dist(centre,KETTLE_CENTER)<.025 and np.linalg.norm(objects[KETTLE.name].lin_vel)<.015
    clear = all(math.dist(centre,tcp)>.095 for tcp in LIVE_CONTEXT['tcp'].values())
    return bool(_bloom_ok and 117.<=_dose<=124. and 98.<=_served<=110.
                and _spill<=2. and np.mean(_wet>=.40)>=.94 and _pond<=19.
                and math.dist(vessel,SERVER.pos)<.006 and parked and clear)


def score(objects):
    if _spill>2. or math.dist(LIVE_CONTEXT['body_origins'][SERVER.name],SERVER.pos)>.006:
        return 0.
    return 1. if success(objects) else .5 if _bloom_ok else 0.


def update_visuals():
    global _graphics
    from pxr import Gf, UsdGeom, Vt
    if _graphics is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        server_path = next(prim.GetPath() for prim in stage.Traverse() if prim.GetName()==SERVER.name)
        filter_path = next(prim.GetPath() for prim in stage.Traverse() if prim.GetName()==FILTER.name)
        cup = UsdGeom.Cylinder.Define(stage,str(server_path)+'/liquid')
        cup.CreateRadiusAttr(.040)
        cup.CreateHeightAttr(1.)
        cup.CreateDisplayColorAttr([Gf.Vec3f(.34,.183,.095)])
        cup.CreateDisplayOpacityAttr([.72])
        curve = UsdGeom.BasisCurves.Define(stage,str(server_path)+'/water_stream')
        curve.CreateTypeAttr('linear')
        curve.CreateCurveVertexCountsAttr([2])
        curve.CreateWidthsAttr([.0031])
        curve.SetWidthsInterpolation('constant')
        curve.CreateDisplayColorAttr([Gf.Vec3f(*WATER)])
        spots = []
        for i,(x,y) in enumerate(SAMPLES):
            dot = UsdGeom.Cylinder.Define(stage,str(filter_path)+f'/wet_patch_{i}')
            dot.CreateRadiusAttr(.0062)
            dot.CreateHeightAttr(.00008)
            dot.CreateDisplayColorAttr([Gf.Vec3f(*WET)])
            dot.AddTranslateOp().Set(Gf.Vec3d(float(x),float(y),.0001))
            spots.append(dot)
        _graphics = (cup.AddTranslateOp(),cup.AddScaleOp(),curve,spots)
    translate,scale,curve,spots = _graphics
    height = max(.00001,_served*.000001/(math.pi*.040**2))
    translate.Set(Gf.Vec3d(0.,0.,float(SERVER_BASE-SERVER.pos[2]+height/2.)))
    scale.Set(Gf.Vec3f(1.,1.,float(height)))
    for dot,wetness in zip(spots,_wet):
        dot.MakeVisible() if wetness>=.4 else dot.MakeInvisible()
    if _impact is not None and _stream_rate>.1:
        offset = np.asarray(LIVE_CONTEXT['body_origins'][SERVER.name])
        curve.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float,v-offset)) for v in (_nozzle,_impact)]))
        curve.MakeVisible()
    else:
        curve.MakeInvisible()


PICK = np.asarray(KETTLE_CENTER)+GRIP_OFFSET
HOVER = PICK+(0.,0.,.075)
HIGH = PICK+(0.,0.,.148)
POUR_HOVER = np.array((.388,.027,TOP+.283))
POUR_START = 6.1
PINCH = -.034
SET_DOWN = PICK + np.array((.018,-.004,.004))
TILT = .744
FINISH_TILT = .945


def demo():
    return {'left':[
        Waypoint(1.25,tuple(HOVER),FINGER_OPEN,'approach the practice kettle'),
        Waypoint(.92,tuple(PICK),FINGER_OPEN,'place both fingertips across the outer rim'),
        Waypoint(.86,tuple(PICK),PINCH,'grip the kettle body'),
        Waypoint(1.32,tuple(HIGH),PINCH,'lift clear of the kettle rest'),
        Waypoint(1.7,tuple(POUR_HOVER),PINCH,'move over the paper filter'),
        Waypoint(85.,tuple(POUR_HOVER),PINCH,'wet the bed and drain the coffee'),
    ]}


def ramp(x):
    u = min(1.,max(0.,x))
    return u*u*(3.-2.*u)


def pour_pose(tilt, dose, radius_limit):
    r = axis_angle_matrix(np.array((0.,1.,0.)),-tilt)
    ph = min(1.,dose/7.)*dose*.54
    radius = radius_limit*ramp(dose/8.)
    hit = BED+np.array((radius*math.cos(ph),radius*math.sin(ph),0.))
    nozzle = BED+np.array((0.,0.,.097))
    _,offset,_ = stream(nozzle-r@SPOUT,r,max(0.,LOAD_G-dose),BED[2])
    nozzle[:2] += hit[:2]-offset[:2]
    grip = nozzle-r@SPOUT+r@(_grasp_offset if _grasp_offset is not None else GRIP_OFFSET)
    return grip,r,hit


def demo_target(arm,phase,t,position,rotation,finger):
    global _bloom_at,_finish_at,_grasp_offset,_grasp_rotation
    base = axis_angle_matrix(np.array((0.,1.,0.)),-.34)@grip_rotation()
    if arm=='left' and 3.78<t<5.75 and 'body_origins' in LIVE_CONTEXT:
        native_pose=np.asarray(LIVE_CONTEXT['body_origins'][KETTLE.name])
        native_rotation=quat_xyzw_to_matrix(LIVE_OBJECTS[KETTLE.name].quat_xyzw)
        tool_position=np.asarray(LIVE_CONTEXT['tcp']['left'])
        if native_pose[2]>KETTLE_CENTER[2]+.047 and math.dist(native_pose,tool_position)<.12:
            _grasp_offset=native_rotation.T@(tool_position-native_pose)
            _grasp_rotation=native_rotation.T@LIVE_CONTEXT['tcp_rotation']['left']
    if arm!='left' or t<POUR_START:
        return position,base,finger
    neutral=_grasp_rotation if _grasp_rotation is not None else base
    if _bloom_at is None and _dose>=22.:
        _bloom_at=t
    if _finish_at is None and _dose>=118.:
        _finish_at=t
    if _finish_at is not None and t>_finish_at+1.25:
        elapsed=t-_finish_at-1.25
        lift=POUR_HOVER+(0.,0.,.017)
        if elapsed<1.5:
            return lift+ramp(elapsed/1.5)*(HIGH-lift),neutral,PINCH
        if elapsed<3.15:
            return HIGH+ramp((elapsed-1.5)/1.65)*(SET_DOWN-HIGH),neutral,PINCH
        if elapsed<3.47:
            return SET_DOWN,neutral,PINCH
        if elapsed<4.03:
            return SET_DOWN,neutral,FINGER_OPEN
        return SET_DOWN+ramp((elapsed-4.03)/1.15)*np.array((-.073,.019,.140)),neutral,FINGER_OPEN
    if _finish_at is not None:
        fraction=1.-ramp((t-_finish_at)/.62)
    elif _bloom_at is None:
        fraction=ramp((t-POUR_START)/1.15)
    elif t-_bloom_at<27.:
        fraction=1.-ramp((t-_bloom_at)/.64)
    else:
        fraction=ramp((t-_bloom_at-27.)/1.0)
    if fraction<.01:
        return POUR_HOVER,neutral,PINCH
    second = _bloom_at is not None and t - _bloom_at >= 27.
    tilt = FINISH_TILT if second else TILT
    grip,grip_r,hit = pour_pose(tilt*fraction,_dose,.010 if second else .022)
    if _finish_at is None and fraction < .94:
        grip[2] += .027 * (1. - ramp((fraction - .48) / .46))
    if second and _finish_at is None and _impact is not None and _stream_rate > .8:
        error = np.clip(hit[:2] - _impact[:2],-.043,.043)
        _aim_trim[:] = np.clip(_aim_trim + .071 * error,-.042,.042)
        grip[:2] += _aim_trim + .37 * error
    elif second:
        grip[:2] += _aim_trim
    start = ramp((t-POUR_START)/.92)
    rotation=grip_r@(_grasp_rotation if _grasp_rotation is not None else base)
    return POUR_HOVER+start*(grip-POUR_HOVER),rotation,PINCH
