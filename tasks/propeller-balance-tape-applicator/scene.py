"""Transfer a balancing strip from a hand tool to an unpowered propeller."""

import json
import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED as CLOSED, FINGER_OPEN as OPEN, PAD_TIP_PAST_TCP as PAD
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import quat_xyzw_to_matrix

from dynamic_task_propeller_balance_tape_applicator.geometry import Solid


SCENE = 'station'
RENDER_ANTIALIASING = 'DLAA'
TOP = TABLE_TOP[SCENE]
AXIS = (.337,.063,TOP+.150)
TIP_RADIUS = .101
TAPE_MASS = .00005
BLADE_ROLL = -.23
TAPE_SEAT = np.array((-TIP_RADIUS,0.,.00162))
TAPE_UNDER_TOOL = np.array((0.,0.,-.0215))
PRESS_FORCE_N = .90
MAXIMUM_PRESS_FORCE_N = 12.
MAXIMUM_BACKING_FORCE_N = 15.
PRESS_DURATION_S = .20
GRIP_HEIGHT = .012
PROOF_DURATION_S = 10.
BASE = Box('balancer',pos=(*AXIS[:2],TOP+.0075),size=(.16,.183,.015),
           mass=3.,color=(.19,.26,.33))
ROTOR = Box('propeller',pos=AXIS,size=(.254,.032,.028),mass=.02013,
            color=(.33,.39,.44))
EXCESS = Box('excess_mass',pos=(AXIS[0]+TIP_RADIUS,*AXIS[1:]),
             size=(.015,.017,.00014),mass=TAPE_MASS,color=ROTOR.color)
BACKING = Box('blade_backing',pos=(AXIS[0]-TIP_RADIUS,AXIS[1],AXIS[2]-.00355),
              size=(.016,.019,.004),mass=.050,color=(.42,.46,.50))
REST = Box('tamp_stand',pos=(.181,.225,TOP+.009),size=(.078,.061,.018),
           color=(.24,.32,.37))
TAMP = Box('tamp_tool',pos=(*REST.pos[:2],TOP+.061),size=(.052,.030,.046),
           mass=.046,color=(.67,.50,.24))
TAPE = Box('balancing_tape',pos=tuple(map(float,np.asarray(TAMP.pos)+TAPE_UNDER_TOOL)),
           size=(.015,.017,.00014),mass=TAPE_MASS,color=(.83,.89,.91))
OBJECTS = [BASE,ROTOR,EXCESS,BACKING,REST,TAPE,TAMP]
BODY_PARTS = {
    BASE.name:(
        Solid('box',BASE.size),
        *(Solid('box',(.026,.014,.123),(0.,y,.069)) for y in (-.058,.058)),
        *(Solid('cylinder',(.015,.009),(x,y,.128),axis='Y',color=(.62,.65,.69))
          for x in (-.009,.009) for y in (-.058,.058)),
    ),
    ROTOR.name:(
        Solid('cylinder',(.01016,.009652),axis='Y'),
        Solid('cylinder',(.0018,.1397),axis='Y',color=(.59,.64,.70)),
        *(Solid('box',(length,chord,.0025),(side*x,0.,0.),roll=side*twist)
          for side in (-1.,1.) for x,length,chord,twist in (
              (.023,.035,.022,.42),(.056,.035,.023,.31),(.091,.036,.020,.23),(.117,.020,.013,.18))),
    ),
    BACKING.name:(Solid('box',BACKING.size,roll=BLADE_ROLL),),
    REST.name:(
        Solid('box',REST.size),
        *(Solid('box',(.013,.042,.033),(side*.023,0.,.0255),color=(.12,.21,.28))
          for side in (-1,1)),
    ),
    TAMP.name:(
        Solid('box',(.032,.023,.023),(0.,0.,.0115)),
        Solid('box',(.052,.030,.006),(0.,0.,-.007),color=(.35,.40,.45)),
        Solid('box',(.010,.010,.009),(0.,0.,-.0135)),
        # Keep the footprint inside the foil and give the pad a small recess.
        # The outer blade must take the load through the foil surface.
        Solid('box',(.008,.006,.0024),(0.,-.00027,-.01955),roll=BLADE_ROLL,color=(.59,.61,.62)),
    ),
    TAPE.name:(Solid('box',TAPE.size,roll=BLADE_ROLL),),
}
OBJECT_JOINTS = {
    ROTOR.name:{'kind':'revolute','parent':BASE.name,'anchor':AXIS,'axis':'Y',
                'drive':{'stiffness':1.20,'damping':.0003,'max_force':1.5}},
    EXCESS.name:{'kind':'fixed','parent':ROTOR.name,'anchor':EXCESS.pos},
    BACKING.name:{'kind':'fixed','parent':BASE.name,'anchor':BACKING.pos},
    TAPE.name:{'kind':'fixed','parent':ROTOR.name,'anchor':TAPE.pos,'enabled':False,
               'collision_enabled':True,'break_force_N':12.,'break_torque_Nm':.027},
    # The attachment is authored before physics starts. The load passes from
    # the hand through the tool and attached strip to native blade contact.
    TAMP.name:{'kind':'fixed','parent':TAPE.name,'anchor':TAPE.pos},
}
CONTACT_PAIRS = {
    'tape_on_blade':(TAPE.name,ROTOR.name),
    'tool_on_blade':(TAMP.name,ROTOR.name),
    'blade_on_backing':(ROTOR.name,BACKING.name),
    'tool_on_stand':(TAMP.name,REST.name),
}
RIGID_SOLVER_ITERATIONS = {obj.name:(96,4) for obj in OBJECTS if obj.free}
CONTACT_OFFSETS_M = {obj.name:.000065 for obj in (ROTOR,TAPE,TAMP,BACKING)}
SUCCESS_HOLD_S = .50
MAX_PARTIAL_SCORE = .60
LIVE_CONTEXT = {}
_drive = None
_joints = None
_listener = None
_loaded_s = 0.
_first_release = None
_baseline_drift = 0.
_return_from_angle = None
_lifted = False
_pending_transfer = None
_bond_time = None
_bond_local = None
_swept = False
_bond_intact = False
_unloaded_angle = None
_final_drift = 0.


def on_reset():
    global _drive,_joints,_listener,_loaded_s,_first_release,_baseline_drift,_return_from_angle
    global _lifted,_pending_transfer,_bond_time,_bond_local,_swept,_bond_intact,_unloaded_angle,_final_drift
    if _joints:
        pickup, bond = _joints
        bond.GetJointEnabledAttr().Set(False)
        bond.GetCollisionEnabledAttr().Set(True)
        pickup.GetJointEnabledAttr().Set(True)
        pickup.GetCollisionEnabledAttr().Set(False)
    _drive = _joints = _listener = None
    _first_release = _return_from_angle = _pending_transfer = _bond_time = _bond_local = _unloaded_angle = None
    _loaded_s = _baseline_drift = _final_drift = 0.
    _lifted = _swept = _bond_intact = False


def relative_tape_position(objects,context):
    frame = quat_xyzw_to_matrix(np.asarray(objects[ROTOR.name].quat_xyzw))
    origins = context['body_origins']
    return frame.T@(origins[TAPE.name]-origins[ROTOR.name])


def rotor_angle(objects):
    q = objects[ROTOR.name].quat_xyzw
    return 2.*math.atan2(q[1],q[3])


def observes_grip(context):
    origin = context['body_origins'][TAMP.name]
    return any(context['gripper'][side]<.68 and math.dist(tcp,origin)<.040
               for side,tcp in context['tcp'].items())


def angular_distance(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


def observe(objects,context,dt):
    global _loaded_s,_first_release,_baseline_drift,_lifted,_swept,_unloaded_angle,_final_drift
    time = context['time_s']
    spin = rotor_angle(objects)
    if 1.2<=time<3.65:
        if _first_release is None:
            _first_release = spin
        _baseline_drift = max(_baseline_drift,angular_distance(spin,_first_release))
    grasping = observes_grip(context)
    _lifted |= grasping and objects[TAMP.name].low[2]>TOP+.090
    point = relative_tape_position(objects,context)
    if _bond_time is None:
        pressure = context['contact_forces']['tape_on_blade'][2]
        centered = math.dist(point,TAPE_SEAT)<.002
        backing = context['contact_forces']['blade_on_backing'][2]
        supported = .20<backing<=MAXIMUM_BACKING_FORCE_N
        qualifies = (time>=5.2 and _lifted and grasping and centered and supported
                     and PRESS_FORCE_N<=pressure<=MAXIMUM_PRESS_FORCE_N)
        _loaded_s = _loaded_s+dt if qualifies else 0.
        return
    elapsed = time-_bond_time
    if (4.0<=elapsed<=5.9 and _bond_intact and abs(spin)>=math.radians(64.)
            and math.dist(point,_bond_local)<.002):
        _swept = True
    if elapsed>=7.:
        if _unloaded_angle is None:
            _unloaded_angle = spin
        _final_drift = max(_final_drift,angular_distance(spin,_unloaded_angle))


def transfer_strip():
    global _bond_time,_bond_local,_bond_intact,_listener
    import omni.physx
    from omni.physx.bindings._physx import SimulationEvent
    from pxr import Gf, PhysicsSchemaTools

    pickup,bond = _joints
    assert pickup.GetJointEnabledAttr().Get() is False and not bond.GetJointEnabledAttr().Get()
    rotor_rot = quat_xyzw_to_matrix(np.asarray(LIVE_OBJECTS[ROTOR.name].quat_xyzw))
    tape_rot = quat_xyzw_to_matrix(np.asarray(LIVE_OBJECTS[TAPE.name].quat_xyzw))
    point = relative_tape_position(LIVE_OBJECTS,LIVE_CONTEXT)
    x,y,z,w = ik.matrix_to_quat_xyzw(rotor_rot.T@tape_rot)
    bond.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,point)))
    bond.GetLocalPos1Attr().Set(Gf.Vec3f(0.))
    bond.GetLocalRot0Attr().Set(Gf.Quatf(float(w),float(x),float(y),float(z)))
    bond.GetLocalRot1Attr().Set(Gf.Quatf(1.))
    bond_path = bond.GetPath()

    def on_break(event):
        global _bond_intact
        if event.type==int(SimulationEvent.JOINT_BREAK) and PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath'])==bond_path:
            _bond_intact = False
            print('NATIVE_TAMP_BREAK '+json.dumps({'time_s':LIVE_CONTEXT['time_s']}),flush=True)

    _listener = omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_break)
    # The application surface carries the strip before the bonded contact is
    # activated. The tamp no longer loads the same interface.
    bond.GetCollisionEnabledAttr().Set(False)
    bond.GetJointEnabledAttr().Set(True)
    _bond_local = point
    _bond_time = LIVE_CONTEXT['time_s']
    _bond_intact = True
    print('NATIVE_TAMP_TRANSFER '+json.dumps({'time_s':_bond_time,'tape_local_m':point.tolist(),
          'pressure_s':_pending_transfer['pressure_s'],
          'tape_contact_n':LIVE_CONTEXT['contact_forces']['tape_on_blade'][2],
          'unbalanced_drift_deg':math.degrees(_baseline_drift)}),flush=True)


def update_materials():
    global _drive,_joints,_return_from_angle,_pending_transfer
    import omni.usd
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if _drive is None:
        names = {ROTOR.name+'_joint',TAMP.name+'_joint',TAPE.name+'_joint'}
        joints = {prim.GetName():prim for prim in stage.Traverse() if prim.GetName() in names}
        if set(joints)!=names:
            raise RuntimeError('native applicator, propeller or bond constraint is missing')
        _drive = UsdPhysics.DriveAPI(joints[ROTOR.name+'_joint'],'angular')
        _joints = (UsdPhysics.FixedJoint(joints[TAMP.name+'_joint']),UsdPhysics.FixedJoint(joints[TAPE.name+'_joint']))

    now = LIVE_CONTEXT['time_s']
    if _pending_transfer is None and _loaded_s>=PRESS_DURATION_S:
        _pending_transfer = {'time_s':now,'pressure_s':_loaded_s,
                             'tape_local_m':relative_tape_position(LIVE_OBJECTS,LIVE_CONTEXT).tolist(),
                             'tape_contact_n':LIVE_CONTEXT['contact_forces']['tape_on_blade'][2]}
        pickup,_ = _joints
        pickup.GetJointEnabledAttr().Set(False)
        pickup.GetCollisionEnabledAttr().Set(True)
        print('NATIVE_TAMP_PRESS '+json.dumps(_pending_transfer),flush=True)
    if _pending_transfer is not None and _bond_time is None:
        point = relative_tape_position(LIVE_OBJECTS,LIVE_CONTEXT)
        o = LIVE_CONTEXT['body_origins']
        separation = o[TAMP.name][2]-o[TAPE.name][2]+TAPE_UNDER_TOOL[2]
        pressure = LIVE_CONTEXT['contact_forces']['tape_on_blade'][2]
        if (.15<=now-_pending_transfer['time_s']<=2.5 and separation>.0015
                and .00004<pressure<.08 and LIVE_OBJECTS[TAPE.name].resting()
                and math.dist(point,_pending_transfer['tape_local_m'])<.001
                and math.dist(point,TAPE_SEAT)<.002):
            transfer_strip()
    if _bond_time is None:
        # A separate unpowered release exposes the original heavy side.
        if now<3.65:
            target = 22.*ik.smoothstep(now/.80) if now<1.2 else 0.
        else:
            # Capture the actual angle at the end of the free swing. Slew
            # back to the support instead of striking it at shutter speed.
            if _return_from_angle is None:
                _return_from_angle = math.degrees(rotor_angle(LIVE_OBJECTS))
            target = .30+(_return_from_angle-.30)*(1.-ik.smoothstep((now-3.65)/2.1))
        stiffness = 0. if 1.2<=now<3.65 else 1.20
    else:
        elapsed = now-_bond_time
        target = 70.*ik.smoothstep((elapsed-2.55)/1.4)-40.*ik.smoothstep((elapsed-5.4)/1.3)
        stiffness = 0. if elapsed>=7. else .035 if elapsed>=2.55 else 1.20
    _drive.GetTargetPositionAttr().Set(target)
    _drive.GetStiffnessAttr().Set(stiffness)
    _drive.GetDampingAttr().Set(.008 if stiffness else 1e-6)
    _drive.GetMaxForceAttr().Set(.11)


def success(objects):
    if _bond_time is None or LIVE_CONTEXT['time_s']-_bond_time<PROOF_DURATION_S:
        return False
    tool = objects[TAMP.name]
    origin = LIVE_CONTEXT['body_origins'][TAMP.name]
    released = (math.dist(origin,TAMP.pos)<.013 and tool.resting()
                and LIVE_CONTEXT['contact_forces']['tool_on_stand'][2]>.1
                and all(math.dist(tcp,origin)>.065 for tcp in LIVE_CONTEXT['tcp'].values()))
    retained = math.dist(relative_tape_position(objects,LIVE_CONTEXT),_bond_local)<.002
    return bool(_bond_intact and _swept and retained and released
                and _baseline_drift>=math.radians(15.) and _final_drift<math.radians(7.)
                and abs(objects[ROTOR.name].ang_vel[1])<.15)


def score(objects):
    if success(objects):
        return 1.
    return .60 if _bond_intact and _swept else .35 if _bond_intact else .15 if _lifted else 0.


def demo():
    pick = (*TAMP.pos[:2],TAMP.pos[2]+PAD+GRIP_HEIGHT-.006)
    lift = (*pick[:2],TOP+.200)
    z = AXIS[2]+TAPE_SEAT[2]-TAPE_UNDER_TOOL[2]+PAD+GRIP_HEIGHT-.006
    press = (AXIS[0]-TIP_RADIUS,AXIS[1],z-.006)
    over_blade = (*press[:2],z+.018)
    return {'left': [
        Waypoint(2.0,lift,OPEN,'approach applicator'),
        Waypoint(1.2,pick,OPEN,'align around applicator'),
        Waypoint(.75,pick,CLOSED,'grasp applicator'),
        Waypoint(.9,lift,CLOSED,'lift applicator'),
        Waypoint(1.45,over_blade,CLOSED,'carry over light blade'),
        Waypoint(.9,over_blade,CLOSED,'center above the blade'),
        Waypoint(1.3,press,CLOSED,'press the balancing strip'),
        Waypoint(.85,press,CLOSED,'hold pressure'),
        Waypoint(.85,over_blade,CLOSED,'lift applicator off strip'),
        Waypoint(1.4,lift,CLOSED,'carry to the tool stand'),
        Waypoint(1.05,pick,CLOSED,'seat applicator in stand'),
        Waypoint(.55,pick,OPEN,'release applicator'),
        Waypoint(.85,lift,OPEN,'clear the tool stand'),
        Waypoint(8.5,lift,OPEN,'wait for the balance proof'),
    ]}


def demo_target(arm,label,t,pos,rotation,finger):
    if arm!='left' or not LIVE_CONTEXT or not observes_grip(LIVE_CONTEXT):
        return pos,rotation,finger
    origins = LIVE_CONTEXT['body_origins']
    tcp = LIVE_CONTEXT['tcp'][arm]
    tip = origins[TAPE.name] if _bond_time is None else origins[TAMP.name]+TAPE_UNDER_TOOL
    if label in {'center above the blade','press the balancing strip','hold pressure'}:
        frame = quat_xyzw_to_matrix(np.asarray(LIVE_OBJECTS[ROTOR.name].quat_xyzw))
        # Bias the hover target before the foil contacts the blade. Once the
        # strip is loaded, lateral wrist motion mostly twists the applicator.
        approach_bias = np.array((.00140,-.00185))
        target_xy = origins[ROTOR.name][:2]+(frame@TAPE_SEAT)[:2]+approach_bias
        adjust = np.clip((tcp-tip)[:2]+target_xy-pos[:2],-.009,.009)
        pos = pos+np.r_[adjust,0.]
    elif label in {'seat applicator in stand','release applicator'}:
        adjust = np.clip((tcp-origins[TAMP.name])[:2],-.008,.008)
        pos = pos+np.r_[adjust,0.]
    if (_pending_transfer is not None and label in
            {'center above the blade','press the balancing strip','hold pressure','lift applicator off strip'}):
        # Keep the actual early retract through the nominal lift segment.
        # Dropping back to its starting height strikes the newly bonded foil.
        clear_height = AXIS[2]+TAPE_SEAT[2]-TAPE_UNDER_TOOL[2]+PAD+GRIP_HEIGHT+.012
        pos = pos.copy()
        pos[2] = max(pos[2],clear_height)
    return pos,rotation,finger
