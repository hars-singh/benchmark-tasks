"""Prototype: acquire a bench plane from a moving carrier and plane a marked face.

Native contacts and two independent grippers carry the free tool. A disclosed
material rule removes pencil marks only along loaded forward cutting strokes.
No tool or workpiece pose is assigned during execution. Not submission-ready.
"""
import math
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,SHOULDER,WRIST,FINGER_OPEN as OPEN,FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.task_file import Box,TABLE_TOP
from dynamic_task_moving_carrier_hand_planing.geometry import Solid
from dynamic_task_moving_carrier_hand_planing.motion import OscillatingPlatform
from dynamic_task_moving_carrier_hand_planing.mechanisms import linear_stage
from dynamic_task_moving_carrier_hand_planing.scoring import _rotate
from dynamic_task_moving_carrier_hand_planing.passive_loads import PlaningResistance

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
STAGE=Box('stage',pos=(.30,0,TOP+.015),size=(.14,.32,.03),mass=5.,color=(.25,.25,.28))
PLANE=Box('part',pos=(.30,0,TOP+.0331),size=(.065,.27,.08),mass=1.8,color=(.12,.13,.14))
STOCK=Box('stock',pos=(.15,0,TOP+.0625),size=(.06,.40,.025),color=(.56,.35,.16))
OBJECTS=[STAGE,PLANE,STOCK,
         Box('vise_base',pos=(.15,0,TOP+.025),size=(.10,.42,.05),color=(.12,.20,.28)),
         *(Box('vise_jaw_'+str(i),pos=(.15+side*.035,0,TOP+.06),size=(.01,.40,.02),color=(.5,.54,.58))
           for i,side in enumerate((-1,1)))]
WOOD=(.34,.12,.045)
CHISEL_Y=.020+.020/math.sqrt(2)-.001/math.sqrt(2)
BLADE_LOCAL=np.array((0,CHISEL_Y,-.0031))
blade_z=.020/math.sqrt(2)+.001/math.sqrt(2)-.0031
heel_end=CHISEL_Y-.002;toe_start=CHISEL_Y+.002
BODY_PARTS={'part':(
    Solid('box',(.065,heel_end+.135,.006),(0,(heel_end-.135)/2,0)),
    Solid('box',(.065,.135-toe_start,.006),(0,(toe_start+.135)/2,0)),
    *(Solid('box',(.004,.27,.025),(side*.0305,0,.0125)) for side in (-1,1)),
    Solid('box',(.051,.040,.002),(0,.020,blade_z),roll=-math.pi/4,color=(.65,.67,.69)),
    Solid('cylinder',(.014,.022),(0,.10,.014),color=WOOD),
    Solid('sphere',(.022,),(0,.10,.040),color=WOOD),
    Solid('box',(.022,.045,.020),(0,-.10,.067),color=WOOD),
    Solid('box',(.022,.020,.055),(0,-.122,.038),roll=.25,color=WOOD),
    Solid('box',(.025,.025,.035),(0,-.080,.025),color=WOOD),
    Solid('cylinder',(.008,.012),(0,-.025,.030),axis='Y',color=(.65,.44,.12)))}
hardware,parts=linear_stage(STAGE,.03);OBJECTS.extend(hardware);BODY_PARTS.update(parts)
MACHINE_VISUALS={'stage':{'kind':'linear_stage','amplitude':.03}}
MOTIONS={'stage':OscillatingPlatform(STAGE.pos,(0,.03,0),8.)}
SUPPORT_JOINTS={'stage':{'axis':'Y','lower':-.05,'upper':.05}}
CONTACT_PAIRS={'plane_stock':('part','stock')}
PASSIVE_CUTTING_LOADS={'part':PlaningResistance(tuple(BLADE_LOCAL),
    (STOCK.pos[0]-.030,-.20,TOP+.075-.0015),(STOCK.pos[0]+.030,.20,TOP+.075+.0015),
    'plane_stock',force_N=25.)}
CONTACT_PATCH_RADII={'part':(.003,.001)}
RIGID_SOLVER_ITERATIONS={'part':(32,16)}
SOURCE_R={'left':grip_rotation(-math.pi/2),'right':grip_rotation(math.pi/2)}
# Engage below the knob equator and across more of the tote's upper bar.
# Native V6 closed the left jaws empty at 12 s after the shallow grasp slipped.
GRIPS={'left':np.array((0,.10,.047)),'right':np.array((0,-.10,.074))}
LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.75
MARK_MIN=-.045
MARK_MAX=.045
MARK_Y=np.linspace(MARK_MIN+.0025,MARK_MAX-.0025,18)
# Author the graphite before the reset's renderer warmup. Removing a stripe
# changes visibility only, so it cannot leave a pale replacement shader.
VISUAL_PARTS={'stock':tuple(Solid('box',(.040,.0015,.00002),(0,float(y),.01252),
                                color=(.035,.035,.035)) for y in MARK_Y)}
_intervals=[]
_previous=None
_grasp={}
_press=0.
_force_time=None
_markers=None
_xy_compensation=np.zeros(2)
_xy_last=np.zeros(2)
_compensation_time=None
_attitude_integral=np.zeros(3)
_attitude_time=None
_attitude_goal=np.eye(3)
FINISH_PASS_S=4.
SECURE_HANDLES_S=.6
CUT_START_S=9.
CUT_DURATION_S=2.
LIFT_OFF_START_S=CUT_START_S+CUT_DURATION_S+FINISH_PASS_S
RETURN_START_S=LIFT_OFF_START_S+.8
RETREAT_START_S=RETURN_START_S+1.8+1.2+.8
FORWARD_FEED_PER_ARM_N=35.
MOTOR_BIAS_FRACTION=.85

def on_reset():
    global _intervals,_previous,_grasp,_press,_force_time,_markers,_xy_compensation,_compensation_time,_xy_last,_attitude_integral,_attitude_time,_attitude_goal
    _intervals=[];_previous=None;_grasp={};_press=0.;_force_time=None;_markers=None
    _xy_compensation=np.zeros(2);_xy_last=np.zeros(2);_compensation_time=None
    _attitude_integral=np.zeros(3);_attitude_time=None;_attitude_goal=np.eye(3)

def matrix(q):
    return np.column_stack([_rotate(tuple(v),tuple(q)) for v in np.eye(3)])

def add_interval(a,b):
    global _intervals
    a=max(a,MARK_MIN);b=min(b,MARK_MAX)
    if b<=a:return
    result=[]
    for low,high in sorted(_intervals+[(a,b)]):
        if result and low<=result[-1][1]+1e-7:result[-1]=(result[-1][0],max(result[-1][1],high))
        else:result.append((low,high))
    _intervals=result

def coverage():return min(1.,sum(b-a for a,b in _intervals)/(MARK_MAX-MARK_MIN))

def observe(objects,context,dt):
    global _previous
    body=np.asarray(context['body_origins']['part']);R=matrix(objects['part'].quat_xyzw)
    blade=body+R@BLADE_LOCAL;relative=blade-np.asarray(STOCK.pos)
    half=R[:,0]*.051/2
    full_width=(min(blade[0]-half[0],blade[0]+half[0])<=STOCK.pos[0]-.020
                and max(blade[0]-half[0],blade[0]+half[0])>=STOCK.pos[0]+.020)
    load=abs(float(context.get('contact_forces',{}).get('plane_stock',np.zeros(3))[2]))
    cutting=-float(context.get('passive_loads',{}).get('part',{}).get('force_world_N',np.zeros(3))[1])
    # The blade cuts toward the toe. Moving a reversed plane heel-first is
    # rubbing, even when the sole is level and the normal load is plausible.
    valid=bool(full_width and R[2,2]>=math.cos(math.radians(8)) and R[1,1]>.7
               and abs(blade[2]-(TOP+.075))<=.0015 and 20.<=load<=80. and cutting>=20.)
    if _previous is not None:
        last,last_valid=_previous;dy=relative[1]-last[1]
        if valid and last_valid and 0.<dy<=.20*dt:add_interval(float(last[1]),float(relative[1]))
    _previous=(relative.copy(),valid)

def success(objects):
    if coverage()<.99:return False
    c=LIVE_CONTEXT;part=objects['part'];stage=np.asarray(c['body_origins']['stage'])
    contained=(np.all(part.low[:2]>=stage[:2]-np.array(STAGE.size[:2])/2+.005)
               and np.all(part.high[:2]<=stage[:2]+np.array(STAGE.size[:2])/2-.005))
    top=stage[2]+.015
    supported=top-.001<=part.low[2]<=top+.004
    relative_speed=np.linalg.norm(part.lin_vel-objects['stage'].lin_vel)<.010
    clear=all(np.linalg.norm(np.maximum(np.maximum(part.low-p,p-part.high),0))>.10 for p in c['tcp'].values())
    return bool(contained and supported and relative_speed and np.linalg.norm(part.ang_vel)<.5 and clear)

def score(objects):
    if objects['part'].low[2]<TOP-.03:return 0.
    return 1. if success(objects) else .75*coverage()

def update_materials():
    global _markers
    import omni.usd
    from pxr import UsdGeom
    stage=omni.usd.get_context().get_stage()
    if _markers is None:
        roots=[p for p in stage.Traverse() if p.GetName()=='stock'];assert len(roots)==1
        _markers=[]
        for i,y in enumerate(MARK_Y):
            path=str(roots[0].GetPath())+f'/geometry/visual_{i}'
            imageable=UsdGeom.Imageable(stage.GetPrimAtPath(path))
            if not imageable:raise RuntimeError('initial graphite mark was not authored at spawn')
            _markers.append((float(y),imageable))
    for y,imageable in _markers:
        removed=any(a<=y-.00075 and b>=y+.00075 for a,b in _intervals)
        target=UsdGeom.Tokens.invisible if removed else UsdGeom.Tokens.inherited
        if imageable.GetVisibilityAttr().Get()!=target:imageable.CreateVisibilityAttr().Set(target)

def reset_visuals():
    if _markers:
        for _,imageable in _markers:imageable.MakeVisible()

def capture_grasp(arm,label):
    if arm in _grasp or label not in {'lift','carry','lower','settle','cut','finish pass','lift off','return','lower carrier','release'}:return
    c=LIVE_CONTEXT
    if not c or c['gripper'][arm]>.8:return
    body=np.asarray(c['body_origins']['part']);R=matrix(LIVE_OBJECTS['part'].quat_xyzw)
    _grasp[arm]=(R.T@(c['tcp'][arm]-body),R.T@c['tcp_rotation'][arm])

def process_time(t):
    return min(t,3.8) if t<3.8+SECURE_HANDLES_S else t-SECURE_HANDLES_S

def cooperative_rotation(label,t):
    """Compensate the measured free tool's attitude through both robot wrists."""
    global _attitude_integral,_attitude_time,_attitude_goal
    if label in {'lower','settle','cut','finish pass'} and _grasp:
        if _attitude_time!=LIVE_CONTEXT['time_s']:
            actual=matrix(LIVE_OBJECTS['part'].quat_xyzw)
            error=ik._rot_error(np.eye(3),actual)
            _attitude_integral=np.clip(_attitude_integral+.025*error,-math.radians(8),math.radians(8))
            correction=_attitude_integral+.5*error
            correction*=min(1.,math.radians(12)/max(np.linalg.norm(correction),1e-12))
            from yam_arena.urdf import axis_angle_matrix
            angle=float(np.linalg.norm(correction))
            _attitude_goal=(axis_angle_matrix(correction/angle,angle) if angle>1e-10 else np.eye(3))
            _attitude_time=LIVE_CONTEXT['time_s']
        return _attitude_goal
    if label=='lift off':return _attitude_goal
    if label=='return':
        blend=ik.smoothstep(np.clip((t-RETURN_START_S)/1.8,0.,1.))
        return ik.slerp_matrix(_attitude_goal,np.eye(3),blend)
    return np.eye(3)

def demo_rotation(arm,label,t,default):
    t=process_time(t)
    capture_grasp(arm,label)
    if arm in _grasp:
        if label=='retreat':return ik.slerp_matrix(_grasp[arm][1],SOURCE_R[arm],ik.smoothstep(np.clip((t-RETREAT_START_S)/1.2,0.,1.)))
        if label!='done':return cooperative_rotation(label,t)@_grasp[arm][1]
    initial=ik.tcp_pose(np.asarray(READY),station.BASES[arm])[1]
    return ik.slerp_matrix(initial,SOURCE_R[arm],ik.smoothstep(min(t/1.5,1.)))

def demo_target(arm,label,t,pos,rotation,finger):
    global _press,_force_time,_xy_compensation,_compensation_time,_xy_last
    t=process_time(t)
    c=LIVE_CONTEXT;shift=np.asarray(c['body_origins']['stage'])-np.asarray(STAGE.pos)
    if label in {'to tool','descend','close','secure handles'}:
        pos=pos+np.asarray(c['body_origins']['part'])-np.asarray(PLANE.pos)
        return pos,rotation,finger
    if label in {'retreat','done'}:
        if label=='retreat':
            blend=1.-ik.smoothstep(np.clip((t-RETREAT_START_S)/1.2,0.,1.))
            pos=pos+blend*(shift+(_grasp[arm][0]-GRIPS[arm] if arm in _grasp else 0))
        return pos,rotation,finger
    body_goal=pos-GRIPS[arm]
    if label=='lift':body_goal+=shift
    elif label=='carry':body_goal+=shift*(1.-ik.smoothstep(np.clip((t-5.)/1.8,0.,1.)))
    elif label=='return':body_goal+=shift*ik.smoothstep(np.clip((t-RETURN_START_S)/1.8,0.,1.))
    elif label in {'lower carrier','release'}:body_goal+=shift
    R=cooperative_rotation(label,t)
    nominal_blade=body_goal+BLADE_LOCAL
    # Rotate about the working edge rather than moving that edge as the wrists
    # correct the tool's attitude. All resulting targets still go to joints.
    body_goal=nominal_blade-R@BLADE_LOCAL
    if label in {'settle','cut','finish pass'} and _compensation_time!=c['time_s']:
        actual_blade=np.asarray(c['body_origins']['part'])+matrix(LIVE_OBJECTS['part'].quat_xyzw)@BLADE_LOCAL
        error=nominal_blade[:2]-actual_blade[:2]
        _xy_compensation=np.clip(_xy_compensation+np.clip(np.array((.05,.025))*error,-.0005,.0005),-.025,.025)
        _xy_last=np.clip(_xy_compensation+np.array((1.,.5))*error,-.035,.035)
        _compensation_time=c['time_s']
    if label in {'settle','cut','finish pass','lift off'}:
        body_goal=body_goal.copy();body_goal[:2]+=_xy_last
    elif label=='return':
        body_goal=body_goal.copy();body_goal[:2]+=_xy_last*(1.-ik.smoothstep(np.clip((t-RETURN_START_S)/1.8,0.,1.)))
    if label in {'lower','settle','cut','finish pass'}:
        if _force_time!=c['time_s']:
            load=abs(float(c.get('contact_forces',{}).get('plane_stock',np.zeros(3))[2]))
            body=np.asarray(c['body_origins']['part'])
            blade=body+matrix(LIVE_OBJECTS['part'].quat_xyzw)@BLADE_LOCAL
            if blade[2]-(TOP+.075)<.008 or load>0.:
                _press=float(np.clip(_press+.0001*np.clip((35.-load)/35.,-1.,1.),0.,.006))
            _force_time=c['time_s']
        body_goal=body_goal.copy();body_goal[2]-=_press
    elif label=='lift off':
        body_goal=body_goal.copy();body_goal[2]-=_press*(1.-ik.smoothstep(np.clip((t-LIFT_OFF_START_S)/.8,0.,1.)))
    offset=_grasp[arm][0] if arm in _grasp else GRIPS[arm]
    return body_goal+R@offset,rotation,finger

def demo_joint_target(arm,label,t,q):
    """Position-target force bias through the unchanged native robot motors.

    YAM's wrist position gains are deliberately compliant. A small Cartesian
    pose offset alone did not sustain the required tool load. This is a robot
    controller, not a force applied to the task object or changed robot gains.
    """
    if label not in {'settle','cut','finish pass'}:return q
    context=LIVE_CONTEXT
    normal=abs(float(context.get('contact_forces',{}).get('plane_stock',np.zeros(3))[2]))
    if normal<.1:return q
    # Split the nominal extra normal force and forward resistance between the
    # two grasps. The material's passive load still acts independently.
    feed=FORWARD_FEED_PER_ARM_N if label=='cut' else 0.
    if label=='finish pass':
        blade=np.asarray(context['body_origins']['part'])+matrix(LIVE_OBJECTS['part'].quat_xyzw)@BLADE_LOCAL
        feed=FORWARD_FEED_PER_ARM_N*float(np.clip((.090-blade[1])/.020,0.,1.))
    force=np.array((0.,feed,-(35.-PLANE.mass*9.81)/2))
    point=ik.tcp_pose(q,station.BASES[arm])[0];jac=np.empty((3,6))
    for i in range(6):
        perturbed=q.copy();perturbed[i]+=1e-6
        jac[:,i]=(ik.tcp_pose(perturbed,station.BASES[arm])[0]-point)/1e-6
    gains=np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
    effort=np.array([SHOULDER['effort']]*3+[WRIST['effort']]*3)
    # A uniform 0.2 rad cap under-used the wrist's available drive torque and
    # stalled against sole friction plus the declared 25 N cutting load.
    # Bound each bias by its actual, unchanged motor effort/gain instead.
    limit=MOTOR_BIAS_FRACTION*effort/gains
    bias=np.clip(jac.T@force/gains,-limit,limit)
    return np.clip(q+bias,ik.LOWER,ik.UPPER)

def demo():
    start=np.asarray(PLANE.pos);hover=start+np.array((0,0,.060))
    work=np.array((STOCK.pos[0],-.090-CHISEL_Y,TOP+.0781));end=work+np.array((0,.180,0))
    phases=[(1.5,hover,OPEN,'to tool'),(1.5,start,OPEN,'descend'),(.8,start,CLOSED,'close'),
            (SECURE_HANDLES_S,start,CLOSED,'secure handles'),
            (1.2,hover,CLOSED,'lift'),(1.8,work+np.array((0,0,.015)),CLOSED,'carry'),
            (1.2,work,CLOSED,'lower'),(1.,work,CLOSED,'settle'),(CUT_DURATION_S,end,CLOSED,'cut'),
            (FINISH_PASS_S,end,CLOSED,'finish pass'),
            (.8,end+np.array((0,0,.015)),CLOSED,'lift off'),(1.8,hover,CLOSED,'return'),
            (1.2,start,CLOSED,'lower carrier'),(.8,start,OPEN,'release')]
    plans={}
    for arm in ('left','right'):
        plans[arm]=[Waypoint(d,tuple(p+GRIPS[arm]),f,label) for d,p,f,label in phases]
        clear=np.array((.12,.28 if arm=='left' else -.28,TOP+.19))
        plans[arm]+=[Waypoint(1.2,tuple(clear),OPEN,'retreat'),Waypoint(3.,tuple(clear),OPEN,'done')]
    return plans
