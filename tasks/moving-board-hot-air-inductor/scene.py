"""Hot-air removal of a moulded power inductor from a clamped PCB.

All acquisition, extraction and placement use native free bodies and robot
grippers. A declared thermal rule releases an initial material bond only when
both soldered terminals are molten. Native cold fracture permanently damages pads.
A surface-mount power inductor supplies graspable case area for the standard
fingertips. Visible case surfaces absorb hot air before conduction heats two
underside solder pads.
The geometry follows the reference footprint; the thermal coefficients are nominal.
"""
import math
import json
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN as OPEN,FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint,grip_rotation
from yam_arena.task_file import Box,TABLE_TOP
from dynamic_task_moving_board_hot_air_inductor.geometry import Solid
from dynamic_task_moving_board_hot_air_inductor.scoring import _rotate

SCENE='station'
RENDER_ANTIALIASING='DLAA'
TOP=TABLE_TOP[SCENE]
TOOL_ANGLE=math.radians(32.)
DIR=np.array((0.,math.cos(TOOL_ANGLE),-math.sin(TOOL_ANGLE)))
NOZZLE_LOCAL=.205*DIR
STAGE=Box('stage',pos=(.24,0,TOP+.015),size=(.12,.10,.030),mass=5.,color=(.25,.25,.28))
BOARD=Box('board',pos=(.24,0,TOP+.060),size=(.08,.06,.003),mass=.03,color=(.04,.27,.10))
COMPONENT=Box('part',pos=(.24,0,TOP+.0667),size=(.0152,.0162,.010),mass=.013,color=(.22,.23,.24))
GUN=Box('hot_air',pos=(.30,-.18,TOP+.14),size=(.20,.024,.20),mass=.180,color=(.10,.22,.48))
TRAY=np.array((.35,.20))
RESTS=[]
for sign,name in ((-1,'back'),(1,'front')):
 height=.14-sign*.035*math.sin(TOOL_ANGLE)-.012*math.cos(TOOL_ANGLE)
 RESTS.append(Box('gun_stand_'+name,pos=(.30,-.18+sign*.035*math.cos(TOOL_ANGLE),TOP+height/2),
                  size=(.05,.006,height),color=(.30,.33,.36)))
OBJECTS=[STAGE,BOARD,COMPONENT,GUN,*RESTS,
 Box('tray_floor',pos=(*TRAY,TOP+.005),size=(.10,.10,.01),color=(.45,.49,.53)),
 *(Box('tray_x_'+('low' if s<0 else 'high'),pos=(TRAY[0]+s*.055,TRAY[1],TOP+.025),size=(.01,.12,.05),color=(.45,.49,.53)) for s in (-1,1)),
 *(Box('tray_y_'+('low' if s<0 else 'high'),pos=(TRAY[0],TRAY[1]+s*.055,TOP+.025),size=(.10,.01,.05),color=(.45,.49,.53)) for s in (-1,1))]
CORE_HALF=np.array((.0076,.0081,.0049))
SOLDER_PADS=np.array([(s*.0053,0.,-.0050) for s in (-1,1)])
LEADS=SOLDER_PADS.copy()
HEAT_DWELL_S=35.
HEAT_START_S=8.
HOT_GRASP_HOLD_S=1.5
SCAN_DWELL_S=.8
SCAN_TRAVEL_S=.15
NOZZLE_SIDE_X=.016
HEATING_TAU_S=6.
COOLING_TAU_S=20.
CASE_TO_PAD_TAU_S=2.
BODY_PARTS={
 'part':(Solid('box',tuple(2*CORE_HALF)),
         *(Solid('box',(.003,.0125,.0002),tuple(v),color=(.65,.67,.69)) for v in SOLDER_PADS)),
 'board':(Solid('box',BOARD.size),
          *(Solid('box',(.00318,.0132,.0001),(v[0],v[1],.00155),color=(.65,.67,.69)) for v in SOLDER_PADS),
          *(Solid('box',(.0063,.0032,.002),(-.025,.018-i*.018,.0025),color=(.18,.18,.19)) for i in range(3))),
 'hot_air':(Solid('cylinder',(.012,.090),axis='Y',roll=-TOOL_ANGLE),
            Solid('cylinder',(.009,.090),tuple(.090*DIR),axis='Y',roll=-TOOL_ANGLE,color=(.55,.57,.60)),
            Solid('cylinder',(.004,.070),tuple(.170*DIR),axis='Y',roll=-TOOL_ANGLE,color=(.62,.64,.66))),
}
VISUAL_PARTS={
 'hot_air':(Solid('cylinder',(.0015,.0001),tuple(NOZZLE_LOCAL+.00006*DIR),
                  axis='Y',roll=-TOOL_ANGLE,color=(.015,.015,.018)),),
 'board':tuple(Solid('box',(.02311,.001,.00002),(s*.018445,0,.00152),color=(.65,.36,.14))
               for s in (-1,1))+
         tuple(Solid('cylinder',(.0015,.0001),(s*.030,0,.00155),color=(.65,.36,.14))
               for s in (-1,1))}
for rest in RESTS:
 BODY_PARTS[rest.name]=(Solid('box',rest.size),
     *(Solid('box',(.006,.006,.012),(s*.0155,0,rest.size[2]/2+.006)) for s in (-1,1)))
BODY_PARTS['stage']=(
 Solid('box',STAGE.size),
 Solid('box',(.030,.030,.0285),(0,0,.02925),color=(.58,.63,.68)),
 *(Solid('box',(.009,.018,.034),(side*.033,-.032,.032),color=(.42,.46,.49)) for side in (-1,1)),
 *(Solid('box',(.009,.018,.004),(side*.033,-.027,.049),color=(.64,.66,.68)) for side in (-1,1)),
)
SUPPORT_JOINTS={'stage':{'kind':'fixed'}}
STATIONARY_REWORK_BOARD_TOLERANCE=.001
PAYLOAD_SUPPORTS={'part':{'body':'board','top_local_z':.0015,'penetration_tolerance_m':.0005}}
OBJECT_JOINTS={'board':{'kind':'fixed','parent':'stage','anchor':BOARD.pos},
 'part':{'kind':'fixed','parent':'board','anchor':COMPONENT.pos,'break_force_N':20.,'break_torque_Nm':.08,
         'collision_enabled':True}}
CONTACT_PATCH_RADII={'hot_air':(.003,.001),'part':(.001,.0003)}
CONTACT_PAIRS={rest.name:('hot_air',rest.name) for rest in RESTS}
CONTACT_FRICTION_PAIRS=set(CONTACT_PAIRS)
RIGID_SOLVER_ITERATIONS={'part':(32,16),'hot_air':(16,8),'board':(16,8)}
LIVE_CONTEXT={}
SUCCESS_HOLD_S=1.
MAX_PARTIAL_SCORE=.75
# This scene's hook follows actual actor origins once. The generic tracker
# follows AABB centres; the long handpiece's AABB is offset from its handle.
TRACK_OBJECTS={}
LEFT_R=grip_rotation()
RIGHT_R=grip_rotation(math.pi/2)
COMPONENT_TCP_OFFSET=np.array((0,0,.0110))
GUN_GRASP_OFFSET=np.array((0,0,.006))
GUN_OPEN=-.018
_temperatures=np.full(len(LEADS),25.)
_case_temperatures=np.full(len(LEADS),25.)
_overheat=np.zeros(len(LEADS))
_damaged=False
_bond_released=False
_removed=False
_joint=None
_subscription=None
_gun_home=None
_native_break_seen=False
_lift_origin=None
_native_events=[]
_pad_shaders=None
_gun_grasp=None
_gun_start_R=None
_gun_source_body=None
_gun_grasp_update_time=None
_wet_contacts=np.zeros(len(LEADS),dtype=bool)
_cold_anchor=np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)
_demo_reflow_time=None
_demo_left_grasp=None
_demo_lift_tcp=None
_nozzle_integral=np.zeros(3)
_nozzle_correction=np.zeros(3)
_nozzle_feedback_time=None

def on_reset():
 global _temperatures,_overheat,_damaged,_bond_released,_removed,_joint,_subscription,_gun_home,_native_break_seen,_lift_origin,_native_events,_pad_shaders
 global _case_temperatures
 global _gun_grasp,_gun_start_R,_gun_source_body,_gun_grasp_update_time
 global _wet_contacts,_cold_anchor
 global _demo_reflow_time,_demo_left_grasp,_demo_lift_tcp
 global _nozzle_integral,_nozzle_correction,_nozzle_feedback_time
 _temperatures=np.full(len(LEADS),25.);_overheat=np.zeros(len(LEADS))
 _case_temperatures=np.full(len(LEADS),25.)
 _damaged=False;_bond_released=False;_removed=False
 _joint=None;_subscription=None;_gun_home=None;_native_break_seen=False;_lift_origin=None;_native_events=[]
 _pad_shaders=None
 _gun_grasp=None;_gun_start_R=None;_gun_source_body=None
 _gun_grasp_update_time=None
 _wet_contacts=np.zeros(len(LEADS),dtype=bool)
 _cold_anchor=np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)
 _demo_reflow_time=None;_demo_left_grasp=None;_demo_lift_tcp=None
 _nozzle_integral=np.zeros(3);_nozzle_correction=np.zeros(3);_nozzle_feedback_time=None

def matrix(q):return np.column_stack([_rotate(tuple(v),tuple(q)) for v in np.eye(3)])

def point(objects,context,name,local):
 return np.asarray(context['body_origins'][name])+matrix(objects[name].quat_xyzw)@np.asarray(local)

def segment_intersects_core(start,end):
 """Segment/core slab test for exposed original board-pad heating.

 The material rule rejects heat rays through the actual moulded body. It does
 not model CFD; surface heat conduction uses the stated lumped constants.
 """
 low=-CORE_HALF;high=CORE_HALF
 direction=np.asarray(end)-np.asarray(start);enter=0.;leave=1.
 for i in range(3):
  if abs(direction[i])<1e-12:
   if start[i]<low[i] or start[i]>high[i]:return False
  else:
   a=(low[i]-start[i])/direction[i];b=(high[i]-start[i])/direction[i]
   enter=max(enter,min(a,b));leave=min(leave,max(a,b))
   if enter>leave:return False
 return bool(leave>1e-6 and enter<1.-1e-6)

def case_surface_patches(cells=8):
 """Convex-case surface quadrature; no ray passes through the solid core.

 A heated exterior patch deposits heat into its actual lateral case half.
 A first-order conduction lag then transfers case heat to the underside solder.
 The four thermal nodes and coefficients are nominal, not calibrated CFD.
 """
 points=[];normals=[];areas=[];halves=[]
 for axis in range(3):
  others=[i for i in range(3) if i!=axis]
  for sign in (-1,1):
   for u in range(cells):
    for v in range(cells):
     point=np.zeros(3);point[axis]=sign*CORE_HALF[axis]
     point[others[0]]=-CORE_HALF[others[0]]+(u+.5)*2*CORE_HALF[others[0]]/cells
     point[others[1]]=-CORE_HALF[others[1]]+(v+.5)*2*CORE_HALF[others[1]]/cells
     normal=np.zeros(3);normal[axis]=sign
     points.append(point);normals.append(normal)
     areas.append(4*CORE_HALF[others[0]]*CORE_HALF[others[1]]/cells**2)
     halves.append(0 if point[0]<0 else 1)
 return np.asarray(points),np.asarray(normals),np.asarray(areas),np.asarray(halves)

CASE_POINTS,CASE_NORMALS,CASE_AREAS,CASE_HALVES=case_surface_patches()
CASE_REFERENCE_AREA=2*CORE_HALF[0]*CORE_HALF[2]

def case_heating_fractions(nozzle,direction,origin,rotation):
 points=np.asarray(origin)+(rotation@CASE_POINTS.T).T
 normals=(rotation@CASE_NORMALS.T).T
 delta=points-nozzle;axial=delta@direction
 radial=np.linalg.norm(delta-axial[:,None]*direction,axis=1)
 visible=np.sum(normals*(nozzle-points),axis=1)>1e-10
 heated=(axial>=.010)&(axial<=.045)&(radial<=.0015+axial*math.tan(math.radians(20)))&visible&(direction[2]<-.5)
 return np.array([min(1.,float(np.sum(CASE_AREAS[heated&(CASE_HALVES==side)]))/CASE_REFERENCE_AREA) for side in range(2)])

def scan_offset(t):
 phase=(max(0.,t-HEAT_START_S))%(2*(SCAN_DWELL_S+SCAN_TRAVEL_S))
 if phase<SCAN_DWELL_S:return -NOZZLE_SIDE_X
 if phase<SCAN_DWELL_S+SCAN_TRAVEL_S:
  q=ik.smoothstep((phase-SCAN_DWELL_S)/SCAN_TRAVEL_S)
  return NOZZLE_SIDE_X*(2*q-1)
 if phase<2*SCAN_DWELL_S+SCAN_TRAVEL_S:return NOZZLE_SIDE_X
 q=ik.smoothstep((phase-2*SCAN_DWELL_S-SCAN_TRAVEL_S)/SCAN_TRAVEL_S)
 return NOZZLE_SIDE_X*(1-2*q)

def seated_solder_contacts(objects,context):
 """Actual underside-pad overlap within a nominal 0.3 mm solder bridge.

 Clip each transformed terminal face against its board land and the bridge
 height. A rotated or partially overlapping face may remain in contact; a
 separated component must not acquire a bond merely because it has cooled.
 """
 board=np.asarray(context['body_origins']['board'])
 part=np.asarray(context['body_origins']['part'])
 R0=matrix(objects['board'].quat_xyzw);R1=matrix(objects['part'].quat_xyzw)
 if (R0.T@R1)[2,2]<=0:return np.zeros(len(LEADS),dtype=bool)
 corners=np.array([[-.0015,-.00625,-.0001],[.0015,-.00625,-.0001],
                   [.0015,.00625,-.0001],[-.0015,.00625,-.0001]])
 contacts=[]
 for pad in SOLDER_PADS:
  polygon=list((R0.T@(part-board+(R1@(corners+pad).T).T).T).T)
  centre=np.array((pad[0],pad[1],.0016))
  for axis,half in ((0,.00159),(1,.0066),(2,.0003)):
   for sign in (-1,1):
    clipped=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
     da=sign*(a[axis]-centre[axis])-half
     db=sign*(b[axis]-centre[axis])-half
     if da<=0:clipped.append(a)
     if (da<=0)!=(db<=0):clipped.append(a+(b-a)*da/(da-db))
    polygon=clipped
  area=0.
  if len(polygon)>=3:
   area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(polygon,polygon[1:]+polygon[:1])))/2
  contacts.append(area>1e-8)
 return np.asarray(contacts,dtype=bool)

def material_bond_action(objects,context):
 """Request a native transition; temperature math alone never changes a bond."""
 if _damaged:return None
 if not _bond_released:return 'release' if ready_to_release() else None
 if np.any(_wet_contacts&seated_solder_contacts(objects,context)&(_temperatures<217.)):
  return 'solidify'
 return None

def record_material_transition(action,objects,context):
 """Record only after a native joint transition has actually succeeded."""
 global _bond_released,_removed,_cold_anchor,_wet_contacts
 if action not in {'release','solidify'}:raise ValueError('unknown material transition')
 _bond_released=action=='release'
 if action=='solidify':
  _cold_anchor=matrix(objects['board'].quat_xyzw).T@(
      np.asarray(context['body_origins']['part'])-np.asarray(context['body_origins']['board']))
  _removed=False
 else:
  _wet_contacts|=seated_solder_contacts(objects,context)&(_temperatures>=217.)
 event={'time_s':context['time_s'],'thermal_release':action=='release',
        'solidified':action=='solidify','minimum_lead_C':float(np.min(_temperatures)),
        'maximum_lead_C':float(np.max(_temperatures))}
 _native_events.append(event)
 print('NATIVE_MATERIAL_BOND '+json.dumps(event),flush=True)

def observe(objects,context,dt):
 global _temperatures,_case_temperatures,_overheat,_damaged,_removed,_gun_home
 global _wet_contacts
 if _gun_home is None:_gun_home=np.asarray(context['body_origins']['hot_air']).copy()
 board=np.asarray(context['body_origins']['board']);part=np.asarray(context['body_origins']['part'])
 if np.linalg.norm(board-np.asarray(BOARD.pos))>STATIONARY_REWORK_BOARD_TOLERANCE:_damaged=True
 if not _bond_released:
  # An actual cold fracture or wrenching displacement is permanent pad damage.
  if np.linalg.norm(matrix(objects['board'].quat_xyzw).T@(part-board)-_cold_anchor)>.002:_damaged=True
 elif np.linalg.norm((part-board)-(np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)))>.015:
  # Any genuine extraction direction is allowed after reflow. The instruction
  # does not require a vertical lift before transferring the inductor to its tray.
  _removed=True
 nozzle=point(objects,context,'hot_air',NOZZLE_LOCAL)
 direction=matrix(objects['hot_air'].quat_xyzw)@DIR
 # The reference part has underside terminals. Air heats the visible case,
 # then conducts to solder; it does not heat invented exposed metal caps.
 # Once removed, case heating cannot keep warming the original board pads.
 rate=case_heating_fractions(nozzle,direction,part,matrix(objects['part'].quat_xyzw))
 warm=rate/HEATING_TAU_S;cool=(1.-rate)/COOLING_TAU_S
 total=warm+cool;equilibrium=(350.*warm+25.*cool)/total
 old_case=_case_temperatures.copy()
 _case_temperatures=equilibrium+(_case_temperatures-equilibrium)*np.exp(-dt*total)
 at_site=np.linalg.norm((part-board)-(np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)))<.002
 if at_site:
  target=(old_case+_case_temperatures)/2
  tau=CASE_TO_PAD_TAU_S
 else:
  # Bare board pads can still be overheated after extraction. Heat on the
  # removed case itself never conducts back into those original pads.
  pad_local=LEADS+np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)
  pads=board+(matrix(objects['board'].quat_xyzw)@pad_local.T).T
  delta=pads-nozzle;axial=delta@direction
  radial=np.linalg.norm(delta-axial[:,None]*direction,axis=1)
  R=matrix(objects['part'].quat_xyzw)
  local_nozzle=R.T@(nozzle-part);local_pads=(R.T@(pads-part).T).T
  visible=np.array([not segment_intersects_core(local_nozzle,p) for p in local_pads])
  heated=(axial>=.010)&(axial<=.045)&(radial<=.0015+axial*math.tan(math.radians(20)))&visible&(direction[2]<-.5)
  target=np.where(heated,350.,25.);tau=np.where(heated,HEATING_TAU_S,COOLING_TAU_S)
 _temperatures=target+(_temperatures-target)*np.exp(-dt/tau)
 contacts=seated_solder_contacts(objects,context)
 _wet_contacts=contacts&(_wet_contacts|(_temperatures>=217.))
 _overheat=np.where(_temperatures>320.,_overheat+dt,0.)
 if np.any(_overheat>=1.):_damaged=True

def clear(objects,tcps):
 for name in ('part','hot_air'):
  for tcp in tcps.values():
   gap=np.maximum(np.maximum(objects[name].low-tcp,tcp-objects[name].high),0.)
   if np.linalg.norm(gap)<=.10:return False
 return True

def success(objects):
 if _damaged or not (_bond_released and _removed):return False
 part=objects['part'];c=LIVE_CONTEXT
 contained=np.all(part.low[:2]>=TRAY-.05-1e-5) and np.all(part.high[:2]<=TRAY+.05+1e-5)
 support=sum(float(np.asarray(c.get(channel,{}).get(rest.name,np.zeros(3)))[2])
             for rest in RESTS for channel in ('contact_forces','contact_friction_forces'))
 return bool(contained and part.low[2]>=TOP+.01-1e-5 and part.high[2]<TOP+.05
     and np.linalg.norm(np.asarray(c['body_origins']['hot_air'])-np.asarray(GUN.pos))<.018
     and support>=.9*GUN.mass*9.81
     and part.resting() and objects['hot_air'].resting() and clear(objects,c['tcp']))

def score(objects):
 if _damaged or any(objects[n].low[2]<TOP-.03 for n in ('part','hot_air')):return 0.
 if success(objects):return 1.
 away=np.linalg.norm((np.asarray(LIVE_CONTEXT['body_origins']['part'])-np.asarray(LIVE_CONTEXT['body_origins']['board']))-(np.asarray(COMPONENT.pos)-np.asarray(BOARD.pos)))>.015
 return .5 if _removed and away else 0.

def ready_to_release():return bool(not _damaged and np.all(_temperatures>=217.))

def update_materials():
 global _joint,_subscription,_damaged,_native_break_seen,_bond_released,_pad_shaders
 import omni.usd,omni.physx
 from pxr import UsdPhysics,PhysicsSchemaTools,UsdShade,Gf
 from omni.physx.bindings._physx import SimulationEvent
 if _joint is None:
  matches=[p for p in omni.usd.get_context().get_stage().Traverse() if p.GetName()=='part_joint' and p.IsA(UsdPhysics.FixedJoint)]
  if len(matches)!=1:raise RuntimeError('component material bond did not resolve uniquely')
  _joint=UsdPhysics.Joint(matches[0]);path=str(matches[0].GetPath())
  board_paths=[p.GetPath() for p in omni.usd.get_context().get_stage().Traverse()
               if p.GetName()=='board' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
  if len(board_paths)!=1:raise RuntimeError('PCB material path did not resolve uniquely')
  _pad_shaders=[UsdShade.Shader.Get(omni.usd.get_context().get_stage(),str(board_paths[0])+f'/geometry/solid_{i}_material/shader') for i in range(1,len(LEADS)+1)]
  def callback(event):
   global _damaged,_native_break_seen
   if event.type!=int(SimulationEvent.JOINT_BREAK):return
   if str(PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath']))!=path:return
   _native_break_seen=True
   if not _bond_released:_damaged=True
   _native_events.append({'time_s':LIVE_CONTEXT['time_s'],'cold_fracture':not _bond_released})
   print('NATIVE_MATERIAL_BOND '+json.dumps(_native_events[-1]),flush=True)
  _subscription=omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(callback)
 action=material_bond_action(LIVE_OBJECTS,LIVE_CONTEXT)
 if action=='release':
  _joint.GetCollisionEnabledAttr().Set(True)
  _joint.GetJointEnabledAttr().Set(False)
  if _joint.GetJointEnabledAttr().Get() is not False:raise RuntimeError('thermal material-bond release failed')
  record_material_transition(action,LIVE_OBJECTS,LIVE_CONTEXT)
 elif action=='solidify':
  # Re-anchor at the measured contact pose. Enabling an old anchor would
  # snap a legitimately shifted molten component back to its spawn pose.
  R0=matrix(LIVE_OBJECTS['board'].quat_xyzw);R1=matrix(LIVE_OBJECTS['part'].quat_xyzw)
  delta=R0.T@(np.asarray(LIVE_CONTEXT['body_origins']['part'])-np.asarray(LIVE_CONTEXT['body_origins']['board']))
  q=ik.matrix_to_quat_xyzw(R0.T@R1)
  _joint.GetLocalPos0Attr().Set(Gf.Vec3f(*map(float,delta)))
  _joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
  _joint.GetLocalRot0Attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
  _joint.GetLocalRot1Attr().Set(Gf.Quatf(1))
  _joint.GetJointEnabledAttr().Set(True)
  if _joint.GetJointEnabledAttr().Get() is not True:raise RuntimeError('solder solidification failed')
  record_material_transition(action,LIVE_OBJECTS,LIVE_CONTEXT)
 for shader in _pad_shaders:
  # Pad damage remains visible after cooling and after the component leaves.
  color=Gf.Vec3f(*((.65,.08,.04) if _damaged else (.65,.67,.69)))
  if shader.GetInput('diffuseColor').Get()!=color:shader.GetInput('diffuseColor').Set(color)

def capture_gun_grasp(label):
 global _gun_grasp,_gun_start_R,_gun_source_body
 if _gun_grasp is not None or label!='lift gun' or not LIVE_CONTEXT:return
 if LIVE_CONTEXT['gripper']['right']>.8:return
 R=matrix(LIVE_OBJECTS['hot_air'].quat_xyzw)
 body=np.asarray(LIVE_CONTEXT['body_origins']['hot_air'])
 _gun_grasp=(R.T@(LIVE_CONTEXT['tcp']['right']-body),R.T@LIVE_CONTEXT['tcp_rotation']['right'])
 _gun_start_R=R.copy()
 _gun_source_body=body.copy()

def gun_goal_rotation(label,t):
 if label=='lift gun':return ik.slerp_matrix(_gun_start_R,np.eye(3),ik.smoothstep(np.clip((t-5.)/1.5,0,1)))
 return np.eye(3)

def refresh_gun_grasp(label):
 """Servo the free tool through its measured contact grasp, never its pose.

 V3's fixed transform missed a few degrees of grip rotation, amplified by the
 long nozzle. Update the observed TCP/body relation once per observation while
 the closed gripper remains near the handle. This only changes robot targets.
 """
 global _gun_grasp,_gun_grasp_update_time
 if _gun_grasp is None or label not in {'lift gun','over board','heat leads','extract inductor','return gun','lower gun','release gun'}:return
 c=LIVE_CONTEXT
 if c.get('gripper',{}).get('right',1.)>.8 or _gun_grasp_update_time==c['time_s']:return
 body=np.asarray(c['body_origins']['hot_air']);tcp=np.asarray(c['tcp']['right'])
 if np.linalg.norm(tcp-body)>.05:return
 R=matrix(LIVE_OBJECTS['hot_air'].quat_xyzw)
 _gun_grasp=(R.T@(tcp-body),R.T@c['tcp_rotation']['right'])
 _gun_grasp_update_time=c['time_s']

def demo_rotation(arm,label,t,start):
 if _demo_reflow_time is not None:
  elapsed=t-_demo_reflow_time
  if arm=='left' and _demo_left_grasp is not None and elapsed>=3.8:
   if elapsed<10.3:return _demo_left_grasp[1]
   return ik.slerp_matrix(_demo_left_grasp[1],LEFT_R,ik.smoothstep(np.clip((elapsed-10.3)/1.5,0,1)))
 if arm=='right':
  capture_gun_grasp(label)
  refresh_gun_grasp(label)
  if _gun_grasp is not None:
   if _demo_reflow_time is not None:
    elapsed=t-_demo_reflow_time
    if elapsed>=10.8:
     return ik.slerp_matrix(_gun_start_R@_gun_grasp[1],RIGHT_R,ik.smoothstep(np.clip((elapsed-10.8)/1.5,0,1)))
    R=ik.slerp_matrix(np.eye(3),_gun_start_R,ik.smoothstep(np.clip((elapsed-6.5)/2.,0,1)))
    return R@_gun_grasp[1]
   if label=='clear gun':
    clear_start=HEAT_START_S+HEAT_DWELL_S+HOT_GRASP_HOLD_S+2.+2.+1.5+.8
    return ik.slerp_matrix(_gun_grasp[1],RIGHT_R,ik.smoothstep(np.clip((t-clear_start)/1.5,0,1)))
   if label!='done':return gun_goal_rotation(label,t)@_gun_grasp[1]
 return ik.slerp_matrix(start,LEFT_R if arm=='left' else RIGHT_R,ik.smoothstep(min(t/2.,1.)))

def demo_target(arm,label,t,pos,rotation,finger):
 global _lift_origin,_demo_reflow_time,_demo_left_grasp,_demo_lift_tcp
 global _nozzle_integral,_nozzle_correction,_nozzle_feedback_time
 pos=np.asarray(pos).copy();c=LIVE_CONTEXT;o=LIVE_OBJECTS
 standby=np.array((.23,.20,TOP+.20))
 if _bond_released and _demo_reflow_time is None:_demo_reflow_time=t
 if _demo_reflow_time is not None:
  elapsed=t-_demo_reflow_time
  def blend(a,b,start,duration):
   return np.asarray(a)+(np.asarray(b)-np.asarray(a))*ik.smoothstep(np.clip((elapsed-start)/duration,0,1))
  if arm=='left':
   part=np.asarray(c['body_origins']['part']);grip=part+COMPONENT_TCP_OFFSET
   hover=grip+np.array((0,0,.06))
   if elapsed<1.5:pos=blend(standby,hover,0.,1.5);finger=OPEN
   elif elapsed<2.5:pos=blend(hover,grip,1.5,1.);finger=OPEN
   elif elapsed<4.:
    pos=grip;finger=CLOSED if _bond_released else OPEN
    if elapsed>=3.8 and _demo_left_grasp is None:
     R=matrix(o['part'].quat_xyzw)
     _demo_left_grasp=(R.T@(np.asarray(c['tcp']['left'])-part),R.T@c['tcp_rotation']['left'])
   else:
    if _demo_lift_tcp is None:_demo_lift_tcp=np.asarray(c['tcp']['left']).copy()
    offset=COMPONENT_TCP_OFFSET if _demo_left_grasp is None else _demo_left_grasp[0]
    lifted=_demo_lift_tcp+np.array((0,0,.04 if _bond_released else 0.))
    tray=np.array((*TRAY,TOP+.010+.0051+.002))+offset
    trayhover=tray+np.array((0,0,.06))
    if elapsed<6.:pos=blend(_demo_lift_tcp,lifted,4.,2.);finger=CLOSED if _bond_released else OPEN
    elif elapsed<8.:pos=blend(lifted,trayhover,6.,2.);finger=CLOSED
    elif elapsed<9.5:pos=blend(trayhover,tray,8.,1.5);finger=CLOSED
    elif elapsed<10.3:pos=tray;finger=OPEN
    else:pos=blend(tray,standby,10.3,1.5);finger=OPEN
   return pos,rotation,finger
  if _gun_grasp is not None:
   nozzle=np.asarray(COMPONENT.pos)+np.array((0,0,LEADS[0,2]))-.035*DIR
   nozzle+=np.asarray(c['body_origins']['board'])-np.asarray(BOARD.pos)
   work_body=nozzle-NOZZLE_LOCAL+_nozzle_correction
   retreat=work_body+np.array((.06,-.01,0.))
   home=_gun_home if _gun_home is not None else np.asarray(GUN.pos)
   hover=home+np.array((0,0,.03))
   R=ik.slerp_matrix(np.eye(3),_gun_start_R,ik.smoothstep(np.clip((elapsed-6.5)/2.,0,1)))
   finger=CLOSED
   if elapsed<1.:body=blend(work_body,retreat,0.,1.)
   elif elapsed<6.5:body=retreat
   elif elapsed<8.5:body=blend(retreat,hover,6.5,2.)
   elif elapsed<10.:body=blend(hover,home,8.5,1.5)
   elif elapsed<10.8:body=home;finger=GUN_OPEN
   else:
    end=np.array((.20,-.35,TOP+.18))
    return blend(home+_gun_start_R@_gun_grasp[0],end,10.8,1.5),rotation,GUN_OPEN
   return body+R@_gun_grasp[0],rotation,finger
 if arm=='right':
  if label in {'to gun','approach gun','grip gun','secure gun'}:
   pos+=np.asarray(c['body_origins']['hot_air'])-np.asarray(GUN.pos)
  if _gun_grasp is not None and label in {'lift gun','over board','heat leads','extract inductor','return gun','lower gun','release gun'}:
   body_goal=pos-GUN_GRASP_OFFSET
   R=gun_goal_rotation(label,t)
   home_shift=(_gun_home-np.asarray(GUN.pos)) if _gun_home is not None else np.zeros(3)
   if label=='lift gun':
    blend=ik.smoothstep(np.clip((t-5.)/1.5,0,1))
    body_goal+=(1.-blend)*(_gun_source_body-np.asarray(GUN.pos))+blend*home_shift
   elif label=='return gun':
    start=HEAT_START_S+HEAT_DWELL_S+HOT_GRASP_HOLD_S+2.
    body_goal+=home_shift*ik.smoothstep(np.clip((t-start)/2.,0,1))
   elif label in {'lower gun','release gun'}:body_goal+=home_shift
   if label in {'over board','heat leads','extract inductor'}:
    body_goal+=np.asarray(c['body_origins']['board'])-np.asarray(BOARD.pos)
    # Rotate about the desired nozzle rather than sweeping its 205 mm lever
    # around the handle. Only native robot joint targets receive this command.
    nozzle_goal=body_goal+NOZZLE_LOCAL
    if (label=='heat leads' and c.get('gripper',{}).get('right',1.)<.8
        and 'hot_air' in c['body_origins'] and c['time_s']!=_nozzle_feedback_time):
     # Counter ordinary motor/compliance tracking error with bounded robot
     # target bias. The observed nozzle and all free bodies remain untouched.
     actual=point(o,c,'hot_air',NOZZLE_LOCAL)
     error=nozzle_goal-actual
     dt=.05 if _nozzle_feedback_time is None else min(.1,max(0.,c['time_s']-_nozzle_feedback_time))
     _nozzle_integral+=error*dt
     _nozzle_integral*=min(1.,.020/max(1e-12,np.linalg.norm(_nozzle_integral)))
     _nozzle_correction=.35*error+_nozzle_integral
     _nozzle_correction*=min(1.,.020/max(1e-12,np.linalg.norm(_nozzle_correction)))
     _nozzle_feedback_time=c['time_s']
    if label=='heat leads':nozzle_goal+=_nozzle_correction
    body_goal=nozzle_goal-R@NOZZLE_LOCAL
   pos=body_goal+R@_gun_grasp[0]
  elif _gun_grasp is not None and label=='clear gun':
   clear_start=HEAT_START_S+HEAT_DWELL_S+HOT_GRASP_HOLD_S+2.+2.+1.5+.8
   blend=1.-ik.smoothstep(np.clip((t-clear_start)/1.5,0,1))
   pos+=blend*(_gun_grasp[0]-GUN_GRASP_OFFSET)
 else:
  # A clear standby pose prevents an open jaw/palm from obstructing the air
  # or striking the handpiece while its tip is trying to track the board.
  initial,_=ik.tcp_pose(np.asarray(READY),base_pos=station.BASES['left'])
  pos=initial+(standby-initial)*ik.smoothstep(np.clip(t/2.,0,1));finger=OPEN
 return pos,rotation,finger

def demo():
 gun=np.asarray(GUN.pos)+GUN_GRASP_OFFSET;gunhover=gun+np.array((0,0,.03))
 target=np.asarray(COMPONENT.pos)+np.array((0.,0.,LEADS[0,2]))-.035*DIR
 work=target-(NOZZLE_LOCAL-GUN_GRASP_OFFSET)
 lclear=np.array((.23,.20,TOP+.20));rclear=np.array((.20,-.35,TOP+.18))
 return {'left':[Waypoint(2.,tuple(lclear),OPEN,'standby'),
  Waypoint(HEAT_START_S+HEAT_DWELL_S+HOT_GRASP_HOLD_S+10.8,tuple(lclear),OPEN,'wait for native release')],
 'right':[Waypoint(2.,tuple(gunhover),GUN_OPEN,'to gun'),Waypoint(1.5,tuple(gun),GUN_OPEN,'approach gun'),
  Waypoint(.8,tuple(gun),CLOSED,'grip gun'),Waypoint(.7,tuple(gun),CLOSED,'secure gun'),
  Waypoint(1.5,tuple(gunhover),CLOSED,'lift gun'),Waypoint(1.5,tuple(work),CLOSED,'over board'),
  Waypoint(HEAT_DWELL_S+HOT_GRASP_HOLD_S,tuple(work),CLOSED,'heat leads'),Waypoint(2.,tuple(work),CLOSED,'extract inductor'),
  Waypoint(2.,tuple(gunhover),CLOSED,'return gun'),Waypoint(1.5,tuple(gun),CLOSED,'lower gun'),
  Waypoint(.8,tuple(gun),GUN_OPEN,'release gun'),Waypoint(1.5,tuple(rclear),GUN_OPEN,'clear gun'),Waypoint(5.,tuple(rclear),GUN_OPEN,'done')]}
