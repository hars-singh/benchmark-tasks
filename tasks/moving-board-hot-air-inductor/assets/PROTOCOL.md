# Power inductor rework

Remove a soldered power inductor from an unpowered PCB coupon using a free
hot-air handpiece, preserve its board pads, place the removed component in the
tray and return the released handpiece to its stand. A native full-score
demonstration completes in 30.5 simulated seconds. The action limit is 80 seconds.

## Hardware and reference footprint

The original generic model follows the nominal XAL1510 envelope: a 15.2 by
16.2 mm moulded composite body and 10.0 mm overall height. The case itself is
9.8 mm high. Two underside terminals are 3.0 by 12.5 mm, on 10.6 mm centres
with a 7.6 mm inner gap. Their 0.2 mm thickness is a modelling approximation.
The nominal mass is 13 g; the manufacturer's range is 12.4–15.0 g. The published
land pattern is 3.18 by 13.2 mm per pad. There are no invented side caps.
`geometry.json` specifies all transforms, fixed hardware, materials and solids;
`fixture_reference.scad` is the original dimensional reference. `coupon.svg`
shows the original 80 by 60 mm FR-4 coupon with lands, tracks and test points;
it is a reference drawing, not machine-ready manufacturing data.

The unpowered coupon represents the component/PCB interface used in power-board
rework. It is not a functioning commercial controller. The three neighboring
package proxies and copper markings are specified in the scene. The operation
ends after removing the faulty component; installing its replacement and testing
the converter are outside this trial.

Use an ordinary regulated hot-air rework handpiece and suitable heat-resistant
removable contact covers on the stock fingers in a physical counterpart. The
simulation retains the stock finger shapes and nominal contact properties; it
does not simulate heat conducted into the robot or the covers. The referenced
part permits limited solder-reflow exposure, not indefinite heating or unlimited
reuse. Follow its current processing limits and replace damaged components and
coupons. The supplied CAD does not manufacture a functional heated tool.

## Material rule

Air heats visible exterior case surfaces before heat conducts into the hidden
underside solder. Eight by eight sample patches cover each of the six case faces.
A patch counts only when its convex face faces the actual nozzle, it is 10–45 mm
downstream, inside a 20-degree half-angle cone with 1.5 mm exit radius, and the
axis points downward by over 30 degrees. The deposited area feeds the actual
lateral case half, normalized by 74.48 square mm and capped at one. No ray heats
an underside terminal through an opaque solid.

Each of two case nodes has heating coefficient fraction/6 s and ambient cooling
coefficient (1-fraction)/20 s, toward nominal 350 C and 25 C respectively. Two
solder nodes follow their case halves through a nominal 2 s conduction lag while
the component remains within 2 mm of its original board-relative site. Both solder
nodes must reach 217 C before the native material bond is disabled. The rule is
an uncalibrated four-node approximation, not CFD or a validated reflow profile.

A wetted terminal that remains in contact with its board land solidifies again
when its solder node falls below 217 C. Contact clips the actual transformed
terminal underside against the matching land and a nominal 0.3 mm solder-bridge
height, requiring at least 0.01 square mm overlap. Any continuously wetted contact
can restore the cold constraint. The native joint is re-enabled at the measured
relative contact pose, never by moving the component back to its original pose.
Cold fracture after solidification still permanently damages the pads. A genuinely
separated component may cool without rebonding; returning cold separated surfaces
does not solder them together. Reheating both nodes permits another release.

After extraction, heating the removed case cannot heat the original PCB pads.
Those pads can still be heated directly when exposed to the actual air cone;
the actual component core occludes a blocked path. One second above 320 C at
either solder node is the declared nominal board-damage rule, not a calibrated
FR-4 or component limit. Damage remains visible on the pads. Electrical function,
solder flow and heat response of neighboring packages are omitted.

The initial material constraint is between the component and PCB, never a gripper
and component. Contact is enabled at creation. All grasping, lifting and placement
use native bodies and robot joints. Cold fracture at nominal 20 N or 0.08 Nm is
permanent pad damage. The controller can read state for demonstration planning;
it never assigns a free-object pose or velocity. Any feasible heating trajectory
and extraction direction is allowed. Heating from the opposite visible side is
valid; the scripted scan is not a mandatory sequence.

The current scripted demonstrator holds the left hand clear while the right
aims at the case centre. Only actual native bond release starts its pickup path;
the handpiece withdraws laterally before the left hand descends. Robot target
poses use measured contact grasps. This changes demonstration control, not the
material thresholds, native robot motor limits or task scoring requirements.
A bounded Cartesian proportional/integral correction compensates actual nozzle
tracking error through robot target positions only, with a 20 mm norm cap.
Normal contact and tangential friction are recorded separately; their signed
vertical sum supplies the stand-support check. Normal load channels used by
other scenes retain their previous meaning.

## Outcome, reset and reproduction

Score 0.5 requires a thermally released component with intact board pads still
over 15 mm from its original board-relative site. Returning it to that site gives
no endpoint credit. Full score also requires complete tray containment, released
and resting handpiece return, at least 90% of its weight supported by the two
stands through the net sum of actual normal and tangential-friction contact, clearance from both whole objects
and a 1 s hold.
Cold fracture or declared board damage permanently gives zero. A loose object
currently more than 30 mm below the table gives zero; a recovery is allowed if
the board pads remain intact and the final conditions are met.
Exact numerical conditions are in task.json and the independent scene scorer.

In a real trial, inspect the separated component and pad integrity under ordinary
magnification, then check each pad's continuity to its matching test point.
Inspect the tray, stand, released grippers and final rest in video. Material
thresholds approximate the solder response; they are not extra hidden step credit.

Run a fresh simulator process for each trial. Restore all initial poses, carriage
phase zero, four 25 C nodes, an intact bond, undamaged pads and an empty tray. The
board is unpowered and the handpiece is already regulated and flowing. The
submission's README gives the isolated installation, execution and scoring
commands. Native state traces record carrier motion, support, thermal events and
actual contact loads for independent score replay. A matching idle control keeps
all four thermal nodes at 25 C and earns zero. A separate engineering fixture
test verifies actual thermal release followed by native joint solidification;
it does not substitute for the free-handpiece robot demonstration.

## Robot power-electronics relevance

Coilcraft describes power-inductor filters in supplies for robots and motor
controls. Analog Devices lists robotics applications for a motor controller with
a buck converter. Removing a faulty surface-mount component is a real hands-on
step in developing and repairing these boards. This supports the proposed Impact
on AI timelines category; fixture motion or a lab setting alone does not. These
references do not identify the generic coupon as either manufacturer's circuit.
The defining physical operation is heating both hidden solder terminations
through the case before extraction while preserving the pads. Polymer heat
shrink, pressing heat-set inserts, depositing solder paste and mechanically
depaneling boards involve different material transitions or load paths.

Primary references:

- [Coilcraft XAL1510 dimensions, land pattern and processing limits](https://www.coilcraft.com/getmedia/cd1cef27-13f0-4568-8894-f7311475209b/xal1510.pdf)
- [Coilcraft soldering and reworking guidance](https://www.coilcraft.com/en-us/resources/application-notes/soldering-surface-mount-components/)
- [Coilcraft industrial automation and motor-control inductors](https://www.coilcraft.com/en-us/edu/series/applications-of-inductors-in-industrial-automation/)
- [Analog Devices TMC9660 robotics and buck-converter application](https://www.analog.com/en/products/TMC9660.html)
- [TI QFN prototyping and hot-air rework](https://www.ti.com/lit/an/snva738a/snva738a.pdf), a related procedure, not calibration of this inductor
- [HAKKO 851 heating and extraction procedure](https://www.hakko.com/english/support/doc/result.php?mode=download&seq=4520)

All primitive geometry, the coupon drawing and scoring code are original and
MIT-licensed. No vendor drawing, mesh or research figure is redistributed.
