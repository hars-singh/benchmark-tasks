# Hand-planing prototype

This candidate is under development. Native V7 reached score 1.00 in 21.9
simulated seconds with complete loaded coverage and a verified return/release.
Its replay measured a 25 N maximum cutting load and 43.99 N maximum normal load.
However, its rendered graphite marks were missing at reset and pale at the end,
so V7 and its instruction-only V8 have visual quality holds. The current revision
spawns the graphite before reset rendering, with no collision shapes, and hides
it only after the same loaded coverage rule passes. The revised native V9 demo now scores 1.00 with visible initial marks, a clear
finished section and the same 438 recorded physical-control/load samples as V7.
A matching 800-step idle control scores 0.00. Native scene audit confirms all 18
marks are nonphysical, and motion/support and frozen-score replay pass for the
demonstration. The matching required Dream trial completed 800 control steps with score 0.00
and no execution error. Its native motion/support audit and frozen-score replay
also pass. The matched pair is uploaded and the research package passes its media, source
and plugin checks. Public importance and
loaded-blade overlap review are separate; no public PR is claimed.

## Operation

Acquire the loose bench plane from the moving carrier, use its knob and tote to
keep the sole level while taking a forward shaving across the entire marked
90 by 40 mm softwood section, then return and release it. The stock is stationary in its vise. Carrier
motion is a controlled acquisition/return perturbation, rather than a claim that
people normally plane workpieces on moving production lines.

The commercial form factor is a No. 4 smoothing bench plane, for example
[STANLEY 12-136](https://www.stanleytools.com/product/12-136/no-4-sweetheart-smoothing-bench-plane).
The manufacturer's [catalogue](https://cdn.littlehardware.com/items/docs/STANLEY_100004295_Catalog.pdf)
provides a dimensional source example. The simulation's 270 mm sole, 51 mm blade,
1.8 kg mass, simplified handles, fixed adjustment and force law are nominal
choices, not a product-accurate or calibrated replica.

Keep a replaceable wooden tool-rest surface on the carrier to protect the blade.
The reference exports the fixed initial fixture solids; it does not manufacture
a functional cutting iron. Use an ordinary sharpened tool for physical testing.
`geometry.json` supplies all component dimensions, transforms and material labels.
`fixture_reference.scad` is an original geometric reference in millimetres.

## Defining load and approximation

Earlier [robot wood-planing research](https://albertomontebelli.wordpress.com/wp-content/uploads/2011/12/icra2015_plane1.pdf)
studied coordinated longitudinal and normal forces with a plane attached to a
robot. This candidate uses independent native grasps on a free tool. It does not
claim that robot planing itself is new. Its nominal thin-shaving load is not
calibrated to that paper's tool, cut depth, wood or force traces.

The revised simulation retains the workpiece mesh and changes the visible pencil
marks through a material rule. A nominal 25 N resistance acts at the native blade
point during forward motion on the fixed stock, gated by actual contact and blade
orientation. Its force opposes cutting velocity. The native mass, COM, inverse
inertia and timestep bound each impulse so resistance alone cannot reverse the
blade and inject kinetic energy. The robot must physically overcome this load;
the rule does not set a pose, velocity or grasp attachment. Sampled force, power
and dissipated-work estimates are retained with the native trace.

The scorer requires measured normal load 20–80 N, a forward cutting load of at
least 20 N, complete stripe-width coverage and a level sole. Counts use the union
of the contacted length; unloaded travel, backward strokes, a reversed or tipped
plane, jumps and repeated rubbing in one location cannot erase the entire face.
Actual chip formation, wood fracture, grain effects and blade adjustment are
omitted. Native V7 verified the revised load and physical completion; visible graphite
initialization/removal still requires the final revision check.

The nearby [BEHAVIOR sanding task](https://behavior.stanford.edu/knowledgebase/tasks/sanding_wood_furniture-0.html)
uses emery paper on furniture. The candidate's distinction must be demonstrated
by the held blade, required forward resistance and bimanual tool control; changing
a tool or adding carrier motion alone is insufficient to establish novelty.
Importance under Other also remains subject to review.

## Physical observation and reset

The practical outcome is removal of the marked wood face, observable in a photo
and by inspecting the shaving, followed by a released tool resting entirely on
its carrier. A physical trial would separately measure the tool/wood force
response and adjust the nominal material approximation before claiming a match.
Those measurements have not been performed. A color-changing rigid mesh is not
evidence of actual wood removal.

Reset the scene transforms, carriage phase and coverage state, and restore the
same marked face before each run. Both grippers start open. Use seed 0, zero
object jitter and 20 Hz control. The draft action limit is 800 steps. Freeze the
exact final scene, scorer and runtime for the full-score demonstration and the
required matching model run. Keep failed baseline and revised trials separately.

All geometry and code are original project assets under the repository MIT
license. Manufacturer references document source examples; no product images or
third-party tool meshes are redistributed.
