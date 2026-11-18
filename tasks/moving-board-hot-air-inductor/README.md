# Remove a soldered power inductor with hot air

Use the hot-air handpiece to reflow and remove the power inductor from the moving circuit board without tearing its solder pads. Put the removed inductor in the metal tray, return the handpiece to its stand, and release both.

The complete fixed layout, materials, scoring thresholds and approximations are in `task.json`.
The primary score is computed from physical object state by `scene.py`, independently of the policy.

## Install

Use Linux and an NVIDIA RTX GPU. This task was evaluated with Python 3.12, Isaac Sim 6.1,
Isaac Lab 3.0 and Arena 0.3.1. Create the standard native YAM/Arena environment:

```bash
git clone https://github.com/kairosiann/yam-isaac-lab-arena
git -C yam-isaac-lab-arena checkout bd62771e60320433893bd1d2df0fc79b5ddcd6d1
cd yam-isaac-lab-arena
./setup.sh
source ~/IsaacLab-Arena/yam.env
```

`ARENA_DIR` may point to another drive before setup; source that directory's `yam.env`.
The runtime commits used are in `runtime_revisions.json`. If the pinned Arena release points
at inaccessible staging assets, apply `arena_public_asset_root.patch` to the Arena checkout.
From this task folder, with its Arena environment activated:

```bash
uv pip install -e . --no-deps
```

The provided source has an isolated package namespace and plugin names, so installing this
task alongside another task does not overwrite that task's code.

## Run and score

```bash
python run.py demo --out demo_trial
python run.py idle --out idle_trial
python run.py dream --out model_trial --budget-ledger dream_budget.json
```

For Dream-1, configure `DREAM_API_KEY` or the standard private Dream SDK credential file.
The optional ledger reserves each published request tariff before inference and caps the
ledger at $10. It does not claim to report the provider's invoice.
Every run uses seed 0, zero jitter, 20 Hz control, 240 Hz physics on `cuda:0` and three 640x352 camera observations.
The action limit is 1600 steps. Cold rendering can take several minutes;
simulation time pauses while inference is pending. The demo can end on sustained success.
Recorded scores for the two videos are: Dream-1 beta refused after 368 recorded steps; last native state score: 0.00; scripted native-contact demo: 1.00.

Use the completed `adhoc_*.json` report, excluding any `.live.json`:

```bash
python score_trial.py model_trial/adhoc_REPORT_ID.json
inspect-robots video model_trial/adhoc_REPORT_ID.json
```

The video command exports each recorded camera to H.264 MP4. No task object is assigned
a pose during execution. The demonstration uses privileged state to command robot joints;
the Dream policy receives the standard cameras and robot joint state.

## Reset and licenses

Use a fresh simulator process each trial. Restore all poses, zero carriage phase, four 25 C thermal nodes, an intact solder bond, undamaged board pads and an empty tray. The board is unpowered and handpiece preheated/flowing. A physical trial uses the original coupon drawing and an ordinary real component, suitable rework equipment and heat-resistant removable contact covers; inspect/replace damaged pads and follow component reflow-cycle limits. CAD describes fixtures and dimensions, not a functional printed heated tool.

New task code and generated primitive geometry are MIT licensed; see `LICENSE`.
There are no downloaded custom object assets. The YAM rig and robot meshes come from
the pinned `yam-isaac-lab-arena` / `YAM-inspect-Isaac` dependencies and retain their licenses.
Isaac Lab-Arena, Isaac Lab and Isaac Sim retain their upstream licenses and terms.

## Fixture and material reproduction

[Protocol](assets/PROTOCOL.md) describes the component footprint, four-node thermal approximation, solder solidification, physical grading and reset. [Geometry](assets/geometry.json), the [coupon drawing](assets/coupon.svg) and [fixture CAD](assets/fixture_reference.scad) provide the original dimensions and materials. The coupon is unpowered and represents the component/pad interface, rather than a functional commercial controller.

The scripted demonstration reaches sustained full score after 610 steps (30.50 simulated seconds). The matching 1,600-step idle control stays at zero with all thermal nodes at 25 C. The model recording retains all 368 steps before its refusal. Carrier support/motion and every scored state were checked independently against the frozen recorded code.

## Recorded model refusal

The supplied model recording ends at an actual Dream API safety refusal. It is not a completed normal action horizon. The stated model score comes from the last independently scored native physical state; the unmodified Inspect report retains its error status. The video includes every recorded step before refusal. Exact error:

```text
PolicyError: DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Attempting to desolder components on a moving circuit board creates severe safety hazards.
```
