# Measure and record the radial runout of a conveyor roller

Turn the crank slowly through at least one full turn to measure the roller. Read the runout on the gauge. Press the green button if the reading is 0.40 millimetres or less, or the orange button if it is higher. Let go and wait for the roller to stop.

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
The action limit is 1200 steps. Cold rendering can take several minutes;
simulation time pauses while inference is pending. The demo can end on sustained success.
Recorded scores for the two videos are: Dream-1 beta run: 0.00; scripted native-contact demo: 1.00.

Use the completed `adhoc_*.json` report, excluding any `.live.json`:

```bash
python score_trial.py model_trial/adhoc_REPORT_ID.json
inspect-robots video model_trial/adhoc_REPORT_ID.json
```

The video command exports each recorded camera to H.264 MP4. No task object is assigned
a pose during execution. The demonstration uses privileged state to command robot joints;
the Dream policy receives the standard cameras and robot joint state.

## Reset and licenses

Reload the station at seed 0, restore all parts and the crank with its grip hanging down, leave both buttons up, clear the gauge display, and reset the recorder.

New task code and generated primitive geometry are MIT licensed; see `LICENSE`.
There are no downloaded custom object assets. The YAM rig and robot meshes come from
the pinned `yam-isaac-lab-arena` / `YAM-inspect-Isaac` dependencies and retain their licenses.
Isaac Lab-Arena, Isaac Lab and Isaac Sim retain their upstream licenses and terms.
