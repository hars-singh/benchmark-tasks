# Feed a pencil into a moving sharpener while turning its crank

Sharpen the yellow pencil in the moving crank sharpener. Feed the pencil into the front opening while turning the rear crank clockwise, then withdraw the pencil and return it to its two rests. Let go and move both hands clear.

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
Every run uses seed 0, zero jitter, 20 Hz control, 240 Hz physics and three 320x176 cameras.
The action limit is 1600 steps. Cold rendering can take several minutes;
simulation time pauses while inference is pending. The demo can end on sustained success.
Scores submitted for the two recordings are: Dream-1 beta run: 0.00; scripted native-contact demo: 1.00.

Use the completed `adhoc_*.json` report, excluding any `.live.json`:

```bash
python score_trial.py model_trial/adhoc_REPORT_ID.json
inspect-robots video model_trial/adhoc_REPORT_ID.json
```

The video command exports each recorded camera to H.264 MP4. No task object is assigned
a pose during execution. The demonstration uses privileged state to command robot joints;
the Dream policy receives the standard cameras and robot joint state.

## Reset and licenses

Return the carriage to its centre, reset the crank with the knob below the axle, unload the cutter spring and place a fresh blunt 140 mm pencil on the two rests. Restart the motion clock at phase zero and both arms in READY. Real trials consume one pencil and require removal of shavings; use an ordinary enclosed manual-feed sharpener on the moving plate. The supplied CAD describes the fixture and simulated geometry, not a functional printed cutter. A commercial tool source and the limits of product matching are documented in assets/PROTOCOL.md.

New task code and generated primitive geometry are MIT licensed; see `LICENSE`.
There are no downloaded custom object assets. The YAM rig and robot meshes come from
the pinned `yam-isaac-lab-arena` / `YAM-inspect-Isaac` dependencies and retain their licenses.
Isaac Lab-Arena, Isaac Lab and Isaac Sim retain their upstream licenses and terms.

## Scoring regression checks

With the same activated environment, run `python -m unittest discover -s tests -q`. The nine checks cover absent load, absent contact, misalignment, empty cranking, reversible-angle motion, reset, either pencil end and separate progress for opposite ends. The physical fixture dimensions, material-response approximation and real-trial qualifications are in [the protocol](assets/PROTOCOL.md).

## Detailed simulation fidelity

All acquisition, insertion, axial pressure, crank rotation, withdrawal and placement are native rigid-body contacts and passive constraints. Only the 5 kg stage receives bounded external PD actuation, at most 20 N and 0.5 Nm. The pencil, cutter, crank and knob are never assigned poses or velocities during execution. The script changes only robot joint commands. A stated material-removal rule replaces the pencil's visible end after eight actual loaded turns; it leaves the original 7 mm diameter collider and 6 g mass intact, conservatively overestimating contact at the sharpened tip. Individual cutting teeth, shavings and fracture are omitted. The 1-3 N feed resistance and approximately 0.031 N m crank resistance at a four-second revolution are declared surrogate loads, not measurements of a commercial sharpener. The printed-dimensional housing is an approximation, not a manufacturer-specific asset. Physics runs at 240 Hz, control at 20 Hz, with static/dynamic contact friction 1.2/1.0. Dream-1 sees three standard camera views and robot joints; the demo uses privileged state feedback. Simulation pauses during model inference, so this does not test real-time latency. Actual spawned compound geometry and fixture joint connections are audited before action. Sources, printable fixture reference and qualifications are in assets/PROTOCOL.md.
