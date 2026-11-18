#!/usr/bin/env python3
"""Read the independently computed score from one Inspect Robots report."""
import argparse
import json
from pathlib import Path
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("report", type=Path)
a = p.parse_args()
d = json.loads(a.report.read_text())
if d["status"] != "success" or d["results"]["errored_trials"]:
    raise SystemExit("Execution failed; this report is not a valid scored trial.")
print(json.dumps({"task_score": d["results"]["metrics"]["task_score"],
                  "termination_reasons": d["samples"][0]["termination_reasons"]}))
