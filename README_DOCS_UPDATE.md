# robotics_lab documentation update package

This package contains current source-of-truth documentation for the simulator-first Cartesian acceptance phase.

Copy from this package into the repo root:

```text
AGENTS.md
REVIEW.md
README.md                 # optional but recommended
docs/
```

If you want the minimal update, copy only:

```text
AGENTS.md
REVIEW.md
docs/
```

Recommended command from the repository root after unpacking this package elsewhere:

```bash
cp AGENTS.md /path/to/robotics_lab/AGENTS.md
cp REVIEW.md /path/to/robotics_lab/REVIEW.md
cp README.md /path/to/robotics_lab/README.md
rsync -a --delete docs/ /path/to/robotics_lab/docs/
```

Then inspect the diff and run:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
./scripts/codex_gate.sh HARDEN-10
```

`README.md` is included because the root README currently still benefits from clearer current-phase wording. If you have local README edits, merge manually rather than overwriting blindly.
