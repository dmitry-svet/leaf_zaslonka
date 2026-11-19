# leaf_zaslonka

Parametric 3D-printable parts for a parking-heater air duct on a Nissan Leaf:
plate with an oval-to-round 45 degree tube, body-side frame, sliding shutter,
clamp bar. One CadQuery script, `zaslonka.py`. User-facing docs: `README.md`.

## Skill

Use the `parametric-3d-printing` skill (`~/.claude/skills/parametric-3d-printing`)
for every model change. It provides the build wrapper and `mating.py` (copied
into this repo).

## Build

Build with previews and checks (bed is 256 x 256):

```bash
.venv/bin/python ~/.claude/skills/parametric-3d-printing/run_cadquery_model.py zaslonka.py --preview --strict --bed 256x256x256
```

Plain build, no previews: `.venv/bin/python zaslonka.py`.

- The wrapper also renders `print_*_preview.png`; delete them, they are not kept.
- A plain build rewrites the `.step` files (timestamp only). Do not commit that
  when nothing else changed.

## Outputs

Per part (`zaslonka`, `zaslonka_frame`, `zaslonka_shutter`, `zaslonka_clamp`):

- `<name>.stl`, `<name>.step` - nominal size.
- `print_<name>.stl` - scaled by `1/(1 - shrinkage)` (2%), for the slicer.
- `<name>_preview.png` - render of the nominal part.

All of them are committed.

## Model conventions

- All dimensions live in the `PARAMETERS` block. Derived values go in `MODEL`.
- `result` is the full assembly (plate + body-side rib). It is split by a slab
  at `rib_out_h` into `frame` (below) and `body` (above, moved down to z = 0).
  Body-side features must be added to `result` before the split.
- The plate outline is `outline_wire`; ribs, rails and the frame ear fill are
  clipped to it. Change the outline there, not per feature.
- Tube wall grows inward: the outer tube size must not change.
- Rails sit flush with the plate edge: `win_overlap + fit_clearance + rail_w`
  must equal `edge_margin`. Change them together.
- `check_mating` must pass: the rails hold the shutter in Y and Z with
  `fit_clearance` per side.
- After a change: build, look at the preview, check every STL is watertight.

## Lessons

- Scaling the B-rep before tessellation gives a non-watertight STL. Scale the
  mesh instead.
- After scaling, float32 rounding can turn a sliver triangle collinear and open
  the mesh. The export collapses the shortest edge of truly degenerate faces
  only (`~mesh.nondegenerate_faces()`); an area threshold breaks the mesh.
- The `Exported: ...` line uses the OCC bounding box, which is about 1 mm too
  tall for the lofted tube. Trust the mesh extents.

## Git

- Commit and push to `origin` (`git@github.com:dmitry-svet/leaf_zaslonka.git`),
  branch `master`, no ticket prefix: personal repo.
- `_task/` holds voice notes; keep it untracked.
- Update `README.md` in the same commit when parameters or outputs change.
