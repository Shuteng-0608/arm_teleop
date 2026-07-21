# 5×5 hole-grid data collection

The MuJoCo peg-in-hole task now supports coverage-balanced X-Z grid sampling.
The default configuration divides `[-60, 60] mm` on each axis into five cells
and uses their centers:

```text
X/Z centers = [-48, -24, 0, 24, 48] mm
```

See [hole_random_5x5_grid.svg](hole_random_5x5_grid.svg) for the coordinate
layout. `R1` is the highest-Z row and `C1` is the lowest-X column.

## Episode policy

- The first grid position is applied during controller initialization.
- `shuffled` traversal visits all 25 cells exactly once per cycle.
- A kept episode advances to the next cell.
- A discarded episode retries the same cell.
- Auto-completed episodes advance only after the operator keeps them.
- After cell 25 is kept, the scheduler starts a newly shuffled cycle.

The default position is the center of each cell. Change
`hole_grid_sample_mode` to `uniform_in_cell` to use one deterministic random
point inside each assigned cell while preserving balanced coverage.

## Reproducibility and resume

`hole_grid_seed` controls both the shuffled traversal and optional in-cell
sampling. HDF5 metadata records the effective seed, cycle, sequence index, row,
column, cell bounds, center offset, actual offset, nominal body position, and
actual body position.

Each episode also records task geometry derived from the compiled MuJoCo model:

- `peg_radius_m` and `peg_length_m` from `cylindrical_peg`;
- `peg_tip_site_initial_pos_world` from `peg_tip_site` at record start;
- `hole_radius_m` and `hole_depth_m` inferred from all `wall_hole_ring_*` geoms;
- `hole_goal_site_initial_pos_world` from `hole_goal_site` at record start;
- `hole_goal_site_nominal_pos_world` before grid/random offset;
- `hole_offset_from_nominal_xyz` for the episode assignment.

For the current XML these dimensions are a `0.011 m` peg radius, `0.090 m` peg
length, `0.014 m` hole radius, and `0.045 m` hole depth. The values are inferred
when the recorder starts, so XML geometry changes are reflected automatically.
The same metadata is written to both `episode.hdf5/episode_metadata` and the
episode `metadata.json` sidecar.

The resume cursor is zero-based:

```yaml
hole_grid_start_cycle: 0
hole_grid_start_index: 0
```

For example, `start_cycle: 1` and `start_index: 7` resumes at the eighth
assignment of the second cycle for the configured seed.

## Coverage report

Scan all retained HDF5 episodes under a collection directory:

```bash
python3 scripts/check_hdf5_episode_quality.py \
  --coverage-root data/hole_random_60mm_hmj \
  --grid-target-per-cell 10
```

The command writes `hole_grid_coverage.txt` and `hole_grid_coverage.json` under
`<coverage-root>/grid_coverage/`. The report includes the 5×5 count matrix,
missing and under-target cells, completed cycles, success events, and alarm
counts.
