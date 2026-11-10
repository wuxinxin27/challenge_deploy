# platform_2 — cam_high calibration

Frozen on 2026-11-07. This package contains **only the fixed high-camera extrinsic calibration** (`cam_high`, serial `339222071893`), using the left arm EEF in eye-to-hand mode. Left and right wrist camera extrinsics have not been calibrated.

## Scope and quality

The 10 samples were selected by the lowest per-sample corner reprojection RMS in the full 14-sample fit from `session_20261107_220539/high`, then refitted. Training corner RMS is **1.569160 px**. Pose diversity and the standard hand-eye consistency check pass (maximum translation residual 6.95 mm; rotation residual 0.87 degrees). These consistency metrics belong to the standard solver; the archived camera transform uses joint corner reprojection optimization.

This is a selected training fit, not an independently validated accuracy measurement. The full 14-sample fit had RMS 2.518470 px. Original sessions remain untouched and are not included in the portable package.

## Contents

| Path | Contents |
|---|---|
| `samples/sample_01` through `sample_10` | Selected original samples, including all three raw/rect images, camera metadata, robot feedback and tag detections. Wrist images do not imply wrist extrinsic calibration. |
| `results/cam_high.json` | Recomputed high-camera extrinsics and fit metrics for these 10 samples. |
| `results/selection.json` | Selection criterion, original names and standard-solver check. |
| `results/cam_high_overlay.png` | Best10 simulation over the newly captured local fixed reference, alpha 0.5. |
| `results/render_report.json` | Render inputs, hashes, requested/applied joints and camera readback. Temporary paths identify the original render run. |
| `results/simulation_scene_template.json` | Inherited platform_1 robot scene template. Renderer overrides camera pose/intrinsics and joints; this is not a calibrated platform_2 scene configuration. |
| `reference/sample.json`, `reference/*_raw.png`, `reference/*_rect.png` | Local fixed reference captured on this machine at 22:14, with actual stationary robot feedback. Camera and feedback acquisition is sequential, not hardware synchronized. |
| `reference/reference.json` | Preserved platform_1-derived motion target, not the measured platform_2 feedback. |
| `reference/snapshot.json` | Local reference provenance. |
| `intrinsics/realsense_color_640x480_30fps.json` | Three camera color intrinsics recorded during the local reference capture. |
| `board/` | AprilTag 36h11 ID 0, 100 mm black-square edge; A4 print assets. |
| `hardware.yaml` | Two followers, no masters, and the three local camera serials. |
| `solve.py`, `requirements.txt` | Standalone offline solver copied from platform_1, with the package label updated. |
| `manifest.json` | Source mapping and SHA-256 hashes for the frozen package. |

`T_base_camera` maps OpenCV camera coordinates (x right, y down, z forward) to the **left arm base**, in meters. The fixed algorithm jointly fits this transform and `T_reference_tag`, with fixed intrinsics and tag geometry, PARK/DANIILIDIS initialization and soft-L1 corner reprojection optimization. No manual extrinsic offsets are applied.

## Offline reproduction

From the repository root, use the existing shared environment:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  artifacts/calibration/platform_2/solve.py \
  --output /tmp/platform2_high_recomputed.json
```

The output must not already exist. The solver checks the frozen hashes before computing and does not access hardware. For a standalone copy, install `requirements.txt` into the destination environment first.

Coarse alignment still uses `../platform_1/coarse_reference`; the local motion configuration remains `configs/local_two_followers.yaml`. Only this curated package is unignored for Git; raw sessions and additional local captures remain local.
