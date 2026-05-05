# 3D Passive Positioning Landing Simulator

A PyBullet-based physics simulation that evaluates passive-positioning drone landings on angled I-shaped platforms. The simulator sweeps a grid of start positions over each platform configuration, classifies every landing attempt, and produces per-case CSV data and heatmap plots.

## Overview

The drone is released from a fixed clearance height above each platform and falls under gravity alone — there is **no thrust, no attitude stabilisation, and no active control** of any kind during descent or landing. The outcome depends entirely on the drop position, platform geometry, and passive contact dynamics. Each attempt is classified into one of three outcomes:

| Result | Meaning |
|---|---|
| **Successful** | Drone settles on the platform arms within position and tilt tolerances, no body–platform collision |
| **Success but collision detected** | Landing meets all success criteria, but the drone body contacted the platform during descent |
| **Unsuccessful** | Drone slides off, tips over, or fails to settle within the time limit |

## Simulation Parameters

| Parameter | Value |
|---|---|
| Base sizes | 100 mm – 400 mm in 10 mm steps (31 sizes) |
| Platform angles | 30°–75° in 15° steps (4 angles) |
| Drone leg lengths | 300 mm – 600 mm in 10 mm steps (31 lengths) |
| Arm thickness | 2 mm |
| Landing grid resolution | 5 mm |
| Drop clearance | 40 mm above the shallowest platform peak |
| Gravity | 9.81 m/s² |
| Physics time step | 1 ms |
| Settle duration | 1.0 s of continuous low-velocity stability |
| Settle drift limit | 2 mm (independent of arm thickness) |
| Max landing attempt time | 3.0 s |
| Position tolerance | 5 mm from centre |
| Tilt tolerance | 10° |

## Getting Started

### Prerequisites

- **Python 3.x** — download from [python.org](https://www.python.org/downloads/) if not already installed.
- **Microsoft Visual C++ 14.0** (Windows only) — required to build certain native packages. Install "Build Tools for Visual Studio" from [here](https://visualstudio.microsoft.com/visual-cpp-build-tools/).

### Dependencies

Install the required libraries with pip:

```
pip install pybullet pybullet_data numpy pandas matplotlib Pillow
```

### URDF Files

The following URDF files must be present in the `urdf/` directory:

- `urdf/I_shape.urdf` — the angled platform segment
- `urdf/drone.urdf` — the quadrotor template model

At runtime the simulator generates a temporary copy of `drone.urdf` with the arm link geometry, mass, and inertia tensor rewritten to match each swept leg length. The temporary file is cleaned up automatically after each case.

### Running the Simulation

```
python simulator.py
```

The simulation runs all 3,844 parameter cases (31 base sizes × 4 angles × 31 drone leg lengths) using parallel workers by default. Progress is displayed in the terminal.

To watch a single run in the PyBullet GUI, set `VISUALIZE = True` at the top of `simulator.py` (this forces single-worker mode).


## Output

### CSV Data (`csv_results/`)

Each parameter case produces two CSV files:

- **`<case>.csv`** — raw simulation results with columns: `StartX`, `StartY`, `Time (s)`, `Result`.
- **`<case>_processed.csv`** — processed copy where unsuccessful landing times are replaced by the minimum successful time (when at least one success exists), preserving the raw data.

A summary file **`success_rate_summary.csv`** aggregates per-case statistics including success rates and the maximum fully-successful round landing dimension.

### Heatmap Plots (`plot_results/`)

Each case generates a heatmap plot saved as a PNG at 300 DPI. The heatmap maps every grid position to its landing outcome using a discrete three-colour palette:

- 🟢 `#62BB46` — Successful
- 🟡 `#FFC20E` — Success but collision detected
- 🔴 `#F15B40` — Unsuccessful

Axes are labelled in millimetres. The plot uses Times New Roman for publication readiness.

### Combined Plots (`plot_results/all_results_combined_leg_<length>m.png`)

Generated automatically at the end of the simulation, one composite image is created for each drone leg length (31 images total). Each composite arranges the heatmaps for that leg length in a **31 rows × 4 columns** grid:

- **Rows** — base sizes from 100 mm (top) to 400 mm (bottom)
- **Columns** — platform angles from 30° to 75° in 15° steps

All heatmaps share the same axis limits (based on the largest base size) so that plots are directly comparable across different base sizes. Each cell contains the corresponding heatmap plot, with row and column labels for easy reference. The image is saved at 300 DPI for publication use.

## Stopping the Simulation

Press `Ctrl+C` in the terminal. The simulator handles the interrupt gracefully and shuts down parallel workers.

## Author

- Chanaka Chathuranga Ubesingha

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
