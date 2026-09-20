# CLAUDE.md — mrac_vehicle

Research-grade MRAC (Model Reference Adaptive Control) vehicle dynamics simulation project.

## Setup

### Docker (recommended)

```bash
docker-compose build
docker-compose run python3 bash
```

### Local

```bash
pip install -r requirements.txt
pip install -e adaptive_system -e vehicle_dynamics -e util
```

Python 3.6+ required. Key dependencies: `control`, `numpy`, `scipy`, `sympy`, `cvxopt`, `pykalman`, `matplotlib`, `slycot`.

## Running experiments

All simulation scripts live in `experiments/`. Run from the project root after installing packages:

```bash
python experiments/path_following_fix_vx.py
python experiments/byrun.py
```

Output CSVs are written to `data/output/`. Gnuplot scripts in `visualization/` generate plots from those CSVs.

## Architecture

Three independently installable sub-packages compose together in experiment scripts:

```
adaptive_system/       # Adaptive control algorithms
├── mrac/              #   Model Reference Adaptive Control (primary)
├── dmrac/             #   Direct MRAC variant
└── backstepping/      #   Backstepping controller

vehicle_dynamics/      # Vehicle simulation & dynamics
├── vehicle.py         #   Nonlinear bicycle model (7-state)
├── vehicle_LTI.py     #   Linear time-invariant model
├── vehicle_dynamics.py #  Core dynamics equations
├── vehicle_params.py  #   Parameter container (mass, Iz, cf, cr, lf, lr)
└── linear_system.py   #   control.TransferFunction/StateSpace wrapper

util/                  # Utility functions
├── bezier.py          #   Cubic Bezier curve for path representation
└── util.py            #   Distance calculations
```

## Data flow

```
Road data (data/*.txt)
  → Vehicle model (nonlinear or LTI) with VehicleParam
  → Design reference model (transfer function / state-space)
  → Design auxiliary filters w1, w2 via lambda polynomial
  → AdaptiveController: u = theta^T * phi
  → Simulation loop (dt = 0.01s):
      plant output yp, reference output yr, error e = yp - yr
      parameter update: dtheta = -e * gamma * phi
  → CSV output (data/output/) → Gnuplot visualization (visualization/)
```

## Experiment script naming conventions

- `_fix_vx` — fixed longitudinal velocity
- `_var_vx` — variable velocity
- `_lti_` — uses linear vehicle model
- `_cbf` — Control Barrier Function safety constraint
- `_stat` — static parameters (no adaptation)
- `_pid` — PID controller comparison
- `_umax` — input saturation limits
