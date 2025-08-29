# Mecanum Plugin for CoppeliaSim

This project implements a custom **Python Remote API plugin** for CoppeliaSim to
improve realism in simulating Mecanum wheel robots.  
It introduces:

- **Roller-aware lateral friction** (Stribeck model)  
- **Motor dynamics surrogate** (first-order lag + torque/current limits)  

Goal: narrow the sim-to-real gap, so controllers tuned in simulation transfer more faithfully to hardware.

---

## Features
- Injects realistic **roller slip and drift** at each physics step
- Models **motor lag and torque limits**
- Logs results as CSV for analysis
- Supports **ROS /cmd_vel** input or local test scripts

---

## Repo Layout
- `mecanum_plugin.py` – main plugin
- `mecanum_params.json` – parameter config file
- `examples/demo_cmdvel.py` – scripted motion commands
- `examples/plot_results.py` – plots slip, yaw drift, and velocity traces
- `scenes/mecanum_base.ttt` – sample CoppeliaSim scene with a 4-wheel base
- `logs/` – generated CSV logs
- `docs/` – diagrams for architecture and workflow

---

## Installation

1. Clone repo  
   ```bash
   git clone https://github.com/<your-username>/mecanum-plugin-sim.git
   cd mecanum-plugin-sim
