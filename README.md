# HALO: Hazard-Aware Landing Optimization for Autonomous Systems

[![HALO: Hazard Aware Landing Optimization for Autonomous Systems (ICRA 2023)](media/youtube_snapshot.png)](https://www.youtube.com/watch?v=KqCXGDTntDU&ab_channel=AutonomousControlLaboratory "HALO: Hazard Aware Landing Optimization for Autonomous Systems (ICRA 2023)")

## Abstract
**(See the full paper [here]([https://arxiv.org/abs/2304.01583](https://ieeexplore.ieee.org/document/10160655)))**

With autonomous aerial vehicles enacting safety-critical missions, such as the Mars Science Laboratory Curiosity rover's landing on Mars, the tasks of automatically identifying and reasoning about potentially hazardous landing sites is paramount. This paper presents a coupled perception-planning solution which addresses the hazard detection, optimal landing trajectory generation, and contingency planning challenges encountered when landing in uncertain environments. Specifically, we develop and combine two novel algorithms, Hazard-Aware Landing Site Selection (HALSS) and Adaptive Deferred-Decision Trajectory Optimization (Adaptive-DDTO), to address the perception and planning challenges, respectively. The HALSS framework processes point cloud information to identify feasible safe landing zones, while Adaptive-DDTO is a multi-target contingency planner that adaptively replans as new perception information is received. We demonstrate the efficacy of our approach using a simulated Martian environment and show that our coupled perception-planning method achieves greater landing success whilst being more fuel efficient compared to a nonadaptive DDTO approach.

## Overview
This repository contains submodules to both the **Hazard-Aware Landing Site Selection (HALSS)** and **Adaptive Deferred-Decision Trajectory Optimization (Adaptive-DDTO)** repositories, as well as code to interface these submodules together in the **AirSim** environment. In each aforementioned submodule repository, you will find a 📜`demo.ipynb` file that illustrates how each algorithm works. Results from the full paper cannot be precisely replicated at this time, as the full HALO environment assets cannot be made public, however instructions are provided below to enable simulation in a similar environment with a provided digital elevation map (DEM).

If any information in this README is unclear, any setup issues are encountered, or consultation for a specific application is desired, please feel free to reach out to the corresponding authors:
* Christopher Hayner (`haynec@uw.edu`)
* Samuel Buckner (`sbuckne1@uw.edu`)

## Setup

### Software Requirements:
* Unreal Engine version 4.27.2 (see setup instructions [here](https://www.unrealengine.com/en-US/free-download/game-development-engine?utm_source=GoogleSearch&utm_medium=Performance&utm_campaign=20121040491&utm_id=150460841753&utm_term=video%20game%20engines&utm_content=669868333880))
* Microsoft AirSim platform (see setup instructions [here](https://microsoft.github.io/AirSim/))
* Julia -- latest version (see setup instructions [here](https://julialang.org/downloads/))
* Anaconda -- latest version (see setup instructions [here]([https://www.anaconda.com/](https://www.anaconda.com/download))

### Environment Configuration:
TBD (will use `Airsim/setup/halo_depthmap.png`)

### Other Steps:
The settings file provided at `AirSim/setup/settings.json` must be relocated on a Windows machine to the path `C:/Documents/AirSim/settings.json` as per AirSim setup instructions. 

Additionally, a new Anaconda ("conda") environment must be created using the `environment.yaml` file with the following commmand:
```
$ conda env create -f environment.yml
```

## Operation
HALSS and Adaptive-DDTO are currently configured to communicate through a manual publisher-subscriber architecture using [NumPy data files](https://numpy.org/devdocs/user/how-to-io.html) temporarily stored in `AirSim/temp`. This will eventually be refactored for communication in [ROS](https://www.ros.org/) instead. The `AirSim/` folder contains two run files, `run_halss.py` and `run_addto.py`, along with utility scripts and functions in the `utils/` folder. Most hyperparameters can be set in either of these top-level run files (with some exceptions that must be set in `AdaptiveDDTO/src/params.jl`); please see the next section for more information. While HALSS is natively written in Python, Adaptive-DDTO is entirely written in the Julia language, with the `PyCall.jl` package used to facilitate communication with Adaptive-DDTO codebase.

To simulate a HALO scenario, open two separate terminals, `cd` to the top level of this repository, and begin with activating the configured Anaconda environment in both:
```
$ conda activate halo
```

The actual AirSim simulation loop is initiated in `run_addto.py`, and so proceed to calling the following in one terminal:
```
$ python AirSim/run_addto.py
```

This script will proceed with vehicle setup until the vehicle is positioned at the desired initial conditions (set in the corresponding run file). Once the vehicle is positioned correctly, the script will pause for user input with the line `[INPUT]: Press any key to begin landing maneuver`. At this point, switch to the other terminal, and proceed to call the HALSS process:
```
$ python AirSim/run_halss.py
```

Once the HALSS process has began producing a (consistently-updated) plotting interface, switch back to the Adaptive-DDTO process terminal, and press any key to engage the simulation loop. We note that the first time Julia runs the Adaptive-DDTO code stack, it is also compiling, and so the first trajectory execution will take a considerably-longer amount of time than all proceeding executions. We also note that HALSS can also be instantiated as a Python subprocess by setting the flag `flag_HALSS_subprocess = True` in `run_addto.py`, allowing this whole simulation to be executed in one terminal, however doing this will prevent access to debugging print statements in the HALSS process.

## Scenario and Hyperparameter Configuration
We will briefly describe some important notes on hyperparameters and scenario configuration for a specific application.

### HALSS
The `run_halss.py` script contains a variety of flags to enable plotting and debugging capabilities, as well as `flag.flag_coarse_method`. This flag is particularly important, as it controls whether the HALSS segmentation network is used (as described in the full-length paper) or not. If the user wants to run a HALO scenario without extra configuration in their environment, the geometric approach to coarse landing site selection (`geo` flag) can be used for any landing surface. However, if the segmentation network is desired (`nn` flag), the network must be trained for the specific environment using `HALSS/HALSS_utils/network_utils/train.py` (more documentation TBD -- please reach out to the corresponding authors to discuss training for a specific application). Specific parameters relating to the HALSS algorithm, as well as to the safety of the vehicle (see `params.alpha`), are also set here.

### Adaptive-DDTO
Due to limitations with the `PyCall` package, configuration for Adaptive-DDTO is handled in two separate locations: the `run_addto.py` script and the `AdaptiveDDTO/src/params.jl` file.

In `run_addto.py`, all scenario parameters can be set, such as the initial conditions of the vehicle in terms of position and velocity, tracking parameters such as the guidance lock (`h_cut`) and landing success criterion (`h_term`) altitudes, and other simulation parameters. Additionally, many flags for simulation control, visualization modification and debugging can be set here.

In `AdaptiveDDTO/src/params.jl`, all Adaptive-DDTO algorithm parameters and vehicle configuration can be set. Many parameters are already appropriately set for the default AirSim quadcopter object in a landing scenario, however if the problem definition had to change for a specific application (in terms of optimization constraints, for example), parameters can be appropriated adjusted in the `Lander` object located here (along with modifying the problem definition in `solve_ddto.jl` and `solve_optimal.jl`).

> [!IMPORTANT]  
> HALSS and Adaptive-DDTO have one "shared" parameter: the maximum number of landing sites to be considered. This parameter must be set to the same desired value in **two** locations (or else unexpected behavior may occur): `params.max_sites` in `run_halss.py` and `n_targs_max` in `AdaptiveDDTO/src/params.jl`.

## Citing
If you use either of the aforementioned algorithms, kindly cite the following associated publication.
```
@inproceedings{hayner2023halo,
  title={HALO: Hazard-Aware Landing Optimization for Autonomous Systems},
  author={Hayner, Christopher R and Buckner, Samuel C and Broyles, Daniel and Madewell, Evelyn and Leung, Karen and A{\c{c}}ikme{\c{s}}e, Beh{\c{c}}et},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={3261--3267},
  year={2023},
  organization={IEEE}
}
```
