# HALO: Hazard-Aware Landing Optimization for Autonomous Systems

[![HALO: Hazard Aware Landing Optimization for Autonomous Systems (ICRA 2023)](https://img.youtube.com/vi/watch?v=KqCXGDTntDU&ab_channel=AutonomousControlLaboratory/0.jpg)](https://www.youtube.com/watch?v=KqCXGDTntDU&ab_channel=AutonomousControlLaboratory)

## Abstract
**(See the full paper on arXiv [here](https://arxiv.org/abs/2304.01583)**
With autonomous aerial vehicles enacting safety-critical missions, such as the Mars Science Laboratory Curiosity rover's landing on Mars, the tasks of automatically identifying and reasoning about potentially hazardous landing sites is paramount. This paper presents a coupled perception-planning solution which addresses the hazard detection, optimal landing trajectory generation, and contingency planning challenges encountered when landing in uncertain environments. Specifically, we develop and combine two novel algorithms, Hazard-Aware Landing Site Selection (HALSS) and Adaptive Deferred-Decision Trajectory Optimization (Adaptive-DDTO), to address the perception and planning challenges, respectively. The HALSS framework processes point cloud information to identify feasible safe landing zones, while Adaptive-DDTO is a multi-target contingency planner that adaptively replans as new perception information is received. We demonstrate the efficacy of our approach using a simulated Martian environment and show that our coupled perception-planning method achieves greater landing success whilst being more fuel efficient compared to a nonadaptive DDTO approach.

## Overview
This repository contains submodules to both the **Hazard-Aware Landing Site Selection (HALSS)** (TBD) and **Adaptive Deferred-Decision Trajectory Optimization (Adaptive-DDTO)** repositories. In each aforementioned repository, you will find a 📜`demo.ipynb` file that illustrates how each algorithm works.

## Where is the AirSim component?
For multiple reasons, the authors have decided to leave out the AirSim component of this work in the public release. If you are interested in using or replicating our results in the AirSim environment, please reach out to either `haynec@uw.edu` or `sbuckne1@uw.edu` directly and we can discuss setting it up.
