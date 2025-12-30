# Solution to MATLAB and Simulink Challenge project 247 Multi-UAV Path Planning for Urban Air Mobility

---

Due to the rapid development of the low-altitude economy, future parcel delivery will become increasingly unmanned and is likely to be handled by drones. In China, couriers currently deliver packages from distribution centers to parcel lockers located in each building of residential communities. We envision that, in the future, drones could take over these tasks. Therefore, we designed this project to allow multiple drones to depart from aerial charging stations (which also serve as pickup points) in a section of Manhattan, deliver packages to various task points in order of priority based on energy efficiency and time optimality, and finally return to the charging station closest to their last task point.

> **Note:**  
> Absolutely all content, instructions, and details are included in **UAM_Document**.  
> For completeness, both Markdown and PDF versions of UAM_Document are provided.

# Project details

See **UAM_Document**:
- **Section 1:** Overview of Implemented Functions
- **Section 2:** Technical Completeness and Innovation
- **Section 3:** Detailed Implementation Process

---

# How to run section

> **Note:**  
> Please run the following scripts **in sequence**, as the outputs of each step are required by the next.

1. **First**, add the `src` directory to the MATLAB path.
2. **Run** `manhattan_3d_planning.m` (main function):
   - This will generate:
     - MTSP task allocation diagram
     - BiRRT path planning figure
     - Minimum Snap smoothed trajectory figure
     - UAVs' actual trajectory figure
     - A before/after comparison chart (X, Y, Z channels) for a smoothed path segment
3. **Run** `export_and_run_active_uavs.m`:
   - Generates individual animation videos for each UAV's solo flight.
4. **Run** `viisual_all_traj2.m`:
   - Generates an animation for all UAVs flying together.
5. **For collision testing:**  
   - Run `test1.m`, `test2.m`, or `test3.m` at any time to display collision detection and avoidance results.
6. **MATLAB Toolboxes Required:**  
   - **MATLAB (Core)**
   - **Aerospace Toolbox / Aerospace Blockset**
   - **Robotics System Toolbox**
   - **Navigation Toolbox**
   - **Sensor Fusion and Tracking Toolbox**
   - **Optimization Toolbox**
   - **Global Optimization Toolbox**
   - **Curve Fitting Toolbox**
   - **Statistics and Machine Learning Toolbox**
   - **Automated Driving Toolbox**
   - **Mapping Toolbox**
   - **Simulink / Simulink 3D Animation**

---

# Demo/Results

See results and demonstration videos in the **results** folder, or refer to **Section 4: Results and Demonstrations** of the **UAM_Document**.

> **Note:** All videos in the **results** folder must be downloaded locally to view.

## Version Updates

1. The functions `manhattan_3d_planning` and `uav_avoidance_with_helper2` have been updated to support collision avoidance among multiple UAVs.
2. A new live script, `Scenario_Configurator`, has been added to configure the entire simulation.



---

# Reference

See **Section 6: References** of the **UAM_Document**.

---

## Code Function Structure Diagram

See **Section 5: Code File Dependency Diagram** of the **UAM_Document**, or the corresponding images in the **results** folder.
