# Solution to MATLAB and Simulink Challenge project <'project number'> <'Project Title'>
This is a template repo for MATLAB and Simulink Challenge Project solutions.

Please add the following items:

[Program link](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub)

[Project description link]<Add link to the project description from the list of projects above>

---

# Project details

See **UAM_Documentation**:
- **Section 1:** Project Introduction and Background  
- **Section 2:** Implementation Details  
- **Section 3:** Algorithm Design and System Architecture  

---

# How to run section

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

See results and demonstration videos in the **videos_and_photos** folder, or refer to **Section 4** of the **UAM_Documentation** PDF.

---

# Reference

See **Section 6** of the **UAM_Documentation** PDF.

---

## Code Function Structure Diagram

See **Section 5** of the **UAM_Documentation** PDF, or the corresponding images in the **videos_and_photos** folder.

---

**Note:**  
Absolutely all content, instructions, and documentation are included in **UAM_Documentation**.
