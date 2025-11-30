# Solution to MATLAB and Simulink Challenge project 247 Multi-UAV Path Planning for Urban Air Mobility

---

# Project details

See **UAM_Documentation**:
- **Section 1:** Overview of Implemented Functions
- **Section 2:** Technical Completeness and Innovation
- **Section 3:** Detailed Implementation Process

---

# How to run section

> **Note:** Please run the following scripts in sequence, as each step depends on the previous one.

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

See results and demonstration videos in the **videos_and_photos** folder, or refer to **Section 4: Results and Demonstrations** of the **UAM_Documentation** PDF.

---

# Reference

See **Section 6: References** of the **UAM_Documentation** PDF.

---

## Code Function Structure Diagram

See **Section 5: Code File Dependency Diagram** of the **UAM_Documentation** PDF, or the corresponding images in the **videos_and_photos** folder.

---

**Note:**  
Absolutely all content, instructions, and documentation are included in **UAM_Documentation**.
