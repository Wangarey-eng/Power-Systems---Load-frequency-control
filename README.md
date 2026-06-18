# Load Frequency Control (LFC) in Power Systems

This repository contains a MATLAB & Simulink framework designed to simulate and analyze **Load Frequency Control (LFC)** (also known as Automatic Generation Control - AGC) in an electrical power system. The simulation models how a generation area dynamically responds to sudden changes in load demand to restore system frequency ($50\text{Hz}$ or $60\text{Hz}$) and maintain scheduled power exchanges.

---

## 📚 Theoretical Background

In a power system, a continuous balance between total power generation ($P_g$) and total load demand ($P_d$) must be maintained. Any mismatch between generation and load results in a kinetic energy exchange with the rotating masses of the synchronous generators, causing a system frequency deviation ($\Delta f$).

### The Generator-Load Governance Equation
The baseline dynamics of the grid area frequency can be modeled via the linearized swing equation structure:

$$\frac{d\Delta f}{dt} = \frac{1}{2H} \left( \Delta P_g - \Delta P_d - D \cdot \Delta f \right)$$

Where:
* **$\Delta f$**: Frequency deviation (Hz or per-unit).
* **$H$**: Combined generator inertia constant (seconds).
* **$D$**: Load damping constant (% change in load per % change in frequency).
* **$\Delta P_g$**: Change in mechanical power generation.
* **$\Delta P_d$**: Change in non-frequency-dependent load demand.

### Primary vs. Secondary Control Loops
1. **Primary Control (Governor Speed Droop):** Instantaneous, proportional response based on a governor droop characteristic ($R$). It arrests the frequency drop but leaves a steady-state frequency error.
2. **Secondary Control (LFC Loop):** Uses an Integral/PI controller acting on the **Area Control Error (ACE)** to completely eliminate the steady-state frequency error ($\Delta f \to 0$) and return the system to nominal frequency.

---

## 📊 Simulink Model Architecture

The Simulink model (`.slx`) implements the standard transfer function blocks representing the physical system components linked in a closed feedback loop:



+------------+      +-------------+      +----------------+
|  Governor  | ---> |   Turbine   | ---> | Generator/Load | ---> Delta f (Output)
|  1/(1+sTg) |      |  1/(1+sTt)  |      |   1 / (2Hs+D)  |    |
+------------+      +-------------+      +----------------+    |
^                                                        |
|------------- (Droop Feedback: -1/R) -------------------|
|                                                        |
+------------- (Secondary PI Control Loop) --------------+


### Core Subsystems Configured:
* **Governor Subsystem:** Modeled with a first-order lag transfer function $\frac{1}{1 + sT_g}$, where $T_g$ is the governor time constant.
* **Turbine Subsystem:** Modeled with a first-order lag transfer function $\frac{1}{1 + sT_t}$, where $T_t$ is the steam/hydro turbine charging time constant.
* **Generator-Load (Inertia) Subsystem:** Modeled as $\frac{1}{2Hs + D}$, capturing the combined rotational inertia of the power grid area.
* **Secondary PI Controller:** Takes the frequency deviation feedback and integrates it to adjust the speed changer motor position ($P_{ref}$), forcing the steady-state error to zero.

---

## 🛠️ File Layout & Interaction

The project is structured with an initializing script paired directly to the graphical simulation space:

### 1. `LFC_Parameters.m` (MATLAB Initialization Script)
This file defines all system metrics, time constants, and gain values directly inside the MATLAB workspace variables before running the model:
* System constants: Base Power ($S_{base}$), Frequency ($f_0$).
* Dynamic constants: Inertia ($H$), Damping ($D$), Speed Droop ($R$).
* Controller tuning parameters: Proportional Gain ($K_p$), Integral Gain ($K_i$).
* Disturbance setting: $\Delta P_d$ (Step block value for load increase).

### 2. `Load_Frequency_Control.slx` (Simulink Workspace)
The graphic system diagram that references the workspace variables initialized by the `.m` script. It runs a continuous-time ordinary differential equation (ODE) solver to plot dynamic frequency trajectories following a load disturbance.

---

## 🚀 How to Run the Simulation

1. **Run the Initialization Script:** Open and run `LOAD_FREQUENCY_CONTROL.m` inside MATLAB to initialize your parameters ($H, D, R, K_i$) into the workspace.
2. **Open the Simulink Model:** Open `LoadFrequencyControlslx`.
3. **Execute:** Click the **Run** button inside Simulink.
4. **Analyze the Output:** Open the scope block observing $\Delta f$ to inspect system response traits:
   * **Frequency Nadir:** The maximum transient frequency dip before governor acceleration takes over.
   * **Settling Time:** The speed at which the secondary integral control loop eliminates the frequency deviation.
