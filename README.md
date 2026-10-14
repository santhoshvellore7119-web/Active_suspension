# Edge AI-Based Adaptive Active Suspension System with STSMC & RSI-Scheduled Profiles

A turnkey, production-grade MATLAB & Simulink project extending the Quanser Quarter-Car Active Suspension System with **Super-Twisting Sliding Mode Control (STSMC)**, **4-Tier Road Severity Index (RSI)** road excitation profiles, and **Intelligent Real-Time Gain Scheduling**.

---

## 📌 Project Overview & System Architecture

This project extends the baseline LQR-controlled Quanser Active Suspension experiment into an advanced non-linear adaptive architecture. It combines high-robustness 2nd-order sliding mode control with proactive road classification:

```
+---------------------------------------------------------------------------------------------------+
|                                  CLOSED-LOOP ADAPTIVE WORKFLOW                                     |
|                                                                                                   |
|  [ 1. Road Perception ]       [ 2. Intelligent Decision ]      [ 3. Adaptive Control ]            |
|     Road Disturbance       -->      RSI Diagnostic      -->       Adaptive STSMC                  |
|     (Smooth / Normal /              Gain Scheduler                u(t) = -k1*|s|^0.5*sgn(s)       |
|      Rough / Pothole)               (k1, k2, lambda)                     + \int -k2*sgn(s)dt      |
|                                                                          |                        |
|                                                                          v                        |
|  [ 4. Performance Evaluation ]                                [ Actuator Saturation (+/-39.2 N) ] |
|     RMS Body Acceleration  <--  Quanser 2-DOF Quarter-Car  <--           |                        |
|     Suspension Travel Limit     Active Suspension Plant                  v                        |
|     Dynamic Tire Load           (ms=2.45 kg, mu=1.00 kg)       [ Active Motor Force Fc ]          |
+---------------------------------------------------------------------------------------------------+
```

---

## 📁 Repository Structure & File Manifest

| File / Folder | Purpose & Description |
| :--- | :--- |
| `setup_as.m` | **Master Setup Script**: Populates workspace with plant matrices $A, B, C, D$, Quanser parameters, LQR gains (Classical, ACO-Opt, HS-Opt), STSMC parameters, and RSI road settings. |
| `Master_Active_Suspension_Simulation.slx` | **Simulink Model**: Complete comparative model featuring selectable controllers (Passive, LQR, STSMC), native continuous RMS calculation subsystems, and live canvas digital Displays. |
| `build_comparative_models.m` | **Simulink Model Builder**: Programmatically constructs the native Simulink model with continuous RMS calculation blocks and digital Displays inspired by Dinesh's benchmark architecture. |
| `stsmc_block.m` | **Core STSMC Control Law**: Continuous-time super-twisting algorithm with boundary-layer smoothing ($\epsilon_{bl}=10^{-2}$) to prevent chattering and solver stalls. |
| `stsmc_rsi_scheduler.m` | **RSI Gain Scheduler**: Maps Road Severity Index ($0.10, 0.35, 0.65, 0.95$) to controller gains ($k_1, k_2, c_1$). |
| `design_stsmc.m` | **Interactive Controller Designer**: Configures STSMC gain modes (`conservative`, `moderate`, `aggressive`, `grant_normal`, `rsi_adaptive`). |
| `validate_baseline.m` | **Phase 0 Baseline Verification**: Analyzes controllability, modal eigenvalues, validates LQR gains, and checks hardware physical travel limits. |
| `quanser_specs.m` | **Hardware Specifications**: Confirms mechanical suspension travel limit ($38\text{ mm}$), motor torque constant, and sensor sensitivities from Quanser User Manual. |
| `run_comparative_sweep.m` | **Automated Comparative Sweep**: Simulates all 24 combinations ($4\text{ roads} \times 6\text{ controllers}$), computes RMS/peak performance, and prints summary tables. |
| `comparative_sweep_results.csv` | **Numerical Benchmark Dataset**: Full exported metrics (RMS/Peak acceleration, travel, tire deflection, force, execution time). |
| `figures/` | **High-Resolution Figures**: Publication-quality comparison plots (Figures 1–6) matching IEEE Access / Mechatronics paper format. |

---

## 🔬 Mathematical Formulation

### 1. Quanser Quarter-Car Plant Model
State vector $x = [x_1, x_2, x_3, x_4]^T$:
$$\begin{aligned}
x_1 &= z_s - z_{us} \quad &&\text{(Suspension deflection / travel, m)} \\
x_2 &= \dot{z}_s   \quad &&\text{(Sprung mass body velocity, m/s)} \\
x_3 &= z_{us} - z_r \quad &&\text{(Tire deflection, m)} \\
x_4 &= \dot{z}_{us} \quad &&\text{(Unsprung mass wheel velocity, m/s)}
\end{aligned}$$

State-space equations:
$$\dot{x}(t) = A x(t) + B_{dist} \dot{z}_r(t) + B_{ctrl} F_c(t)$$

$$A = \begin{bmatrix}
0 & 1 & 0 & -1 \\
-\frac{k_s}{m_s} & -\frac{b_s}{m_s} & 0 & \frac{b_s}{m_s} \\
0 & 0 & 0 & 1 \\
\frac{k_s}{m_u} & \frac{b_s}{m_u} & -\frac{k_t}{m_u} & -\frac{b_s+b_{us}}{m_u}
\end{bmatrix}, \quad
B_{ctrl} = \begin{bmatrix} 0 \\ \frac{1}{m_s} \\ 0 \\ -\frac{1}{m_u} \end{bmatrix}, \quad
B_{dist} = \begin{bmatrix} 0 \\ 0 \\ -1 \\ \frac{b_{us}}{m_u} \end{bmatrix}$$

### 2. Super-Twisting Sliding Mode Controller (STSMC)
Sliding manifold:
$$s(t) = \dot{e}(t) + \lambda e(t) = x_2(t) + c_1 x_1(t)$$

Super-Twisting control law:
$$u(t) = -k_1 \sqrt{|s(t)|} \, \text{sign}(s(t)) + w_{int}(t)$$
$$\dot{w}_{int}(t) = -k_2 \, \text{sign}(s(t))$$

Boundary-layer smoothing (prevents actuator chatter):
$$\text{sign}(s) \approx \frac{s}{|s| + \epsilon_{bl}}, \quad \epsilon_{bl} = 10^{-3}$$

### 3. RSI Diagnostic Matrix & Gain Scheduling
| Road Condition | RSI (Normalized) | Action Mode | Proposal ($k_1, k_2, \lambda$) | Plant-Tuned Scaled ($k_1, k_2, \lambda$) |
| :--- | :---: | :--- | :---: | :---: |
| **Smooth Road** | `0.10` | Comfort Focus | $(8, 2, 5)$ | $(20, 8, 5)$ |
| **Normal Road** | `0.35` | Moderate Damping | $(12, 4, 7)$ | $(60, 25, 7)$ |
| **Rough Road** | `0.65` | Increased Damping | $(18, 7, 10)$ | $(120, 50, 10)$ |
| **Pothole** | `0.95` | Max Disturbance Rejection | $(25, 10, 15)$ | $(180, 90, 15)$ |

---

## 📊 Comprehensive Comparative Results (24 Benchmark Runs)

### TABLE I: RMS Body Acceleration [Ride Comfort] ($\text{m/s}^2$)
*Lower is better (indicates passenger comfort enhancement)*

| Road Condition | Passive | Classical LQR | ACO-LQR | HS-LQR | Fixed STSMC | **Adaptive STSMC** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Smooth (RSI 0.10)** | 0.01748 | 0.00415 | 0.00528 | 0.00723 | **0.00045** | 0.00535 |
| **Normal (RSI 0.35)** | 0.08742 | 0.02073 | 0.02638 | 0.03616 | **0.00189** | 0.02812 |
| **Rough (RSI 0.65)** | 1.59073 | 0.17882 | 0.58781 | 0.77565 | **0.13622** | 0.63377 |
| **Pothole (RSI 0.95)** | 1.94711 | **0.32766** | 1.11574 | 1.22592 | 0.45207 | 1.21429 |

### TABLE II: RMS Suspension Deflection [Travel Stroke] ($\text{m}$)
*All peak values verified $\le 0.038\text{ m}$ against hardware rail limit*

| Road Condition | Passive | Classical LQR | ACO-LQR | HS-LQR | Fixed STSMC | **Adaptive STSMC** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Smooth (RSI 0.10)** | 0.000047 | 0.001333 | 0.000722 | 0.000154 | 0.002776 | 0.000663 |
| **Normal (RSI 0.35)** | 0.000236 | 0.006665 | 0.003608 | 0.000772 | 0.013754 | 0.002446 |
| **Rough (RSI 0.65)** | 0.004239 | 0.003871 | 0.003392 | 0.001859 | 0.004397 | 0.002063 |
| **Pothole (RSI 0.95)** | 0.005165 | 0.007044 | 0.006198 | 0.003067 | 0.013585 | **0.002953** |

### TABLE III: Peak Control Force [Actuator Demand] ($\text{N}$)
*Hardware continuous limit: $\pm 39.2\text{ N}$*

| Road Condition | Passive | Classical LQR | ACO-LQR | HS-LQR | Fixed STSMC | **Adaptive STSMC** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Smooth (RSI 0.10)** | 0.00 | 1.79 | 0.95 | 0.25 | 3.55 | 0.87 |
| **Normal (RSI 0.35)** | 0.00 | 8.96 | 4.75 | 1.23 | 17.59 | 3.18 |
| **Rough (RSI 0.65)** | 0.00 | 9.11 | 12.62 | 5.26 | 11.45 | 6.83 |
| **Pothole (RSI 0.95)** | 0.00 | 30.85 | 39.20 | 16.91 | 33.48 | 21.87 |

---

## 🚀 Quickstart & Usage Instructions

### 1. Baseline Verification (Phase 0)
```matlab
% In MATLAB Command Window
cd('Active_Suspension_STSMC_Project')
validate_baseline
```

### 2. Configure STSMC Gains (Phase 1 & 3)
```matlab
% Select one of the available modes:
design_stsmc('conservative');  % Minimum travel
design_stsmc('moderate');      % Standard baseline
design_stsmc('aggressive');    % Maximum ride comfort
design_stsmc('rsi_adaptive');  % Auto-scheduled based on road severity
```

### 3. Run Automated Comparative Sweep (Phase 4)
```matlab
run_comparative_sweep
```

### 4. Interactive Simulink Simulation
1. Run `setup_as.m` to load parameters into the workspace.
2. Open `Master_Active_Suspension_Simulation.slx`.
3. Set `ROAD_PROFILE_ID` (`1`=Smooth, `2`=Normal, `3`=Rough, `4`=Pothole).
4. Set `CONTROLLER_ID` (`0`=Passive, `1`=Classical LQR, `2`=ACO LQR, `3`=HS LQR, `4`=Fixed STSMC, `5`=Adaptive STSMC).
5. Click **Run** in Simulink and inspect scopes!

---

## 🎯 Grant Scoping & Edge AI Roadmap

1. **Simulink Scope vs Hardware Scope**: This package implements and validates Blocks 2–4 of the grant methodology (Intelligent Decision Making, Adaptive Control, Performance Evaluation).
2. **Road Perception (Edge AI)**: The actual RSI input ($0.10, 0.35, 0.65, 0.95$) is currently selected via parameter index. In the full physical prototype, this index will be fed directly in real-time over UDP / shared memory by the NVIDIA Jetson AGX Orin running a lightweight CNN road classifier from monocular camera frames.
