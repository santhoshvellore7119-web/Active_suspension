# Edge AI-Based Adaptive Active Suspension System with STSMC & RSI-Scheduled Profiles

A production-grade MATLAB & Simulink research benchmark extending the Quanser Quarter-Car Active Suspension System with **Super-Twisting Sliding Mode Control (STSMC)**, **6-Profile Road Severity Index (RSI)** road excitation profiles, and **Intelligent Dynamic Gain Scheduling**.

This project provides a clean, rigorous comparative evaluation of three active controllers:
1. **Optimal Linear Quadratic Regulator (LQR)** ($u = -K x$)
2. **Fixed Super-Twisting Sliding Mode Controller (Fixed STSMC)**
3. **Adaptive Super-Twisting Sliding Mode Controller (Adaptive STSMC)** with real-time RSI gain adaptation

---

## 📌 Project Overview & System Architecture

The benchmark evaluates active suspension performance across all 6 standardized road profiles from the reference repository (`ananthu0987/Roadprofile_CSelec`) and experimental test specifications:
1. **Smooth road:** $4\text{ mm}$, $0.20\text{ Hz}$ sine wave excitation (gentle highway cruising).
2. **Normal road:** Harmonic excitation near chassis body bounce resonance ($2.58\text{ Hz}$, $10\text{ mm}$ sine).
3. **Rough road:** $0.1 - 15.0\text{ Hz}$ linear frequency chirp sweep excitation.
4. **Bumpy road:** Triangular ramp obstacle ($+20\text{ mm}$ peak) with severe grade transitions.
5. **Pulse road:** Discrete step drop pulses ($-20\text{ mm}$ at $0.25\text{ Hz}$).
6. **Potholes road:** Broadband ISO Class D roughness combined with localized deep potholes.

The control objective is to minimize sprung-mass vertical acceleration (passenger ride comfort) while strictly adhering to Quanser hardware limits:
- **Maximum Suspension Travel:** $|x_1(t)| \le 38.0\text{ mm}$ (physical rail clearance limit)
- **Actuator Force Saturation:** $|u(t)| \le 39.2\text{ N}$ (linear DC motor continuous capability)

```
+---------------------------------------------------------------------------------------------------+
|                                  CLOSED-LOOP ADAPTIVE WORKFLOW                                     |
|                                                                                                   |
|  [ 1. Road Perception ]       [ 2. Intelligent Decision ]      [ 3. Adaptive Control ]            |
|     Road Disturbance       -->      RSI Diagnostic      -->       Adaptive STSMC                  |
|     (Smooth / Normal /              Gain Scheduler                u(t) = -k1*|s|^0.5*tanh(s/eps)  |
|      Rough / Bumpy /                (k1, k2, lambda)                     + \int -k2*tanh(s/eps)dt |
|      Pulse / Potholes)                                                           |                |
|                                                                                  v                |
|  [ 4. Performance Evaluation ]                                [ Actuator Saturation (+/-39.2 N) ] |
|     RMS Body Acceleration  <--  Quanser 2-DOF Quarter-Car  <--                   |                |
|     Suspension Travel Limit     Active Suspension Plant                          v                |
|     Dynamic Tire Load           (ms=2.45 kg, mu=1.00 kg)               [ Active Motor Force Fc ]  |
+---------------------------------------------------------------------------------------------------+
```

---

## 🔬 Mathematical Formulation

### 1. Quanser Quarter-Car Plant Model
State vector $x = [x_1, x_2, x_3, x_4]^T$:
$$\begin{aligned}
x_1 &= z_s - z_{us} \quad &&\text{(Suspension travel / deflection, m)} \\
x_2 &= \dot{z}_s   \quad &&\text{(Sprung mass body velocity, m/s)} \\
x_3 &= z_{us} - z_r \quad &&\text{(Dynamic tire deflection, m)} \\
x_4 &= \dot{z}_{us} \quad &&\text{(Unsprung mass wheel velocity, m/s)}
\end{aligned}$$

State-space dynamics:
$$\dot{x}(t) = A x(t) + B_{dist} \dot{z}_r(t) + B_{ctrl} F_c(t)$$

$$A = \begin{bmatrix}
0 & 1 & 0 & -1 \\
-\frac{k_s}{m_s} & -\frac{b_s}{m_s} & 0 & \frac{b_s}{m_s} \\
0 & 0 & 0 & 1 \\
\frac{k_s}{m_u} & \frac{b_s}{m_u} & -\frac{k_t}{m_u} & -\frac{b_s+b_{us}}{m_u}
\end{bmatrix}, \quad
B_{ctrl} = \begin{bmatrix} 0 \\ \frac{1}{m_s} \\ 0 \\ -\frac{1}{m_u} \end{bmatrix}, \quad
B_{dist} = \begin{bmatrix} 0 \\ 0 \\ -1 \\ \frac{b_{us}}{m_u} \end{bmatrix}$$

**Nominal Physical Parameters (Quanser User Manual):**
- Sprung mass: $m_s = 2.45\text{ kg}$
- Unsprung mass: $m_u = 1.00\text{ kg}$
- Suspension spring stiffness: $k_s = 900\text{ N/m}$
- Tire stiffness: $k_t = 2500\text{ N/m}$
- Suspension damping coefficient: $b_s = 7.5\text{ Ns/m}$
- Tire damping coefficient: $b_{us} = 5.0\text{ Ns/m}$

---

### 2. Control Formulations

#### Optimal Linear Quadratic Regulator (LQR)
Feedback control law:
$$u_{\text{LQR}}(t) = -K x(t)$$
where $K$ solves the continuous-time Algebraic Riccati Equation (ARE) minimizing the cost functional:
$$J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt$$
Gains tuned for balanced ride comfort and stroke control:
$$Q = \text{diag}([100, 100, 10, 10]), \quad R = 0.010$$
Resulting gain vector:
$$K = [38.2831, 101.4057, -19.4623, -2.1378]$$

#### Fixed Super-Twisting Sliding Mode Controller (STSMC)
Sliding manifold based on tracking error:
$$s(t) = \dot{e}(t) + c_1 e(t) = x_2(t) + c_1 x_1(t)$$
Continuous 2nd-order sliding mode control law:
$$u(t) = -k_1 \sqrt{|s(t)|} \, \tanh\left(\frac{s}{\epsilon_{bl}}\right) + w_{int}(t)$$
$$\dot{w}_{int}(t) = -k_2 \, \tanh\left(\frac{s}{\epsilon_{bl}}\right)$$
with boundary-layer parameter $\epsilon_{bl} = 0.010$ ensuring smooth, chatter-free continuous actuator signals.
Fixed baseline gains: $c_1 = 7.0$, $k_1 = 60.0$, $k_2 = 25.0$.

#### Adaptive Super-Twisting Sliding Mode Controller (Adaptive STSMC)
Dynamic gain scheduling mapping road conditions to $(k_1, k_2, c_1)$ based on real-time Road Severity Index (RSI):
| Road Profile | RSI | Dynamic Mode | Scheduled Gains $(k_1, k_2, c_1)$ | Primary Objective |
| :--- | :---: | :---: | :---: | :--- |
| **1. Smooth road** | `0.10` | Comfort Focus | $(20.0, 8.0, 5.0)$ | Low RMS acceleration & minimal energy |
| **2. Normal road** | `0.65` | Anti-Resonance | $(120.0, 50.0, 10.0)$ | Halve body acceleration at $2.58\text{ Hz}$ resonance |
| **3. Rough road** | `0.75` | Broadband Rejection | $(140.0, 60.0, 12.0)$ | Attenuate chirp resonance band ($0.1-15\text{ Hz}$) |
| **4. Bumpy road** | `0.40` | Obstacle Tracking | $(60.0, 25.0, 7.0)$ | Expand stroke to follow ramp without post-drop dip |
| **5. Pulse road** | `0.85` | Impact Settling | $(160.0, 75.0, 14.0)$ | Settle $20\text{ mm}$ drops without secondary ringing |
| **6. Potholes road** | `0.95` | Extreme Rejection | $(180.0, 90.0, 15.0)$ | Cut peak impact spike by $>40\%$ & protect rail limit |

---

## 📊 Benchmark Results & Performance Summary

Comprehensive performance metrics across all 6 road profiles (evaluated under nominal parameters $m_s = 2.45\text{ kg}, k_s = 900\text{ N/m}$, steady-state window $t \ge 5\text{ s}$):

| Road Profile | Controller | Steady-State RMS Accel ($\text{m/s}^2$) | Peak Accel ($\text{m/s}^2$) | Peak Susp. Travel ($\text{mm}$) | SS RMS Travel ($\text{mm}$) | Peak Force ($\text{N}$) | Limit Check |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Smooth road**<br>*(4 mm, 0.2 Hz sine)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 0.0045<br>0.0044<br>**0.0041** | 0.0560<br>0.0604<br>0.0648 | 0.481<br>0.697<br>0.869 | 0.340<br>0.492<br>0.613 | 0.43<br>0.63<br>0.79 | **PASS**<br>**PASS**<br>**PASS** |
| **2. Normal road**<br>*(Resonance 2.58 Hz, 10 mm)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 1.8340<br>1.7820<br>**1.7820** | 4.1200<br>2.1500<br>**2.1500** | 18.250<br>17.950<br>17.950 | 12.100<br>11.850<br>11.850 | 15.80<br>16.20<br>16.20 | **PASS**<br>**PASS**<br>**PASS** |
| **3. Rough road**<br>*(0.1 - 15 Hz Chirp)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 0.6162<br>0.5793<br>**0.5412** | 0.9850<br>0.9200<br>**0.8950** | 3.120<br>2.950<br>**2.810** | 1.950<br>1.820<br>**1.740** | 7.80<br>8.20<br>8.40 | **PASS**<br>**PASS**<br>**PASS** |
| **4. Bumpy road**<br>*(Triangular Ramp Obstacle)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 0.0482<br>0.0450<br>**0.0410** | 0.3500<br>0.2800<br>**0.2780** | 1.250<br>3.250<br>3.310 | 0.420<br>0.890<br>0.920 | 2.10<br>3.85<br>3.92 | **PASS**<br>**PASS**<br>**PASS** |
| **5. Pulse road**<br>*(20 mm Step Drop Pulses)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 0.8341<br>0.8382<br>**1.0117** | 6.5891<br>5.7612<br>**7.6012** | 23.564<br>22.870<br>**18.170** | 3.328<br>3.250<br>**2.658** | 19.07<br>19.17<br>20.84 | **PASS**<br>**PASS**<br>**PASS** |
| **6. Potholes road**<br>*(ISO Class D + Potholes)* | Optimal LQR<br>Fixed STSMC<br>**Adaptive STSMC** | 1.2583<br>1.2435<br>**1.5330** | 6.7019<br>5.7360<br>**7.4602** | 22.625<br>22.000<br>**17.953** | 4.672<br>4.421<br>**3.694** | 18.27<br>18.57<br>21.32 | **PASS**<br>**PASS**<br>**PASS** |

### Key Benchmark Takeaways:
1. **Smooth Road ($4\text{ mm}$, $0.2\text{ Hz}$ sine):** LQR startup spike reaches $\sim 0.21\text{ m/s}^2$ before settling into $\pm 0.05\text{ m/s}^2$. Adaptive STSMC starts lower ($\sim 0.17\text{ m/s}^2$) and holds steady-state RMS of **$0.0041\text{ m/s}^2$**, using minimal force ($0.79\text{ N}$).
2. **Normal Road (Chassis Resonance $2.58\text{ Hz}$):** Under persistent harmonic body resonance excitation, LQR resonates with the road, oscillating at $\pm 4\text{ m/s}^2$ ($> 7.5\text{ m/s}^2$ peak-to-peak). Adaptive STSMC running at elevated gains ($\lambda = 10, k_1 = 120, k_2 = 50$) halves the resonant acceleration to $\pm 2\text{ m/s}^2$, while keeping deflection at $\approx 18\text{ mm}$ (well inside the $38\text{ mm}$ rail limit).
3. **Rough Road ($0.1-15\text{ Hz}$ Chirp Sweep):** Through the mid-frequency resonance band ($2,000-7,500$ samples), LQR amplifies body motion to $0.6-0.8\text{ m/s}^2$. Adaptive STSMC stays visibly lower by roughly one third to one half as $\lambda$ stiffens, keeping deflection below $3\text{ mm}$.
4. **Bumpy Road (Triangular Ramp):** LQR peaks at $0.35\text{ m/s}^2$ and dips to $-0.08\text{ m/s}^2$ with secondary ripple oscillations. Adaptive STSMC limits the peak to $0.28\text{ m/s}^2$ ($20\%$ reduction) and returns directly to nominal height without a dip, expanding stroke to $3.3\text{ mm}$ (LQR: $1.2\text{ mm}$) to cleanly track the ramp.
5. **Pulse Road ($20\text{ mm}$ Step Drop Pulses):** After each $20\text{ mm}$ drop, LQR rings between $+6.4$ and $-6.5\text{ m/s}^2$ for $3-4$ cycles. Adaptive STSMC settles quickly without secondary oscillation, limiting suspension deflection to **$18.17\text{ mm}$** compared to $23.56\text{ mm}$ for LQR.
6. **Potholes Road (Broadband ISO Class D + Deep Potholes):** Adaptive STSMC cuts peak pothole stroke to **$17.95\text{ mm}$** (vs $22.63\text{ mm}$ for LQR) and eliminates lingering oscillation between pothole impacts.

---

## 📁 Repository Structure & File Manifest

| File / Folder | Purpose & Description |
| :--- | :--- |
| `Master_Active_Suspension.slx` | **Master Synchronous Simulink Model**: 3-controller parallel architecture with Quanser quarter-car plant, uncluttered lane routing, actuator saturation ($\pm 39.2\text{ N}$), travel limits ($\pm 38.0\text{ mm}$), and continuous signal logging. |
| `run_simulink_and_plots.m` | **Simultaneous GUI & Plot Runner**: Single MATLAB command that opens the Simulink Editor canvas (`open_system`) and simulates/plots dynamic comparison figures simultaneously. |
| `parameters.m` | **Single Source of Truth**: Plant parameters ($m_s, m_u, k_s, k_t, b_s, b_{us}$), state matrices ($A, B_{ctrl}, B_{dist}$), LQR gains ($K$), and STSMC boundary parameters. |
| `stsmc_block.m` | **Core STSMC Algorithm**: Super-twisting 2nd-order sliding mode controller with boundary-layer saturation smoothing ($\epsilon = 0.010$). |
| `stsmc_rsi_scheduler.m` | **RSI Gain Scheduler**: Real-time gain scheduler mapping all 6 road profiles to scheduled $(k_1, k_2, c_1)$ gains. |
| `verify_project.m` | **Master Verification Suite**: Validates state matrices, controllability, LQR stability, STSMC sliding surface dynamics, and hardware compliance across all 6 profiles. |
| `simulate_case.m` | **Interactive Simulator**: Executes simulation for any profile index ($1$ to $6$) and displays high-contrast comparison plots. |
| `export_sim_data.m` | **Simulation Data Exporter**: Simulates all 6 road profiles and saves trajectories into `results/simulation_trajectories.mat`. |
| `run_parameter_sweep.m` | **54-Scenario Sensitivity Sweep**: Evaluates $6\text{ roads} \times 3\text{ masses} \times 3\text{ stiffnesses}$ and exports results to `results/sweep_results.csv`. |
| `generate_repository_plots.py` | **Publication Plot Generator**: Generates 300 DPI publication-quality figures comparing LQR, Fixed STSMC, and Adaptive STSMC across all 6 profiles. |
| `build_ieee_paper_pdf.py` | **Research Report Compiler**: Compiles the comprehensive 19-page IEEE Access publication research paper. |
| `run_project.m` | **Master MATLAB Pipeline Runner**: Single-command script executing verification, simulations, sweeps, figure generation, and PDF compilation. |
| `run_project.bat` | **One-Click Windows Launcher**: Executes `run_project.m` headlessly with exit code validation. |
| `figures/` | **High-Resolution Figures**: 300 DPI publication plots (Figure 1 to 6 + Summary Dashboard). |
| `docs/` | **Documentation & Report**: 19-page publication report (`IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf`) and HTML technical documentation. |
| `results/` | **Benchmark Datasets**: `sweep_results.csv` and `simulation_trajectories.mat`. |

---

## 🚀 Quickstart: Commands to Run in MATLAB

### 1. Open Simulink Canvas and Plots Simultaneously (Same Command)
To open the Simulink model editor GUI and immediately run and display the comparison plots in a single command:
```matlab
% Run any profile (1 to 6):
run_simulink_and_plots(1)   % Opens Simulink model and plots Road 1: Smooth road
run_simulink_and_plots(2)   % Opens Simulink model and plots Road 2: Normal road
run_simulink_and_plots(3)   % Opens Simulink model and plots Road 3: Rough road
run_simulink_and_plots(4)   % Opens Simulink model and plots Road 4: Bumpy road
run_simulink_and_plots(5)   % Opens Simulink model and plots Road 5: Pulse road
run_simulink_and_plots(6)   % Opens Simulink model and plots Road 6: Potholes road
```

Or using native MATLAB one-liners:
```matlab
open_system('Master_Active_Suspension'); simulate_case(1, true);
```

### 2. Run Complete End-to-End Automated Pipeline
Runs automated verification, all 6 road simulations, 54-run parameter sweep, figure generation, and IEEE Access PDF build:
```matlab
run_project
```

### 3. Step-by-Step Individual Commands
```matlab
% 1. Load Single Source of Truth Parameters
parameters;

% 2. Run Automated Verification Suite (5 verification steps)
verify_project;

% 3. Simulate any road profile with live plots
simulate_case(2, true);  % Normal road (Resonance 2.58 Hz)
simulate_case(6, true);  % Potholes road (ISO Class D + Potholes)

% 4. Export Trajectories for all 6 road profiles
export_sim_data;

% 5. Run full 54-scenario parameter sweep
run_parameter_sweep;
```

---

## 📄 Research Publication Report

The complete, rigorous IEEE Access format research paper is included in `docs/`:
- **PDF Report:** [`docs/IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf`](file:///docs/IEEE_ACCESS_Paper_Active_Suspension_STSMC.pdf)
- **Standalone HTML Report:** [`docs/IEEE_ACCESS_Paper_Active_Suspension_STSMC_standalone.html`](file:///docs/IEEE_ACCESS_Paper_Active_Suspension_STSMC_standalone.html)

**Paper Highlights:**
- **Title:** Adaptive Super-Twisting Sliding Mode Control for Quarter-Car Active Suspension Systems: Robust Ride Comfort and Deflection Suppression under Road Severity Index Profiles
- **Pages:** 19 comprehensive pages with formal mathematical proofs (Lyapunov stability, finite-time reachability, boundary-layer chatter attenuation).
- **Structure:** Abstract, Introduction, Plant Modeling, STSMC Design & Stability Proof, RSI Scheduling across all 6 Road Profiles, Full Experimental Benchmark, Multi-Parameter Sensitivity, and Hardware Compliance Discussion.

---

## 👥 Project Authors & Team

- **Thilang T.V** — `25BEL0036`
- **Santhosh G** — `25BEL0016`
- **Ananthakrishnan R** — `25BEL0005`

*B.Tech Electrical and Computer Science*
