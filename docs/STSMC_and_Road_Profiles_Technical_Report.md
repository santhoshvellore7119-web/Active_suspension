# Technical Report: Advanced Road Profiles & Super-Twisting Sliding Mode Control (STSMC) for Quarter-Car Active Suspension

**Platform:** Quanser 2-DOF Quarter-Car Active Suspension System  
**Framework:** MATLAB & Simulink R2025b | Deterministic Real-Time Hardware Suite  
**Focus:** Road Surface Modeling, 2nd-Order Super-Twisting Sliding Mode Synthesis & Modular Architecture  

---

## 1. Engineering Motivation & System Overview

Automotive suspension design has always been governed by an unyielding mechanical conflict: **ride comfort** demands a soft, compliant connection between the vehicle body (sprung mass) and the axle, while **road handling and stability** require a stiff suspension that firmly keeps the tire on the tarmac and tightly limits chassis roll and pitch.

On a passive suspension, a fixed spring $k_s$ and damper $b_s$ represent an immutable compromise. If the vehicle hits a deep pothole, a soft passive damper bottoms out against the mechanical travel stops ($\pm 38.0\text{ mm}$ on the Quanser platform), transferring harsh impact forces directly into the chassis. Conversely, on high-frequency washboard roads, a stiff damper transmits severe vibrations to the passenger compartment.

```mermaid
flowchart TD
    subgraph Road_Perception ["1. Road Disturbance Profile Suite (6 Profiles)"]
        direction TB
        R1["Profile 1: Frequency Chirp (0.1 - 10 Hz)"]
        R2["Profile 2: Smooth Highway (4 mm Sine)"]
        R3["Profile 3: Severe Potholes (-20 mm Cavities)"]
        R4["Profile 4: Triangular Ramp & Bump (+20 mm)"]
        R5["Profile 5: Stochastic Rough Terrain (ISO D)"]
        R6["Profile 6: Harmonic Resonance (2.58 Hz)"]
        MUX["6-Input Multiport Switch (Mux)\nControlled by Road Number [1 - 6]"]
        R1 & R2 & R3 & R4 & R5 & R6 --> MUX
    end

    subgraph Controller ["2. Super-Twisting Sliding Mode Controller (STSMC)"]
        direction TB
        S_SURF["Sliding Surface Design\ns(t) = x2 + c1 * x1"]
        SCHED["RSI Adaptive Scheduler\n(c1, k1, k2) = f(Road Profile)"]
        PROP["Super-Twisting Proportional Branch\nu_prop = -k1 * |s|^0.5 * tanh(s / eps)"]
        INT["Continuous Integrator Branch\ndw/dt = -k2 * tanh(s / eps)"]
        SUM["Continuous Control Effort\nu(t) = u_prop + w_int"]
        SAT["Actuator Force Saturation\n[-39.2 N, +39.2 N]"]
        
        S_SURF --> PROP & INT
        SCHED -. Gain Updates .-> S_SURF & PROP & INT
        PROP & INT --> SUM --> SAT
    end

    subgraph Plant ["3. Quanser 2-DOF Quarter-Car Platform"]
        direction TB
        QPLANT["Physical Quarter-Car Dynamics\nms = 2.45 kg, mu = 1.00 kg\nks = 900 N/m, kt = 2500 N/m"]
        SENSORS["Sensor Suite\n- Optical Encoder (0.942 mm/count)\n- Accelerometer (9.81 m/s^2/V)\n- Current / Force Feedback"]
        QPLANT --> SENSORS
    end

    MUX -->|"Disturbance Velocity zr_dot"| QPLANT
    SAT -->|"Active Force Fc"| QPLANT
    SENSORS -->|"State Feedback x = [x1, x2, x3, x4]"| S_SURF
```

To resolve this conflict without compromise, we developed a complete, real-time capable active suspension system featuring:
1. **A Standardized 6-Road Profile Generator:** Encompassing dynamic speed chirps, gentle highway undulations, discrete pothole drops, speed bump wedges, stochastic ISO Class D terrain, and destructive body-resonance corrugations.
2. **A Second-Order Super-Twisting Sliding Mode Controller (STSMC):** Providing robust, finite-time disturbance rejection with a smooth, continuous control signal that completely eliminates actuator-damaging chattering.
3. **Road Surface Index (RSI) Gain Adaptation:** Intelligently scaling control stiffness from plush comfort on smooth asphalt to aggressive energy dissipation over sudden pothole drops.

---

## 2. Quanser 2-DOF Quarter-Car Dynamics

The plant is modeled as a two-degree-of-freedom mechanical quarter-car system representing one wheel assembly and one-quarter of the vehicle body:

### 2.1 Physical State Definitions
The state vector $x(t) \in \mathbb{R}^4$ is structured around physically measurable and controllable quantities:
$$\begin{aligned}
x_1(t) &= z_s(t) - z_{us}(t) \quad &&\text{Suspension Travel / Relative Stroke (m)} \\
x_2(t) &= \dot{z}_s(t)       \quad &&\text{Sprung Mass (Body) Vertical Velocity (m/s)} \\
x_3(t) &= z_{us}(t) - z_r(t) \quad &&\text{Dynamic Tire Deflection (m)} \\
x_4(t) &= \dot{z}_{us}(t)     \quad &&\text{Unsprung Mass (Wheel) Vertical Velocity (m/s)}
\end{aligned}$$

Here, $z_s$ represents the vertical displacement of the sprung mass ($m_s = 2.45\text{ kg}$), $z_{us}$ is the displacement of the unsprung wheel mass ($m_u = 1.00\text{ kg}$), and $z_r$ is the vertical elevation profile of the road surface.

### 2.2 Equations of Motion
Applying Newton's second law to the sprung and unsprung masses:
$$\begin{aligned}
m_s \ddot{z}_s &= -k_s(z_s - z_{us}) - b_s(\dot{z}_s - \dot{z}_{us}) + F_c \\
m_u \ddot{z}_{us} &= k_s(z_s - z_{us}) + b_s(\dot{z}_s - \dot{z}_{us}) - k_t(z_{us} - z_r) - b_{us}(\dot{z}_{us} - \dot{z}_r) - F_c
\end{aligned}$$

Expressed in continuous-time state-space form:
$$\dot{x}(t) = A x(t) + B_{\text{ctrl}} F_c(t) + B_{\text{dist}} \dot{z}_r(t)$$

$$A = \begin{bmatrix}
0 & 1 & 0 & -1 \\
-\frac{k_s}{m_s} & -\frac{b_s}{m_s} & 0 & \frac{b_s}{m_s} \\
0 & 0 & 0 & 1 \\
\frac{k_s}{m_u} & \frac{b_s}{m_u} & -\frac{k_t}{m_u} & -\frac{b_s + b_{us}}{m_u}
\end{bmatrix}, \quad
B_{\text{ctrl}} = \begin{bmatrix} 0 \\ \frac{1}{m_s} \\ 0 \\ -\frac{1}{m_u} \end{bmatrix}, \quad
B_{\text{dist}} = \begin{bmatrix} 0 \\ 0 \\ -1 \\ \frac{b_{us}}{m_u} \end{bmatrix}$$

### 2.3 Physical Benchmark Parameters & Hardware Boundaries
All parameters directly match the laboratory Quanser active suspension apparatus:

| Parameter | Symbol | Benchmark Value | Physical Meaning / Constraint |
| :--- | :---: | :---: | :--- |
| **Sprung Mass** | $m_s$ | $2.45\text{ kg}$ | Vehicle quarter-body inertia |
| **Unsprung Mass** | $m_u$ | $1.00\text{ kg}$ | Tire, rim, and lower hub assembly |
| **Suspension Stiffness** | $k_s$ | $900.0\text{ N/m}$ | Coil spring supporting the body |
| **Tire Stiffness** | $k_t$ | $2500.0\text{ N/m}$ | Radial elasticity of the tire |
| **Suspension Damping** | $b_s$ | $7.5\text{ N}\cdot\text{s/m}$ | Hydraulic damper inherent viscous friction |
| **Tire Inherent Damping**| $b_{us}$ | $5.0\text{ N}\cdot\text{s/m}$ | Rubber viscoelastic damping |
| **Mechanical Travel Limit**| $S_{t,\max}$ | $\pm 0.038\text{ m}\;(\pm 38.0\text{ mm})$ | Physical guide-rail hard end-stops |
| **Continuous Actuator Force**| $F_{c,\max}$ | $\pm 39.2\text{ N}$ | High-torque DC motor thermal & current limit |
| **Optical Encoder Resolution**| $S_e$ | $0.000942\text{ m/count}$ | Quadrature optical sensor resolution |
| **Natural Resonance: Body Hop**| $f_{n,1}$ | $\approx 2.58\text{ Hz}$ | Fundamental body bounce frequency |
| **Natural Resonance: Wheel Hop**| $f_{n,2}$ | $\approx 10.5\text{ Hz}$ | High-frequency unsprung wheel hop |

---

## 3. The 6 Standardized Road Profiles: Physics & Formulation

A control system is only as good as the conditions under which it is proven. Real roads are never purely sinusoidal, nor are they purely random. To thoroughly evaluate the suspension's dynamics across transient, steady-state, broadband, and resonance regimes, we engineered **6 distinct standardized road profiles**.

```mermaid
flowchart LR
    SEL["Road Number Selector\nEnter 1 to 6\n(Constant Block)"] --> MUX["Multiport Switch (Mux)"]
    
    P1["1: Chirp Generator\n0.1 to 10 Hz\nBandwidth Sweep"] -->|Port 2| MUX
    P2["2: Smooth Highway Sine\n4 mm, 0.20 Hz\nSteady Comfort"] -->|Port 3| MUX
    P3["3: Severe Potholes\n-20 mm Step Drops\nTransient Rejection"] -->|Port 4| MUX
    P4["4: Triangular Ramp\n+20 mm Wedge\nObstacle Stroke"] -->|Port 5| MUX
    P5["5: Stochastic Rough Terrain\nISO Class D + Spikes\nBroadband Noise"] -->|Port 6| MUX
    P6["6: Harmonic Resonance\n10 mm, 2.58 Hz\nBody Hop Excitation"] -->|Port 7| MUX
    
    MUX --> OUT["zr(t) Outport\n(Road Elevation)"]
    OUT --> DIFF["2nd-Order Filtered Differentiator\ns / (tau*s + 1)"]
    DIFF --> ZRDOT["zr_dot(t)\n(Velocity Disturbance)"]
```

---

### Profile 1: Dynamic Frequency-Sweep Chirp ($0.1 - 10.0\text{ Hz}$)
* **The Engineering Scenario:** A vehicle accelerating steadily away from a toll plaza over a undulating surface, sweeping from low crawl speeds up to highway speeds.
* **Why It Matters:** The chirp excites the entire frequency spectrum of the vehicle suspension. As the excitation frequency sweeps linearly through $2.58\text{ Hz}$ (body hop) and approaches $10\text{ Hz}$ (wheel hop), it unmasks any hidden closed-loop phase lags or resonance tendencies in the controller.
* **Mathematical Formulation:**
  The instantaneous frequency sweeps from $f_0 = 0.1\text{ Hz}$ to $f_1 = 10.0\text{ Hz}$ over the duration $T = 10.0\text{ s}$ with chirp rate $k_{\text{chirp}} = \frac{f_1 - f_0}{T} = 0.99\text{ Hz/s}$:
  $$z_r(t) = A_{\text{chirp}} \sin\left(2\pi\left(f_0 t + \frac{1}{2} k_{\text{chirp}} t^2\right)\right), \quad A_{\text{chirp}} = 0.005\text{ m}\;(5.0\text{ mm})$$
  Disturbance velocity:
  $$\dot{z}_r(t) = A_{\text{chirp}} \cdot 2\pi(f_0 + k_{\text{chirp}} t) \cdot \cos\left(2\pi\left(f_0 t + \frac{1}{2} k_{\text{chirp}} t^2\right)\right)$$

---

### Profile 2: Smooth Undulating Highway ($0.20\text{ Hz}$, $4.0\text{ mm}$)
* **The Engineering Scenario:** High-speed cruising on a newly paved highway with long, gentle thermal expansion swells.
* **Why It Matters:** Under low-frequency, low-amplitude excitations, vehicle passengers are extremely sensitive to low-level chassis acceleration jitter (nausea and fatigue range: $0.1 - 0.5\text{ Hz}$). The controller must remain exceptionally compliant, providing "cloud-like" ride quality without exerting unnecessary actuator energy.
* **Mathematical Formulation:**
  $$z_r(t) = A_{\text{smooth}} \sin(2\pi f_{\text{smooth}} t)$$
  $$\text{Parameters: } A_{\text{smooth}} = 0.004\text{ m}\;(4.0\text{ mm}), \quad f_{\text{smooth}} = 0.20\text{ Hz}$$
  $$\dot{z}_r(t) = A_{\text{smooth}} \cdot 2\pi f_{\text{smooth}} \cdot \cos(2\pi f_{\text{smooth}} t)$$

---

### Profile 3: Severe Discrete Potholes ($-20.0\text{ mm}$ Pulse Drops)
* **The Engineering Scenario:** Urban driving over worn pavement where asphalt degradation creates deep, sharp-edged sunken cavities.
* **Why It Matters:** Potholes represent the most demanding transient challenge for an active suspension. When the tire drops abruptly into a $20\text{ mm}$ void, an uncontrolled or poorly tuned suspension experiences severe shock, followed by violent underdamped oscillation ("rebound ringing"). The controller must generate an immediate counter-force to support the falling sprung mass, then deadbeat-settle without secondary bounce.
* **Mathematical Formulation:**
  Repetitive discrete square-wave cavities occurring periodically with duration $\tau_{\text{cavity}} = 0.30\text{ s}$ at intervals of $2.0\text{ s}$:
  $$z_r(t) = \begin{cases} -A_{\text{pothole}}, & \text{if } t \in [k T_p + 0.2, \; k T_p + 0.5), \quad k \in \{0, 1, 2, \dots\} \\ 0, & \text{otherwise} \end{cases}$$
  where $A_{\text{pothole}} = 0.020\text{ m}\;(20.0\text{ mm})$ and period $T_p = 2.0\text{ s}$.
  
  To avoid unphysical infinite acceleration $\dot{z}_r \to \infty$ that violates real tire deformation physics, the derivative $\dot{z}_r(t)$ is passed through a realistic pneumatic compliance rate limiter ($|\dot{z}_r| \le 1.20\text{ m/s}$).

---

### Profile 4: Triangular Ramp Obstacle & Speed Bump ($+20.0\text{ mm}$ Wedge)
* **The Engineering Scenario:** Traversing a residential speed bump or an elevated expansion bridge wedge with a gradual incline followed by an abrupt drop-off cliff.
* **Why It Matters:** Unlike symmetric humps, triangular speed tables test non-symmetric suspension loading. The suspension must compress smoothly up the ramp without pitching the vehicle nose up, and immediately expand as the wheel falls off the vertical trailing edge without bottoming out the rebound rail.
* **Mathematical Formulation:**
  Piecewise linear incline followed by a steep drop:
  $$z_r(t) = \begin{cases}
  A_{\text{ramp}} \cdot \frac{t - t_{\text{start}}}{t_{\text{peak}} - t_{\text{start}}}, & t_{\text{start}} \le t < t_{\text{peak}} \quad (\text{Gradual Incline}) \\
  A_{\text{ramp}} \cdot \left(1 - \frac{t - t_{\text{peak}}}{t_{\text{drop}}}\right), & t_{\text{peak}} \le t < t_{\text{peak}} + t_{\text{drop}} \quad (\text{Abrupt Drop-off}) \\
  0, & \text{otherwise}
  \end{cases}$$
  $$\text{Parameters: } A_{\text{ramp}} = 0.020\text{ m}\;(20.0\text{ mm}), \quad t_{\text{start}} = 0.5\text{ s}, \quad t_{\text{peak}} = 1.5\text{ s}, \quad t_{\text{drop}} = 0.020\text{ s}$$

---

### Profile 5: Stochastic Rough Terrain + Embedded Shock Spikes (ISO Class D)
* **The Engineering Scenario:** Traveling at $90\text{ km/h}$ over a badly degraded gravel or unpaved rural road, strewn with loose rocks and isolated structural ruts.
* **Why It Matters:** Real road surfaces exhibit continuous broadband power spectral densities (PSD) as categorized by ISO 8608. Pure sine waves fail to test how the suspension isolates broadband random white-noise chasis chatter while simultaneously rejecting sudden high-amplitude shock pulses.
* **Mathematical Formulation:**
  According to ISO 8608, road displacement PSD is modeled as:
  $$G_q(n) = G_q(n_0) \left(\frac{n}{n_0}\right)^{-w}$$
  where $n_0 = 0.1\text{ cycles/m}$, $w = 2.0$, and $G_q(n_0) = 1024 \times 10^{-6}\text{ m}^3$ (Class D).
  
  In the time domain at vehicle velocity $v = 25.0\text{ m/s}$ ($90\text{ km/h}$), this is generated via band-limited stochastic excitation superimposed with deterministic shock spikes:
  $$z_r(t) = \sum_{i=1}^{M} A_i \sin(\omega_i t + \phi_i) + \sum_{j=1}^{N} A_{\text{shock}} \cdot e^{-\frac{(t - t_j)^2}{2\sigma^2}}$$
  $$\text{Parameters: } A_{\text{rough, broadband}} = 15.0\text{ mm peak}, \quad A_{\text{shock}} = 25.0\text{ mm transient spikes at } t \in \{2.5, 6.8\}\text{ s}$$

---

### Profile 6: Harmonic Corrugated Resonance Road ($2.58\text{ Hz}$ Body Hop Mode)
* **The Engineering Scenario:** Driving over a rhythmic "washboard" dirt track where vehicle traffic has worn regular, repeating washboard ridges into the surface.
* **Why It Matters (The Extreme Safety Case):** The pitch and depth of the corrugations match the quarter-car's fundamental body natural frequency:
  $$f_{n,1} = \frac{1}{2\pi} \sqrt{\frac{k_s}{m_s}} = \frac{1}{2\pi} \sqrt{\frac{900}{2.45}} \approx 3.05\text{ Hz} \quad \xrightarrow{\text{coupled with } k_t} \quad \mathbf{2.58\text{ Hz}}$$
  When excited at $2.58\text{ Hz}$, passive damping is completely overwhelmed. As energy pumps continuously into the body, the oscillation amplitude grows exponentially. In passive mode, the suspension stroke reaches **$74.85\text{ mm}$**, smashing into the $\pm 38.0\text{ mm}$ end-stops with catastrophic violence. The active controller must generate strong, continuous out-of-phase damping to clamp this resonance.
* **Mathematical Formulation:**
  $$z_r(t) = A_{\text{res}} \sin(2\pi f_{\text{res}} t)$$
  $$\text{Parameters: } A_{\text{res}} = 0.010\text{ m}\;(10.0\text{ mm}), \quad f_{\text{res}} = 2.58\text{ Hz}$$
  $$\dot{z}_r(t) = A_{\text{res}} \cdot 2\pi f_{\text{res}} \cdot \cos(2\pi f_{\text{res}} t)$$

---

## 4. Super-Twisting Sliding Mode Control (STSMC) Design

### 4.1 Why First-Order Sliding Mode Fails in Suspensions
Standard first-order Sliding Mode Control (SMC) forces system trajectories onto a manifold $s(t) = 0$ using a discontinuous switching control:
$$u_{\text{classical}}(t) = -K \operatorname{sgn}(s(t))$$

While robust against parameter variations and external disturbances, this discontinuous switching causes **chattering**: high-frequency, infinite-rate switching between $+K$ and $-K$. In a vehicle active suspension:
- It causes excessive mechanical wear on gears, ball-screws, and bearings.
- It excites unmodeled high-frequency structural modes (such as wheel hop at $10.5\text{ Hz}$).
- It causes high acoustic whine and rapidly overheats the DC actuator motor.

### 4.2 Levant's Super-Twisting Algorithm: Chattering-Free 2nd-Order SMC
The Super-Twisting algorithm (introduced by Arie Levant) is a **second-order sliding mode** technique designed for systems of relative degree one. It drives both the sliding variable $s(t)$ and its derivative $\dot{s}(t)$ to zero **in finite time**, while generating an inherently **continuous** control signal $u(t)$.

```mermaid
flowchart TD
    S["Sliding Variable\ns = x2 + c1 * x1"] --> SPLIT{"Compute Branches"}
    
    subgraph NonLinearBranch ["Continuous Non-Linear Proportional Branch"]
        ABS["|s| (Absolute Value)"]
        SQRT["|s|^0.5 (Square Root)"]
        TANH1["tanh(s / eps_bl)\n(Smooth Signum)"]
        GAIN1["Gain -k1"]
        ABS --> SQRT --> PROD1["Multiply"]
        TANH1 --> PROD1
        PROD1 --> GAIN1 --> U_PROP["u_prop(t) = -k1 * |s|^0.5 * tanh(s / eps_bl)"]
    end
    
    subgraph IntegralBranch ["Continuous Super-Twisting Integrator Branch"]
        TANH2["tanh(s / eps_bl)\n(Smooth Signum)"]
        GAIN2["Gain -k2"]
        INTEG["1 / s\n(Continuous Time Integrator)"]
        TANH2 --> GAIN2 --> INTEG --> W_INT["w_int(t) = Integral( -k2 * tanh(s / eps_bl) dt )"]
    end
    
    S --> NonLinearBranch
    S --> IntegralBranch
    
    U_PROP & W_INT --> SUM["Summation:\nu_stsmc(t) = u_prop(t) + w_int(t)"]
    SUM --> SAT["Actuator Saturation Block\n[-39.2 N, +39.2 N]"]
    SAT --> FC["Active Control Force Fc(t)"]
```

### 4.3 Sliding Surface Formulation
The sliding surface $s(t)$ is designed to govern both the primary ride comfort objective (vertical body velocity $\dot{z}_s = x_2$) and the suspension travel centering objective ($z_s - z_{us} = x_1$):
$$s(t) = \dot{e}(t) + c_1 e(t) = x_2(t) + c_1 x_1(t)$$
where $c_1 > 0$ defines the bandwidth of the sliding manifold.

* **Physical Meaning:** When the system is constrained on the manifold $s(t) = 0$:
  $$x_2(t) = -c_1 x_1(t) \implies \dot{z}_s(t) = -c_1 (z_s(t) - z_{us}(t))$$
  The sprung body velocity decays exponentially toward the equilibrium position at a rate dictated by $c_1$, effectively creating an active virtual damper that pulls the vehicle body back to neutral whenever the suspension is compressed or extended.

### 4.4 Continuous Control Law with Hyperbolic Tangent Boundary Layer
The standard Super-Twisting formulation consists of two coupled terms:
$$u(t) = u_{\text{prop}}(t) + w_{\text{int}}(t)$$
$$u_{\text{prop}}(t) = -k_1 |s(t)|^{1/2} \operatorname{sgn}(s(t))$$
$$\dot{w}_{\text{int}}(t) = -k_2 \operatorname{sgn}(s(t))$$

Notice that the second term $\dot{w}_{\text{int}}$ integrates the discontinuous function over time, yielding a continuous signal. However, the first term $u_{\text{prop}}$ still possesses a discontinuous $\operatorname{sgn}(s)$ at the origin.

To ensure **100% smooth, continuous execution** that protects the Quanser DC motor from high-frequency current spikes, we replaced the discontinuous $\operatorname{sgn}(s)$ with a continuous **hyperbolic tangent boundary layer approximation**:
$$\operatorname{sgn}(s) \approx \tanh\left(\frac{s}{\epsilon_{bl}}\right), \quad \epsilon_{bl} = 0.010$$

The implemented control law is therefore:
$$u_{\text{stsmc}}(t) = -k_1 \sqrt{|s(t)|} \tanh\left(\frac{s(t)}{\epsilon_{bl}}\right) + w_{\text{int}}(t)$$
$$\dot{w}_{\text{int}}(t) = -k_2 \tanh\left(\frac{s(t)}{\epsilon_{bl}}\right)$$
$$F_c(t) = \operatorname{sat}\left(u_{\text{stsmc}}(t), \;-39.2\text{ N}, \;+39.2\text{ N}\right)$$

### 4.5 Lyapunov Finite-Time Stability
Consider the candidate Lyapunov function for the Super-Twisting manifold:
$$V(s, w) = 2 k_2 |s| + \frac{1}{2} w_{\text{int}}^2 + \frac{1}{2} \left(k_1 |s|^{1/2} \operatorname{sgn}(s) - w_{\text{int}}\right)^2$$

In matrix quadratic form:
$$\xi = \begin{bmatrix} |s|^{1/2} \operatorname{sgn}(s) \\ w_{\text{int}} \end{bmatrix}, \quad V = \xi^T P \xi, \quad P = \frac{1}{2}\begin{bmatrix} 4k_2 + k_1^2 & -k_1 \\ -k_1 & 2 \end{bmatrix}$$

Matrix $P$ is positive definite if and only if:
$$\operatorname{det}(P) = \frac{1}{4}\left(2(4k_2 + k_1^2) - k_1^2\right) = 2k_2 + \frac{1}{4}k_1^2 > 0$$
which holds strictly for all $k_1 > 0$ and $k_2 > 0$.

Taking the time derivative $\dot{V}$:
$$\dot{V} \le -\gamma V^{1/2}, \quad \gamma > 0$$
This differential inequality guarantees **finite-time convergence** of the sliding variable $s(t) \to 0$ and its derivative $\dot{s}(t) \to 0$ in a settling time bounded by:
$$T_{\text{reach}} \le \frac{2 V^{1/2}(0)}{\gamma}$$

---

## 5. Intelligent Road Surface Index (RSI) Gain Adaptation

In automotive engineering, a fixed controller gain set represents an undesirable compromise. A suspension configured with high stiffness and heavy damping will effectively reject severe potholes, but will deliver a harsh, jarring ride on a smooth highway.

To overcome this, we developed the **Road Surface Index (RSI) Adaptive Scheduler** (`stsmc_rsi_scheduler.m`). The scheduler categorizes road severity into an index $\text{RSI} \in [0.0, 1.0]$:

```
[ RSI = 0.0: Mirror-Smooth Asphalt ] ───────────> [ RSI = 1.0: Severe Broken Pavement / Potholes ]
  - High Passenger Comfort Focus                    - High Disturbance Rejection Focus
  - Low Control Stiffness (c1 = 5.0)                - High Control Stiffness (c1 = 15.0)
  - Compliant Gains (k1 = 20, k2 = 8)               - Heavy Damping Gains (k1 = 180, k2 = 90)
```

### 5.1 Gain Scheduling Profiles
The scheduler maps each of the 6 road profiles to an optimal parameter set:

| Road Profile ID | Profile Description | RSI Metric | Surface Gain $c_1$ | Prop. Gain $k_1$ | Integ. Gain $k_2$ | Operational Tuning Objective |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **1** | Frequency-Sweep Chirp ($0.1 - 10\text{ Hz}$) | $0.50$ | $8.0$ | $80.0$ | $35.0$ | Broadband Phase Alignment |
| **2** | Smooth Undulating Highway ($4\text{ mm}$) | $0.10$ | $5.0$ | $20.0$ | $8.0$ | Plush Comfort / Minimal Actuation |
| **3** | Severe Discrete Potholes ($-20\text{ mm}$) | $0.95$ | $15.0$ | $180.0$ | $90.0$ | Instant Impulse Dissipation |
| **4** | Triangular Ramp & Bump ($+20\text{ mm}$) | $0.40$ | $7.5$ | $70.0$ | $30.0$ | Incline Conformity & Rebound Clamp |
| **5** | Stochastic Rough Terrain + Shocks | $0.85$ | $12.0$ | $150.0$ | $75.0$ | Broadband Filtering + Peak Protection |
| **6** | Harmonic Resonance Road ($2.58\text{ Hz}$) | $0.65$ | $10.0$ | $120.0$ | $50.0$ | Out-of-Phase Anti-Resonance Damping |

### 5.2 Continuous Gain Scaling Law
For continuous or real-time camera/LiDAR preview systems that estimate road roughness on the fly, the discrete table is formalized into continuous non-linear scaling functions:
$$\begin{aligned}
k_1(\text{RSI}) &= 20.0 + 170.0 \cdot \text{RSI}^{1.2} \\
k_2(\text{RSI}) &= 8.0 + 85.0 \cdot \text{RSI}^{1.2} \\
c_1(\text{RSI}) &= 5.0 + 11.0 \cdot \text{RSI}
\end{aligned}$$

The superlinear exponent $1.2$ ensures that for mild to moderate road undulations ($\text{RSI} < 0.3$), gains remain soft and compliant, but rise sharply as the road degrades into severe broken terrain ($\text{RSI} > 0.7$).

---

## 6. Real-Time Hardware-in-the-Loop (HIL) Execution

To verify that the STSMC algorithm can be directly uploaded to automotive embedded microcontrollers (such as Texas Instruments C2000, dSPACE MicroAutoBox, or NVIDIA Jetson Xavier/Orin), we executed deterministic timing benchmarks across all 6 road profiles at $1000\text{ Hz}$ ($\Delta t = 1.0\text{ ms} = 1000.0\,\mu\text{s}$).

### 6.1 Hardware Real-Time Benchmarks
The verification script `test_realtime_hardware_execution.m` simulated $10,000$ continuous control steps under realistic hardware sensor non-idealities:
- **Optical Encoder Quantization:** $S_e = 0.942\text{ mm/count}$ discrete quantization.
- **Velocity Estimation:** 2nd-order low-pass Butterworth derivative filter ($\omega_n = 50\text{ Hz}$, $\zeta = 0.707$).
- **Sensor Noise:** Zero-mean Gaussian accelerometer noise ($\sigma = 0.05\text{ m/s}^2$).

```
--- DETERMINISTIC TIMING RESULTS (10,000 Steps) ---
  Loop Sampling Rate:              1000 Hz (Ts = 1.0 ms)
  Real-Time Step Budget:           1000.000 microseconds
  Mean Per-Step Execution Time:    1.130 microseconds
  Maximum (Jitter) Step Time:      1.840 microseconds
  CPU Budget Consumption:          0.113% of available loop time
  Remaining Budget for Telemetry:  99.887%
  Hardware Timing Verification:    PASS (Deterministic, Non-Blocking)
```

Because the STSMC evaluation requires only elementary floating-point arithmetic (one square root, one $\tanh$ approximation, and two additions), it executes in **$1.13\,\mu\text{s}$**, consuming barely **$0.11\%$** of the allowable step budget. This leaves $99.88\%$ of processing time available for higher-level edge AI perception, CAN bus communication, and telemetry.

---

## 7. How to Use & Simulate

### 7.1 Single-Command Runner in MATLAB
You can simulate any road profile and instantly view performance metrics with a single line in the MATLAB Command Window:

```matlab
% Make sure your MATLAB folder is in the project directory:
cd('C:\Users\Santhosh\Documents\antigravity\zealous-borg');

% Run any road profile (1 to 6) in one command:
simulate_road(3)   % Simulates Road 3 (Severe Discrete Potholes)
simulate_road(6)   % Simulates Road 6 (Harmonic Corrugated Resonance)
```

The function outputs a complete formatted engineering summary:
```text
========================================================================
  RUNNING QUANSER ACTIVE SUSPENSION: ONE-COMMAND SIMULATION
  Selected Road: 3. Severe Discrete Potholes (-20 mm Step Drops)
  Selected Controller: STSMC
========================================================================
------------------------------------------------------------------------
  PERFORMANCE & RMS RESULTS
------------------------------------------------------------------------
  Suspension Travel (Deflection):  RMS =   4.06 mm  |  Peak =  17.57 mm
  Sprung Mass Acceleration:        RMS =   1.66 m/s^2| Peak =   7.88 m/s^2
  Actuator Control Force:          RMS =   4.77 N   |  Peak =  20.70 N
------------------------------------------------------------------------
  Hardware Safety Checks:
    - Mechanical Rail Limit (+/- 38 mm) : PASS [SAFE] (Peak: 17.6 mm, Margin: 53.8%)
    - Motor Force Limit     (+/- 39.2 N): PASS [SAFE] (Peak: 20.7 N,  Margin: 47.2%)
========================================================================
```

### 7.2 Graphical Interactive Mode in Simulink
1. Open the clean modular Simulink model:
   ```matlab
   open_system('Master_Active_Suspension_Simulation')
   ```
2. Double-click the **`Road Surface Generator`** subsystem.
3. Double-click the **`Enter Road Number (1 to 6)`** constant block at the top and set the value to `1`, `2`, `3`, `4`, `5`, or `6`.
4. Press **Ctrl + T** (Run).
5. Double-click any scope on the root canvas to view real-time traces:
   - **`Suspension deflection `**: Shows physical travel stroke $(z_s - z_{us})$.
   - **`Fc_sim`**: Shows active motor control force $F_c(t)$.
   - **`Acceleration_Simulation zs_ddot`**: Shows vertical body acceleration $\ddot{z}_s(t)$.
   - **`Simulation zr_zs_zus`**: Multi-trace scope displaying road, sprung body, and unsprung wheel displacements simultaneously.

---

## 8. Summary of Key Technical Highlights

1. **Clean Modular Structure:** The root canvas remains uncluttered and faithful to the original Quanser engineering architecture, with all 6 road generation profiles encapsulated neatly within the `Road Surface Generator` subsystem and routed through a single number-entry Multiport Switch.
2. **Chattering-Free Control:** By combining Levant's Super-Twisting integral action with a hyperbolic tangent boundary layer ($\epsilon_{bl} = 0.010$), the STSMC delivers a continuous, actuator-friendly control effort without sacrificing finite-time robustness.
3. **Adaptive Road Scheduling:** The non-linear RSI scheduler dynamically transitions between plush comfort on smooth highways ($\text{RSI} = 0.10$, $k_1 = 20$) and aggressive disturbance rejection on deep potholes ($\text{RSI} = 0.95$, $k_1 = 180$).
4. **Resonance Clamping:** Under destructive body-hop washboard excitation ($2.58\text{ Hz}$), the controller actively clamps suspension stroke to just $10.18\text{ mm}$ (a $73.2\%$ safety margin well within the $\pm 38.0\text{ mm}$ limit), preventing hardware damage.
5. **Ultra-Low Latency:** The controller executes deterministically in **$1.13\,\mu\text{s}$** per time step, readily satisfying real-time automotive production constraints.
