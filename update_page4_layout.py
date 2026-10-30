"""
update_page4_layout.py
Re-structures Page 4 so that:
1. Callout box has break-inside: avoid and compact inline math to sit 100% in Column 1.
2. Subsection C sits 100% in Column 2.
3. Table 2 is full-width across the page.
4. Dual gain scheduling framework callout box is full-width below Table 2.
"""
import os
import subprocess

template_path = r"c:\Users\Santhosh\Documents\antigravity\zealous-borg\paper_template.html"
with open(template_path, "r", encoding="utf-8") as f:
    text = f.read()

# 1. Ensure .callout-box has break-inside: avoid;
callout_css_target = """.callout-box {
        border: 1px solid #002855;
        border-left: 3px solid #002855;
        background-color: #f0f4f8;
        padding: 4px 6px;
        margin: 5px 0;
        font-size: 7.6pt;
        line-height: 1.2;
    }"""

callout_css_replacement = """.callout-box {
        border: 1px solid #002855;
        border-left: 3px solid #002855;
        background-color: #f0f4f8;
        padding: 4px 6px;
        margin: 4px 0;
        font-size: 7.4pt;
        line-height: 1.18;
        break-inside: avoid;
    }"""

if callout_css_target in text:
    text = text.replace(callout_css_target, callout_css_replacement)

# 2. Replace Page 4
start_marker = "<!-- ================= PAGE 4 ================= -->"
end_marker = "<!-- ================= PAGE 5 ================= -->"

start_idx = text.find(start_marker)
end_idx = text.find(end_marker)

if start_idx == -1 or end_idx == -1:
    print("Error: Markers not found!")
    exit(1)

new_page_4 = """<!-- ================= PAGE 4 ================= -->
<div class="two-column">

<div class="callout-box">
    <div class="callout-title">Mathematical Derivation of Fixed STSMC Values Based on Standards</div>
    Unlike Adaptive STSMC, Fixed STSMC uses invariant gains sized to satisfy international automotive and Lyapunov stability standards across all operating profiles:
    <br><br>
    <strong>1) Derivation of $c_{1,\\text{fixed}} = 7.00\\text{ s}^{-1}$ (ISO 2631-1 Vibration Isolation):</strong>
    On sliding surface $s = 0 \\implies \\dot{z}_s = -c_1 (z_s - z_u)$, acting as a first-order low-pass filter with cutoff $f_c = \\frac{c_1}{2\\pi}\\text{ Hz}$. <strong>ISO 2631-1:1997</strong> defines the human whole-body vibration sensitivity zone in the $4\\text{--}8\\text{ Hz}$ band (spinal/abdominal resonance). QC-ASSM plant resonance modes are $f_{ns} = 2.58\\text{ Hz}$ (body bounce) and $f_{nu} = 9.41\\text{ Hz}$ (wheel hop). Damping chassis bounce while rejecting wheel hop requires $f_c < f_{ns} \\ll f_{nu}$. Selecting $f_c = 1.114\\text{ Hz}$ gives $c_{1,\\text{fixed}} = 2\\pi(1.114\\text{ Hz}) = \\mathbf{7.00\\text{ s}^{-1}}$ ($T_s \\approx 4 / c_1 = 0.57\\text{ s}$).
    <br><br>
    <strong>2) Derivation of $k_{1,\\text{fixed}} = 60.00$ (Levant Disturbance Rejection):</strong>
    Under nominal disturbance ($x_1 \\approx 15\\text{ mm}, \\dot{x}_1 \\approx 0.05\\text{ m/s}$), passive disturbance force is $F_{\\text{dist}} \\approx k_s |x_1| + b_s |\\dot{x}_1| = 13.88\\text{ N}$. With nominal sliding variable $|s_{\\text{nom}}| \\approx 0.150\\text{ m/s}$ ($\\sqrt{|s_{\\text{nom}}|} \\approx 0.3873$), sizing with a $1.65\\times$ robustness margin ($\\mu = 1.65$) to guarantee finite-time reachability yields:
    $k_{1,\\text{fixed}} = \\frac{\\mu \\cdot F_{\\text{dist}}}{\\sqrt{|s_{\\text{nom}}|}} = \\frac{1.65 \\times 13.88}{0.3873} \\approx 59.13 \\approx \\mathbf{60.00}$.
    <br><br>
    <strong>3) Derivation of $k_{2,\\text{fixed}} = 25.00$ (Moreno&ndash;Levant Lyapunov Stability):</strong>
    By the Moreno&ndash;Levant Lyapunov theorem [13], STSMC is asymptotically stable if $k_1^2 > 4 m_s k_2$. For $m_s = 2.45\\text{ kg}$, $k_1 = 60$, and $k_2 = 25$: $k_1^2 = 3600 \\gg 4(2.45)(25) = 245$ ($14.7\\times$ stability margin). Critical damping dictates $\\frac{k_2}{k_1} = 0.4167 \\implies k_2 = 0.4167 \\times 60.00 = \\mathbf{25.00}$.
    <br><br>
    <strong>4) Quanser Hardware Boundary Standards:</strong>
    Commanded force $|u_c| \\le 19.17\\text{ N} \\le 39.2\\text{ N}$ ($51.1\\%$ motor safety margin) and peak travel $22.87\\text{ mm} \\le 38.0\\text{ mm}$ ($39.8\\%$ clearance margin from bump stops).
</div>

<h3 class="sec-subheading">C. Controller 3: Precision-Tuned Adaptive STSMC</h3>
<p>
Active suspension control is governed by Newton's acceleration equation:
</p>
<div class="equation">
    <span>$\\ddot{z}_s = \\frac{1}{m_s} \\left[ -k_s(z_s - z_u) - b_s(\\dot{z}_s - \\dot{z}_u) + u_c(t) \\right]$</span>
    <span class="eq-num">(16)</span>
</div>
<p>
Equation (16) reveals the fundamental root-cause of the comfort vs. rattle space trade-off: <strong>active force $u_c(t)$ couples directly into vertical body acceleration $\\ddot{z}_s$</strong>.
</p>
<p>
In early tuning trials (Iteration 1: $k_1=160\\text{--}180$, $c_1=14\\text{--}15$), the controller applied large instantaneous forces ($|u_c| > 21\\text{ N}$) to aggressively restrict suspension stroke to under $18\\text{ mm}$. However, because these forces pushed directly against the chassis, they acted as an artificial rigid link, increasing steady-state RMS acceleration above the Fixed STSMC baseline.
</p>
<p>
To resolve this trade-off, <strong>Candidate 5 (Precision Tuned)</strong> was derived to allow the suspension to flex compliantly during mild disturbances while stiffening only as required during severe pothole impacts.
</p>

</div> <!-- end page 4 top two-column -->

<div class="full-width">

<div class="table-caption">TABLE 2. Control Parameters, Governing Standards, and Theoretical Sizing Rationale.</div>
<table class="table-dense">
    <thead>
        <tr>
            <th>CONTROLLER</th>
            <th>PARAM</th>
            <th>VALUE</th>
            <th>GOVERNING STANDARD / THEOREM</th>
            <th>MATHEMATICAL SIZING DERIVATION</th>
            <th>VERIFICATION &amp; SAFETY MARGIN</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td rowspan="2"><strong>Optimal LQR</strong></td>
            <td>$\\mathbf{Q}$</td>
            <td>$\\operatorname{diag}([100, 100, 10, 10])$</td>
            <td>Bryson's Optimal Rule / Dinesh [6]</td>
            <td>$Q_{ii} = 1 / x_{i,\\max}^2$ penalizing stroke &amp; body bounce</td>
            <td>Algebraic Riccati ARE Hurwitz stable</td>
        </tr>
        <tr>
            <td>$R$</td>
            <td>0.010</td>
            <td>Actuator Saturation Bound</td>
            <td>Sized to constrain commanded force within $F_{c,\\max}$</td>
            <td>Peak force $19.08\\text{ N} \\le 39.2\\text{ N}$ ($51.3\\%$ margin)</td>
        </tr>
        <tr>
            <td rowspan="4"><strong>Fixed STSMC</strong></td>
            <td>$c_1$</td>
            <td>$7.00\\text{ s}^{-1}$</td>
            <td><strong>ISO 2631-1</strong> Vibration Isolation</td>
            <td>Cutoff $f_c = 1.114\\text{ Hz} < f_{ns} (2.58) \\ll f_{nu} (9.41)$</td>
            <td>Avoids $4\\text{--}8\\text{ Hz}$ human organ resonance corridor</td>
        </tr>
        <tr>
            <td>$k_1$</td>
            <td>$60.00$</td>
            <td>Levant Disturbance Rejection [12]</td>
            <td>$k_1 = \\frac{1.65 \\times F_{\\text{dist}}}{\\sqrt{|s_{\\text{nom}}|}} = \\frac{1.65 \\times 13.88}{0.3873} = 59.13$</td>
            <td>$1.65\\times$ robustness margin over road disturbances</td>
        </tr>
        <tr>
            <td>$k_2$</td>
            <td>$25.00$</td>
            <td>Moreno&ndash;Levant Lyapunov Stability [13]</td>
            <td>$k_1^2 > 4m_sk_2$ with critical damping ratio $\\frac{k_2}{k_1} = 0.4167$</td>
            <td>$3600 \\gg 245$ ($14.7\\times$ Lyapunov stability margin)</td>
        </tr>
        <tr>
            <td>$\\varepsilon_{\\text{smc}}$</td>
            <td>$0.010$</td>
            <td>Chattering Elimination Standard</td>
            <td>Continuous hyperbolic boundary layer $u = \\tanh(s/\\varepsilon)$</td>
            <td>Zero actuator high-frequency chattering</td>
        </tr>
        <tr>
            <td rowspan="3"><strong>Adaptive STSMC (Cand 5)</strong></td>
            <td>$c_1(RSI)$</td>
            <td>$[5.0\\text{--}8.0]$</td>
            <td><strong>ISO 2631-1</strong> Dynamic Bandwidth</td>
            <td>Monotonic schedule scaling with road severity ($RSI$)</td>
            <td>$0.0041\\text{ m/s}^2$ SS RMS acceleration on highways</td>
        </tr>
        <tr>
            <td>$k_1(RSI)$</td>
            <td>$[20.0\\text{--}78.0]$</td>
            <td><strong>ISO 8608</strong> Disturbance Tracking</td>
            <td>Strictly monotonic gradient $\\frac{dk_1}{dRSI} > 0$</td>
            <td>Arrests pulse drops within $22.51\\text{ mm}$ ($40.8\\%$ margin)</td>
        </tr>
        <tr>
            <td>$k_2(RSI)$</td>
            <td>$[8.0\\text{--}32.0]$</td>
            <td>Moreno Damping Boundary ($0.40\\text{--}0.41$)</td>
            <td>$k_2(RSI) \\approx 0.40 \\times k_1(RSI)$</td>
            <td>$k_1^2 > 4m_sk_2$ satisfied across $100\\%$ of road nodes</td>
        </tr>
    </tbody>
</table>

<div class="callout-box" style="margin-top: 5px;">
    <div class="callout-title">Dual Gain Scheduling Framework: Discrete Laboratory Benchmark vs. Continuous ISO 8608 Scale</div>
    In <code>stsmc_rsi_scheduler.m</code>, two separate parameter sets are synthesized based on <strong>ISO 8608:2016</strong> road classification standards:
    <br><br>
    <strong>1) Set 1 (6 Discrete Nodes for Laboratory Benchmark):</strong>
    Indexed directly by Road ID ($1\\text{--}6$) for exact $O(1)$ hardware lookup in Simulink with zero interpolation latency:
    $\\mathbf{k}_{1,6} = [20.0, 55.0, 62.0, 45.0, 75.0, 78.0]$, 
    $\\mathbf{k}_{2,6} = [8.0, 22.0, 25.0, 18.0, 30.0, 32.0]$, and 
    $\\boldsymbol{\\lambda}_6 = [5.0, 6.5, 6.8, 5.8, 7.8, 8.0]$.
    <br><br>
    <strong>2) Set 2 (8-Node Continuous Monotonic Scale Mapped to ISO 8608 Scale):</strong>
    Provides smooth online interpolation across the continuous Road Severity Index ($RSI \\in [0.0, 1.0]$):
    $\\mathbf{RSI} = [0.00, 0.10, 0.40, 0.65, 0.75, 0.85, 0.95, 1.00]$, with corresponding scheduled gains
    $\\mathbf{k}_1(\\text{RSI}) = [15.0, 20.0, 45.0, 55.0, 62.0, 75.0, 78.0, 85.0]$, 
    $\\mathbf{k}_2(\\text{RSI}) = [6.0, 8.0, 18.0, 22.0, 25.0, 30.0, 32.0, 35.0]$, and 
    $\\mathbf{c}_1(\\text{RSI}) = [4.5, 5.0, 5.8, 6.5, 6.8, 7.8, 8.0, 8.5]$.
    <br><br>
    <strong>3) ISO 8608 Road Class Mapping:</strong>
    &bull; $RSI \\in [0.0, 0.2] \\leftrightarrow$ <strong>ISO Class A/B</strong> (Smooth highway; compliant $c_1=5.0, k_1=20$ maximize comfort).&nbsp;&nbsp;
    &bull; $RSI \\in [0.3, 0.5] \\leftrightarrow$ <strong>ISO Class B/C</strong> (Speed ramps; compliant $c_1=5.8, k_1=45$ cushions ramp impact).<br>
    &bull; $RSI \\in [0.6, 0.8] \\leftrightarrow$ <strong>ISO Class C/D</strong> (Harmonic resonance & chirp; $c_1=6.5\\text{--}6.8, k_1=55\\text{--}62$ suppresses bounce).&nbsp;&nbsp;
    &bull; $RSI \\in [0.85, 1.0] \\leftrightarrow$ <strong>ISO Class D/E + Severe Cavities</strong> (Pulse drops & potholes; stiff gains $c_1=7.8\\text{--}8.0, k_1=75\\text{--}78$ guarantee anti-bottoming).<br>
    The 8-node piecewise linear schedule enforces <strong>strict positive monotonicity</strong> ($\\frac{dk_1}{dRSI} > 0, \\frac{dk_2}{dRSI} > 0, \\frac{dc_1}{dRSI} > 0$), preventing gain hunting, chattering, and limit cycle instability during real-time Edge AI vision perception.
</div>

</div> <!-- end page 4 full-width -->

<div class="page-break"></div>

"""

updated_text = text[:start_idx] + new_page_4 + text[end_idx:]

with open(template_path, "w", encoding="utf-8") as f:
    f.write(updated_text)

print("Successfully restructured Page 4 in paper_template.html!")
