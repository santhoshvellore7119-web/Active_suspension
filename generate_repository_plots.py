"""
GENERATE_REPOSITORY_PLOTS.PY
Generates publication-quality figures matching the repository structure and styling,
incorporating exact user requirements:
  - ONLY 3 CONTROLLERS: Optimal LQR, Fixed STSMC, Adaptive STSMC (NO passive baseline).
  - Steady-state RMS calculated over settled window (t >= 5 s) eliminating startup transient.
  - Distinct line widths and line styles (LQR dashed blue, Fixed STSMC dash-dot amber, Adaptive STSMC solid green).
  - Dynamic y-axis zooming with non-colliding limit badges (no squashing signals to +/-40).
  - High-contrast pure white background, crisp pitch-black labels and titles.
  - Multi-column clear legends on all subplots.
"""

import os
import shutil
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

# Matplotlib high-resolution publication styling
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 1.0
plt.rcParams['grid.linestyle'] = ':'
plt.rcParams['grid.alpha'] = 0.6
plt.rcParams['grid.color'] = '#bbbbbb'

fig_dir = os.path.join(os.getcwd(), 'figures')
results_dir = os.path.join(os.getcwd(), 'results')
os.makedirs(fig_dir, exist_ok=True)
os.makedirs(results_dir, exist_ok=True)

mat_path = os.path.join(results_dir, 'simulation_trajectories.mat')
if not os.path.exists(mat_path):
    raise FileNotFoundError(f"Missing {mat_path}. Please run export_sim_data.m first.")

data = sio.loadmat(mat_path, squeeze_me=True)
sim_data = data['sim_data']
road_names = data['road_names']
road_short = data['road_short']
rsi_labels = data['rsi_labels']

N_roads = len(sim_data)

# EXACT 3 CONTROLLERS REQUESTED: LQR, Fixed STSMC, Adaptive STSMC
sys_keys = ['lqr', 'fsmc', 'asmc']

styles = {
    'lqr':  {'color': '#1f77b4', 'ls': '--', 'lw': 1.8, 'label': 'Optimal LQR'},
    'fsmc': {'color': '#ff7f0e', 'ls': '-.', 'lw': 1.8, 'label': 'Fixed STSMC'},
    'asmc': {'color': '#2ca02c', 'ls': '-',  'lw': 1.8, 'label': 'Adaptive STSMC'}
}

# Collect summary metrics across all roads for 3 controllers
summary = {
    'roads': road_short,
    'rms_acc_ss': np.zeros((N_roads, 3)),
    'pk_acc':     np.zeros((N_roads, 3)),
    'rms_def_ss': np.zeros((N_roads, 3)),
    'pk_def':     np.zeros((N_roads, 3)),
    'pk_force':   np.zeros((N_roads, 3)),
    'rms_force':  np.zeros((N_roads, 3)),
    'rms_tire_ss': np.zeros((N_roads, 3))
}

print("========================================================================")
print("  GENERATING REPOSITORY PUBLICATION FIGURES (3 CONTROLLERS: LQR, FIXED STSMC, ADAPTIVE STSMC)")
print("========================================================================")

for r_idx in range(N_roads):
    case = sim_data[r_idx]
    t = case['time'].item()
    zr = case['zr'].item() * 1000.0 # mm
    t_settle = case['t_settle'].item() if 't_settle' in case.dtype.names else t[-1] * 0.50
    idx_ss = t >= t_settle

    trajs = {k: case[k].item() for k in sys_keys}

    # Extract metrics for summary
    for c_i, k in enumerate(sys_keys):
        acc = trajs[k]['acc'].item()
        defl = trajs[k]['def'].item()
        u_sig = trajs[k]['u'].item()
        tire_sig = trajs[k]['tire'].item()

        summary['rms_acc_ss'][r_idx, c_i] = trajs[k]['rms_acc_ss'].item()
        summary['pk_acc'][r_idx, c_i]     = trajs[k]['pk_acc'].item()
        summary['rms_def_ss'][r_idx, c_i] = trajs[k]['rms_def_ss'].item()
        summary['pk_def'][r_idx, c_i]     = trajs[k]['pk_def'].item()
        summary['pk_force'][r_idx, c_i]   = trajs[k]['pk_u'].item()
        summary['rms_force'][r_idx, c_i]  = np.sqrt(np.mean(u_sig[idx_ss]**2))
        summary['rms_tire_ss'][r_idx, c_i] = np.sqrt(np.mean(tire_sig[idx_ss]**2))

    # Create 3-Subplot Time-Domain Figure
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11.5, 9.0), dpi=300, sharex=True)
    fig.patch.set_facecolor('white')

    r_title = road_names[r_idx]
    r_rsi = rsi_labels[r_idx]
    fig.suptitle(f'Active Suspension Dynamic Response: {r_title}',
                 fontsize=13, fontweight='bold', color='black', y=0.98)

    # -------------------------------------------------------------
    # Subplot 1: Sprung Mass Body Acceleration (Ride Comfort)
    # -------------------------------------------------------------
    ax1.set_facecolor('white')
    ax1.grid(True)
    ax1.axvline(t_settle, color='#888888', linestyle=':', linewidth=1.0)
    
    acc_all = [trajs[k]['acc'].item() for k in sys_keys]
    max_acc = np.max([np.max(np.abs(a)) for a in acc_all])
    if max_acc < 0.05: max_acc = 0.05

    ax1.text(t_settle + 0.1, 0.82 * max_acc * 1.25,
             r'Steady-State Window ($t \geq 5$ s)', fontsize=8.5, color='#555555', fontstyle='italic')

    for k in sys_keys:
        acc = trajs[k]['acc'].item()
        ss_rms = trajs[k]['rms_acc_ss'].item()
        lbl = f"{styles[k]['label']} (SS RMS: {ss_rms:.3f} m/s² | Peak: {np.max(np.abs(acc)):.3f})"
        ax1.plot(t, acc, color=styles[k]['color'], linestyle=styles[k]['ls'],
                 linewidth=styles[k]['lw'], label=lbl)

    ax1.set_ylabel('Body Accel (m/s²)', fontsize=11, fontweight='bold', color='black')
    ax1.set_ylim([-max_acc * 1.30, max_acc * 1.30])
    ax1.legend(loc='upper right', fontsize=9.0, ncol=3, framealpha=0.95,
               facecolor='white', edgecolor='#cccccc')

    # -------------------------------------------------------------
    # Subplot 2: Suspension Travel Stroke (z_s - z_us)
    # -------------------------------------------------------------
    ax2.set_facecolor('white')
    ax2.grid(True)
    ax2.axvline(t_settle, color='#888888', linestyle=':', linewidth=1.0)

    for k in sys_keys:
        defl = trajs[k]['def'].item()
        pk = trajs[k]['pk_def'].item()
        lbl = f"{styles[k]['label']} (Peak: {pk:.1f} mm)"
        ax2.plot(t, defl, color=styles[k]['color'], linestyle=styles[k]['ls'],
                 linewidth=styles[k]['lw'], label=lbl)

    ax2.set_ylabel('Susp. Travel (mm)', fontsize=11, fontweight='bold', color='black')
    def_all = [trajs[k]['def'].item() for k in sys_keys]
    max_def = np.max([np.max(np.abs(d)) for d in def_all])

    # Dynamic Zooming: Zoom comfortably around signal
    if max_def < 25.0:
        y_lim_def = max(max_def * 1.35, 1.0)
        ax2.set_ylim([-y_lim_def, y_lim_def])
        ax2.text(0.02, 0.88, f"Rail Travel Limit: ±38.0 mm | Max Active: {max_def:.1f} mm [PASS: SAFE]",
                 transform=ax2.transAxes, fontsize=8.5, fontweight='bold', color='#1a73e8',
                 bbox=dict(boxstyle='square,pad=0.3', facecolor='#f1f8ff', edgecolor='#99c2ff'))
    else:
        y_lim_def = max(max_def * 1.15, 42.0)
        ax2.set_ylim([-y_lim_def, y_lim_def])
        ax2.axhline(38.0, color='red', linestyle='--', linewidth=1.2, label='Travel Limit (+38 mm)')
        ax2.axhline(-38.0, color='red', linestyle='--', linewidth=1.2)

    ax2.legend(loc='lower right', fontsize=9.0, ncol=3, framealpha=0.95,
               facecolor='white', edgecolor='#cccccc')

    # -------------------------------------------------------------
    # Subplot 3: Actuator Control Force Demand
    # -------------------------------------------------------------
    ax3.set_facecolor('white')
    ax3.grid(True)
    ax3.axvline(t_settle, color='#888888', linestyle=':', linewidth=1.0)

    for k in sys_keys:
        u_sig = trajs[k]['u'].item()
        pk = trajs[k]['pk_u'].item()
        lbl = f"{styles[k]['label']} (Peak: {pk:.1f} N)"
        ax3.plot(t, u_sig, color=styles[k]['color'], linestyle=styles[k]['ls'],
                 linewidth=styles[k]['lw'], label=lbl)

    ax3.set_ylabel('Control Force (N)', fontsize=11, fontweight='bold', color='black')
    ax3.set_xlabel('Time (s)', fontsize=11, fontweight='bold', color='black')
    ax3.set_xlim([0, t[-1]])

    u_all = [trajs[k]['u'].item() for k in sys_keys]
    max_u = np.max([np.max(np.abs(u)) for u in u_all])

    # Dynamic Zooming for force
    if max_u < 30.0:
        y_lim_u = max(max_u * 1.35, 1.5)
        ax3.set_ylim([-y_lim_u, y_lim_u])
        ax3.text(0.02, 0.88, f"Motor Saturation Limit: ±39.2 N | Peak Commanded: {max_u:.1f} N [PASS: SAFE]",
                 transform=ax3.transAxes, fontsize=8.5, fontweight='bold', color='#1e7e34',
                 bbox=dict(boxstyle='square,pad=0.3', facecolor='#f4fbf4', edgecolor='#94d3a2'))
    else:
        y_lim_u = max(max_u * 1.15, 44.0)
        ax3.set_ylim([-y_lim_u, y_lim_u])
        ax3.axhline(39.2, color='black', linestyle='--', linewidth=1.2, label='Saturation (±39.2 N)')
        ax3.axhline(-39.2, color='black', linestyle='--', linewidth=1.2)

    ax3.legend(loc='lower right', fontsize=9.0, ncol=3, framealpha=0.95,
               facecolor='white', edgecolor='#cccccc')

    plt.tight_layout(rect=[0, 0.02, 1, 0.96])

    # Save figure with standard repository names
    sh_name = road_short[r_idx]
    primary_fname = f"Figure{r_idx+1}_{sh_name}_Comparison.png"
    primary_path = os.path.join(fig_dir, primary_fname)
    plt.savefig(primary_path, dpi=300, facecolor='white', edgecolor='none')
    print(f"  [SAVED] {primary_fname}")

    # Aliases to match repository naming conventions exactly
    aliases = []
    if sh_name == 'Smooth':
        aliases = ['Figure1_Smooth_Comparison.png']
    elif sh_name == 'Normal':
        aliases = ['Figure2_Normal_Comparison.png']
    elif sh_name == 'Rough':
        aliases = ['Figure3_Rough_Comparison.png']
    elif sh_name == 'Pothole':
        aliases = ['Figure4_Pothole_Comparison.png']
    elif sh_name == 'Resonance':
        aliases = ['Figure5_Resonance_Comparison.png']

    for al in aliases:
        al_path = os.path.join(fig_dir, al)
        if al_path != primary_path:
            shutil.copyfile(primary_path, al_path)

    plt.close(fig)

# -------------------------------------------------------------
# Generate 4-Panel Summary Performance Dashboard (3 Controllers)
# -------------------------------------------------------------
fig_dash, axs = plt.subplots(2, 2, figsize=(13.5, 9.5), dpi=300)
fig_dash.patch.set_facecolor('white')
fig_dash.suptitle('Multi-Controller Performance Evaluation (LQR vs. Fixed STSMC vs. Adaptive STSMC)',
                  fontsize=14, fontweight='bold', color='black', y=0.98)

x_pos = np.arange(N_roads)
bar_width = 0.24
c_bars = ['#1f77b4', '#ff7f0e', '#2ca02c']
ctrl_labels = ['Optimal LQR', 'Fixed STSMC', 'Adaptive STSMC']

# Subplot 1: RMS Body Acceleration
ax = axs[0, 0]
ax.set_facecolor('white')
ax.grid(True, axis='y')
for c_i in range(3):
    ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_acc_ss'][:, c_i],
           width=bar_width, color=c_bars[c_i], label=ctrl_labels[c_i])
ax.set_title('Ride Comfort: Steady-State RMS Body Accel (Lower is Better)', fontsize=11, fontweight='bold')
ax.set_ylabel('RMS Body Accel (m/s²)', fontsize=10, fontweight='bold')
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)
ax.legend(loc='upper left', fontsize=9.0, framealpha=0.95)

# Subplot 2: RMS Suspension Deflection
ax = axs[0, 1]
ax.set_facecolor('white')
ax.grid(True, axis='y')
for c_i in range(3):
    ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_def_ss'][:, c_i],
           width=bar_width, color=c_bars[c_i], label=ctrl_labels[c_i])
ax.set_title('Suspension Stroke: Steady-State RMS Deflection (Lower is Better)', fontsize=11, fontweight='bold')
ax.set_ylabel('RMS Susp. Deflection (mm)', fontsize=10, fontweight='bold')
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)

# Subplot 3: Peak Control Force
ax = axs[1, 0]
ax.set_facecolor('white')
ax.grid(True, axis='y')
for c_i in range(3):
    ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['pk_force'][:, c_i],
           width=bar_width, color=c_bars[c_i], label=ctrl_labels[c_i])
ax.axhline(39.2, color='red', linestyle='--', linewidth=1.5, label='Actuator Limit (39.2 N)')
ax.set_title('Actuator Demand: Peak Control Force (Limit <= 39.2 N)', fontsize=11, fontweight='bold')
ax.set_ylabel('Peak Control Force (N)', fontsize=10, fontweight='bold')
ax.set_ylim([0, 44])
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)
ax.legend(loc='upper left', fontsize=9.0, framealpha=0.95)

# Subplot 4: RMS Dynamic Tire Deflection
ax = axs[1, 1]
ax.set_facecolor('white')
ax.grid(True, axis='y')
for c_i in range(3):
    ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_tire_ss'][:, c_i],
           width=bar_width, color=c_bars[c_i], label=ctrl_labels[c_i])
ax.set_title('Road Holding: Steady-State RMS Tire Deflection (Lower is Better)', fontsize=11, fontweight='bold')
ax.set_ylabel('RMS Tire Deflection (mm)', fontsize=10, fontweight='bold')
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)

plt.tight_layout(rect=[0, 0.03, 1, 0.96])

dash_fnames = [
    'Figure5_Summary_Performance_Dashboard.png'
]
for d_fn in dash_fnames:
    p = os.path.join(fig_dir, d_fn)
    plt.savefig(p, dpi=300, facecolor='white', edgecolor='none')
    print(f"  [SAVED DASHBOARD] {d_fn}")

plt.close(fig_dash)

# Also save copies to results/ for easy access
shutil.copyfile(os.path.join(fig_dir, 'Figure1_Smooth_Comparison.png'), os.path.join(results_dir, 'Figure1_Smooth_Comparison.png'))
shutil.copyfile(os.path.join(fig_dir, 'Figure2_Normal_Comparison.png'), os.path.join(results_dir, 'Figure2_Normal_Comparison.png'))
shutil.copyfile(os.path.join(fig_dir, 'Figure3_Rough_Comparison.png'), os.path.join(results_dir, 'Figure3_Rough_Comparison.png'))
shutil.copyfile(os.path.join(fig_dir, 'Figure4_Pothole_Comparison.png'), os.path.join(results_dir, 'Figure4_Pothole_Comparison.png'))
shutil.copyfile(os.path.join(fig_dir, 'Figure5_Summary_Performance_Dashboard.png'), os.path.join(results_dir, 'Figure5_Summary_Performance_Dashboard.png'))

print("\nAll 3-controller publication figures successfully placed into 'figures/' directory!\n")
