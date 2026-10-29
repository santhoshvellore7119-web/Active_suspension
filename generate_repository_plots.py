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

# Ultra-High-Contrast Distinct Color Palette:
# 1. Optimal LQR:      Deep Cobalt Blue (#0044CC), dashed '--', 2.4 pt (Luminance ~0.15)
# 2. Fixed STSMC:      Vivid Crimson Red (#CC0000), dash-dot '-.', 2.4 pt (Luminance ~0.28)
# 3. Adaptive STSMC:   Deep Forest Emerald Green (#006622), solid '-', 2.8 pt (Luminance ~0.20)
# When curves coincide (e.g., Road 2 & Road 5), solid green base with red dash-dot and blue dashes
# on top guarantees ALL 3 COLORS are vivid, sharp, and 100% visible on white and in print.
styles = {
    'lqr':  {'color': '#0044CC', 'ls': '--', 'lw': 2.4, 'label': 'Optimal LQR',     'zorder': 4},
    'fsmc': {'color': '#CC0000', 'ls': '-.', 'lw': 2.4, 'label': 'Fixed STSMC',    'zorder': 3},
    'asmc': {'color': '#006622', 'ls': '-',  'lw': 2.8, 'label': 'Adaptive STSMC', 'zorder': 2}
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
    ax1.grid(True, linestyle=':', color='#cccccc', alpha=0.7)
    ax1.axvline(t_settle, color='#666666', linestyle=':', linewidth=1.2)
    
    acc_all = [trajs[k]['acc'].item() for k in sys_keys]
    max_acc = np.max([np.max(np.abs(a)) for a in acc_all])
    if max_acc < 0.05: max_acc = 0.05

    ax1.text(t_settle + 0.1, 0.82 * max_acc * 1.25,
             r'Steady-State Window ($t \geq 5$ s)', fontsize=8.5, color='#444444', fontstyle='italic')

    # Plot in z-order: Adaptive STSMC solid base first, then Fixed STSMC dash-dot, then LQR dashed
    plot_handles = {}
    for k in ['asmc', 'fsmc', 'lqr']:
        acc = trajs[k]['acc'].item()
        ss_rms = trajs[k]['rms_acc_ss'].item()
        lbl = f"{styles[k]['label']} (SS RMS: {ss_rms:.3f} m/s² | Peak: {np.max(np.abs(acc)):.3f})"
        line_obj, = ax1.plot(t, acc, color=styles[k]['color'], linestyle=styles[k]['ls'],
                             linewidth=styles[k]['lw'], label=lbl, zorder=styles[k]['zorder'])
        plot_handles[k] = line_obj

    ax1.set_ylabel('Body Accel (m/s²)', fontsize=11, fontweight='bold', color='black')
    ax1.set_ylim([-max_acc * 1.30, max_acc * 1.30])
    ordered_handles = [plot_handles['lqr'], plot_handles['fsmc'], plot_handles['asmc']]
    ax1.legend(handles=ordered_handles, loc='upper right', fontsize=9.0, ncol=3, framealpha=0.96,
               facecolor='white', edgecolor='#aaaaaa')

    # -------------------------------------------------------------
    # Subplot 2: Suspension Travel Stroke (z_s - z_us)
    # -------------------------------------------------------------
    ax2.set_facecolor('white')
    ax2.grid(True, linestyle=':', color='#cccccc', alpha=0.7)
    ax2.axvline(t_settle, color='#666666', linestyle=':', linewidth=1.2)

    plot_handles_2 = {}
    for k in ['asmc', 'fsmc', 'lqr']:
        defl = trajs[k]['def'].item()
        pk = trajs[k]['pk_def'].item()
        lbl = f"{styles[k]['label']} (Peak: {pk:.1f} mm)"
        line_obj, = ax2.plot(t, defl, color=styles[k]['color'], linestyle=styles[k]['ls'],
                             linewidth=styles[k]['lw'], label=lbl, zorder=styles[k]['zorder'])
        plot_handles_2[k] = line_obj

    ax2.set_ylabel('Susp. Travel (mm)', fontsize=11, fontweight='bold', color='black')
    def_all = [trajs[k]['def'].item() for k in sys_keys]
    max_def = np.max([np.max(np.abs(d)) for d in def_all])

    # Dynamic Zooming: Zoom comfortably around signal
    ordered_handles_2 = [plot_handles_2['lqr'], plot_handles_2['fsmc'], plot_handles_2['asmc']]
    if max_def < 25.0:
        y_lim_def = max(max_def * 1.35, 1.0)
        ax2.set_ylim([-y_lim_def, y_lim_def])
        ax2.text(0.02, 0.88, f"Rail Travel Limit: ±38.0 mm | Max Active: {max_def:.1f} mm [PASS: SAFE]",
                 transform=ax2.transAxes, fontsize=8.5, fontweight='bold', color='#0044CC',
                 bbox=dict(boxstyle='square,pad=0.3', facecolor='#f0f4ff', edgecolor='#99b3ff'))
    else:
        y_lim_def = max(max_def * 1.15, 42.0)
        ax2.set_ylim([-y_lim_def, y_lim_def])
        h_lim = ax2.axhline(38.0, color='#333333', linestyle='--', linewidth=1.3, label='Travel Limit (±38 mm)')
        ax2.axhline(-38.0, color='#333333', linestyle='--', linewidth=1.3)
        ordered_handles_2.append(h_lim)

    ax2.legend(handles=ordered_handles_2, loc='lower right', fontsize=9.0, ncol=3, framealpha=0.96,
               facecolor='white', edgecolor='#aaaaaa')

    # -------------------------------------------------------------
    # Subplot 3: Actuator Control Force Demand
    # -------------------------------------------------------------
    ax3.set_facecolor('white')
    ax3.grid(True, linestyle=':', color='#cccccc', alpha=0.7)
    ax3.axvline(t_settle, color='#666666', linestyle=':', linewidth=1.2)

    plot_handles_3 = {}
    for k in ['asmc', 'fsmc', 'lqr']:
        u_sig = trajs[k]['u'].item()
        pk = trajs[k]['pk_u'].item()
        lbl = f"{styles[k]['label']} (Peak: {pk:.1f} N)"
        line_obj, = ax3.plot(t, u_sig, color=styles[k]['color'], linestyle=styles[k]['ls'],
                             linewidth=styles[k]['lw'], label=lbl, zorder=styles[k]['zorder'])
        plot_handles_3[k] = line_obj

    ax3.set_ylabel('Control Force (N)', fontsize=11, fontweight='bold', color='black')
    ax3.set_xlabel('Time (s)', fontsize=11, fontweight='bold', color='black')
    ax3.set_xlim([0, t[-1]])

    u_all = [trajs[k]['u'].item() for k in sys_keys]
    max_u = np.max([np.max(np.abs(u)) for u in u_all])

    # Dynamic Zooming for force
    ordered_handles_3 = [plot_handles_3['lqr'], plot_handles_3['fsmc'], plot_handles_3['asmc']]
    if max_u < 30.0:
        y_lim_u = max(max_u * 1.35, 1.5)
        ax3.set_ylim([-y_lim_u, y_lim_u])
        ax3.text(0.02, 0.88, f"Motor Saturation Limit: ±39.2 N | Peak Commanded: {max_u:.1f} N [PASS: SAFE]",
                 transform=ax3.transAxes, fontsize=8.5, fontweight='bold', color='#006622',
                 bbox=dict(boxstyle='square,pad=0.3', facecolor='#f0f8f2', edgecolor='#80c294'))
    else:
        y_lim_u = max(max_u * 1.15, 44.0)
        ax3.set_ylim([-y_lim_u, y_lim_u])
        h_sat = ax3.axhline(39.2, color='#333333', linestyle='--', linewidth=1.3, label='Saturation (±39.2 N)')
        ax3.axhline(-39.2, color='#333333', linestyle='--', linewidth=1.3)
        ordered_handles_3.append(h_sat)

    ax3.legend(handles=ordered_handles_3, loc='lower right', fontsize=9.0, ncol=3, framealpha=0.96,
               facecolor='white', edgecolor='#aaaaaa')

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
c_bars = ['#0044CC', '#CC0000', '#006622']
edge_colors = ['#002266', '#660000', '#003311']
ctrl_labels = ['Optimal LQR', 'Fixed STSMC', 'Adaptive STSMC']

# Subplot 1: RMS Body Acceleration
ax = axs[0, 0]
ax.set_facecolor('white')
ax.grid(True, axis='y', linestyle=':', color='#cccccc', alpha=0.7)
for c_i in range(3):
    bars = ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_acc_ss'][:, c_i],
                  width=bar_width, color=c_bars[c_i], edgecolor=edge_colors[c_i], linewidth=1.2, label=ctrl_labels[c_i])
    for bar in bars:
        h = bar.get_height()
        if h > 0.001:
            ax.text(bar.get_x() + bar.get_width()/2.0, h + 0.015, f'{h:.3f}',
                    ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#111111', rotation=25)
ax.set_title('Ride Comfort: Steady-State RMS Body Accel (Lower is Better)', fontsize=11, fontweight='bold')
ax.set_ylabel('RMS Body Accel (m/s²)', fontsize=10, fontweight='bold')
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)
ax.legend(loc='upper left', fontsize=9.0, framealpha=0.96, facecolor='white', edgecolor='#aaaaaa')

# Subplot 2: RMS Suspension Deflection
ax = axs[0, 1]
ax.set_facecolor('white')
ax.grid(True, axis='y', linestyle=':', color='#cccccc', alpha=0.7)
for c_i in range(3):
    bars = ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_def_ss'][:, c_i],
                  width=bar_width, color=c_bars[c_i], edgecolor=edge_colors[c_i], linewidth=1.2, label=ctrl_labels[c_i])
    for bar in bars:
        h = bar.get_height()
        if h > 0.1:
            ax.text(bar.get_x() + bar.get_width()/2.0, h + 0.15, f'{h:.2f}',
                    ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#111111', rotation=25)
ax.set_title('Suspension Stroke: Steady-State RMS Deflection (Lower is Better)', fontsize=11, fontweight='bold')
ax.set_ylabel('RMS Susp. Deflection (mm)', fontsize=10, fontweight='bold')
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)

# Subplot 3: Peak Control Force
ax = axs[1, 0]
ax.set_facecolor('white')
ax.grid(True, axis='y', linestyle=':', color='#cccccc', alpha=0.7)
for c_i in range(3):
    bars = ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['pk_force'][:, c_i],
                  width=bar_width, color=c_bars[c_i], edgecolor=edge_colors[c_i], linewidth=1.2, label=ctrl_labels[c_i])
    for bar in bars:
        h = bar.get_height()
        if h > 0.5:
            ax.text(bar.get_x() + bar.get_width()/2.0, h + 0.6, f'{h:.1f}',
                    ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#111111', rotation=25)
ax.axhline(39.2, color='#222222', linestyle='--', linewidth=1.5, label='Actuator Limit (39.2 N)')
ax.set_title('Actuator Demand: Peak Control Force (Limit <= 39.2 N)', fontsize=11, fontweight='bold')
ax.set_ylabel('Peak Control Force (N)', fontsize=10, fontweight='bold')
ax.set_ylim([0, 46])
ax.set_xticks(x_pos)
ax.set_xticklabels([f"{road_short[i]}\n({rsi_labels[i]})" for i in range(N_roads)], fontsize=8.5)
ax.legend(loc='upper left', fontsize=9.0, framealpha=0.96, facecolor='white', edgecolor='#aaaaaa')

# Subplot 4: RMS Dynamic Tire Deflection
ax = axs[1, 1]
ax.set_facecolor('white')
ax.grid(True, axis='y', linestyle=':', color='#cccccc', alpha=0.7)
for c_i in range(3):
    bars = ax.bar(x_pos + (c_i - 1.0) * bar_width, summary['rms_tire_ss'][:, c_i],
                  width=bar_width, color=c_bars[c_i], edgecolor=edge_colors[c_i], linewidth=1.2, label=ctrl_labels[c_i])
    for bar in bars:
        h = bar.get_height()
        if h > 0.1:
            ax.text(bar.get_x() + bar.get_width()/2.0, h + 0.08, f'{h:.2f}',
                    ha='center', va='bottom', fontsize=7.0, fontweight='bold', color='#111111', rotation=25)
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
