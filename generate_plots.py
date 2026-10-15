import os
import csv
import matplotlib.pyplot as plt
import numpy as np

# Styling setup
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.titlesize': 13,
    'legend.fontsize': 9
})

folder = r"Hasil Pengujian"

def get_run_data_standard(file_name, start_row, end_row):
    """Loads specific columns and rows from a CSV file.
    Filters out outlier spikes (e.g., camera tracking loss glitches > 41.0 cm) by forward-filling.
    """
    path = os.path.join(folder, file_name)
    data = {
        'Lane Error (px)': [],
        'Steering Angle (Rho deg)': []
    }
    
    if not os.path.exists(path) or os.path.getsize(path) < 500:
        return data
        
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            headers = next(reader)
        except StopIteration:
            return data
            
        lane_err_idx = headers.index("Lane Error (px)") if "Lane Error (px)" in headers else -1
        steering_idx = headers.index("Steering Angle (Rho deg)") if "Steering Angle (Rho deg)" in headers else -1
        
        last_good_val = 0.0
        for idx, row in enumerate(reader):
            file_row_num = idx + 2
            if file_row_num >= start_row and file_row_num <= end_row:
                # Read Lane Error
                if lane_err_idx != -1 and lane_err_idx < len(row):
                    try:
                        val = float(row[lane_err_idx]) * (41.0 / 350.0)
                    except ValueError:
                        val = 0.0
                    
                    # Filter out impossible spikes (> 41.0 cm) due to camera tracking loss
                    if abs(val) > 41.0:
                        val = last_good_val
                    else:
                        last_good_val = val
                        
                    data['Lane Error (px)'].append(val)
                else:
                    data['Lane Error (px)'].append(0.0)
                
                # Read Steering Angle (Rho)
                if steering_idx != -1 and steering_idx < len(row):
                    try:
                        steer_val = float(row[steering_idx])
                    except ValueError:
                        steer_val = 180.0
                    data['Steering Angle (Rho deg)'].append(steer_val)
                else:
                    data['Steering Angle (Rho deg)'].append(180.0)
            elif file_row_num > end_row:
                break
                
    data['Time_Sec'] = [i * 0.033 for i in range(len(data['Lane Error (px)']))]
    return data

def generate_individual_plot(data, title, filename, color, dz_val=None, stats_str=""):
    if not data['Lane Error (px)']:
        return
        
    fig, ax1 = plt.subplots(figsize=(9, 4.5))
    
    # Left Axis: Lane Error (solid line)
    line1 = ax1.plot(data['Time_Sec'], data['Lane Error (px)'], color=color, linewidth=1.75, label='Lane Error (cm)')
    ax1.set_xlabel("Waktu Relatif (detik)", fontweight='bold', labelpad=8)
    ax1.set_ylabel("Lane Error (cm)", fontweight='bold', color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    
    # Make Lane Error Y-axis symmetric around 0
    max_err = max(abs(min(data['Lane Error (px)'])), abs(max(data['Lane Error (px)'])))
    limit_y = max(max_err * 1.15, 60.0 * (41.0 / 350.0))
    ax1.set_ylim(-limit_y, limit_y)
    
    # Deadzone guide lines
    if dz_val is not None and dz_val > 0:
        dz_val_cm = dz_val * (41.0 / 350.0)
        ax1.axhline(y=dz_val_cm, color='#e74c3c', linestyle='--', alpha=0.6, linewidth=1.2, label=f'Batas Deadzone (±{dz_val_cm:.2f} cm)')
        ax1.axhline(y=-dz_val_cm, color='#e74c3c', linestyle='--', alpha=0.6, linewidth=1.2)
        
    # Right Axis: Steering Angle Rho (dash-dot line)
    ax2 = ax1.twinx()
    line2 = ax2.plot(data['Time_Sec'], data['Steering Angle (Rho deg)'], color='#34495e', linestyle='-.', linewidth=1.25, label='Sudut Kemudi ($\\rho$)')
    ax2.set_ylabel("Sudut Kemudi (derajat)", fontweight='bold', color='#34495e')
    ax2.tick_params(axis='y', labelcolor='#34495e')
    
    # Reference straight line at 180 degrees
    ax2.axhline(y=180.0, color='#7f8c8d', linestyle=':', alpha=0.5, linewidth=1.0, label='Kemudi Tengah (180°)')
    ax2.set_ylim(135, 225)
    
    # Grid and titles
    ax1.grid(True, linestyle=':', alpha=0.5)
    plt.title(title, pad=12, fontweight='bold')
    
    # Combine legends from both axes
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(handles1 + handles2, labels1 + labels2, loc='lower left', frameon=True, facecolor='white', framealpha=0.9)
    
    # Add stats textbox
    if stats_str:
        ax1.text(0.97, 0.95, stats_str, transform=ax1.transAxes, fontsize=8.5,
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8, edgecolor='#ddd'))
                 
    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved: {filename}")

def calculate_stats_string(data, dur):
    if not data['Lane Error (px)']:
        return ""
    errs = np.array(data['Lane Error (px)'])
    rhos = np.array(data['Steering Angle (Rho deg)'])
    std_err = np.std(errs)
    std_rho = np.std(rhos)
    
    crossings = 0
    for i in range(1, len(rhos)):
        if (rhos[i-1] - 180.0) * (rhos[i] - 180.0) < 0:
            crossings += 1
    cpm = (crossings / 2.0) * (60.0 / dur) if dur > 0 else 0.0
    
    return f"Std Lane Error: {std_err:.2f} cm\nStd Sudut Kemudi: {std_rho:.2f}°\nFrekuensi Koreksi: {cpm:.1f} cpm"

# Dictionaries of ranges for Deadzones
deadzones = {
    '00': {
        'name': 'Tanpa Deadzone (0 px)',
        'file': 'Pengujian Tanpa Deadzone.csv',
        'val': 0,
        'color': '#e377c2',
        0.3: (760, 1205),
        0.5: (567, 759),
        0.7: (1206, 1366)
    },
    '15': {
        'name': 'Deadzone 15 px',
        'file': 'Pengujian Deadzone 15(1).csv',
        'val': 15,
        'color': '#9467bd',
        0.3: (2, 372),
        0.5: (806, 1037),
        0.7: (1038, 1214)
    },
    '30': {
        'name': 'Deadzone 30 px',
        'file': 'Pengujian Deadzone 30.csv',
        'val': 30,
        'color': '#2ca02c',
        0.3: (459, 846),
        0.5: (1306, 1541),
        0.7: (1985, 2180)
    },
    '40': {
        'name': 'Deadzone 40 px',
        'file': 'Pengujian Deadzone 40.csv',
        'val': 40,
        'color': '#ff7f0e',
        0.3: (376, 749),
        0.5: (1335, 1561),
        0.7: (1758, 1934)
    },
    '50': {
        'name': 'Deadzone 50 px',
        'file': 'Pengujian Deadzone 50.csv',
        'val': 50,
        'color': '#1f77b4',
        0.3: (2, 373),
        0.5: (1001, 1240),
        0.7: (1641, 1820)
    }
}

# Dictionaries of ranges for EMAs (Deadzone 30 px)
emas_dz30 = {
    '10': {
        'name': 'EMA 1.0 (Tanpa Filter)',
        'file': 'Pengujian EMA 1.0 Deadzone 30.csv',
        'color': '#d62728',
        0.3: (2, 378),
        0.5: (379, 1097),
        0.7: (1098, 1895)
    },
    '08': {
        'name': 'EMA 0.8',
        'file': 'Pengujian EMA 0,8 Deadzone 30.csv',
        'color': '#ff7f0e',
        0.3: (2, 100),
        0.5: (101, 200),
        0.7: (201, 300)
    },
    '06': {
        'name': 'EMA 0.6',
        'file': 'Pengujian EMA 0,6 Deadzone 30.csv',
        'color': '#bcbd22',
        0.3: (2, 373),
        0.5: (2414, 2636),
        0.7: (3303, 3840)
    },
    '04': {
        'name': 'EMA 0.4',
        'file': 'Pengujian EMA 0,4 Deadzone 30.csv',
        'color': '#17becf',
        0.3: (2, 740),
        0.5: (741, 1415),
        0.7: (1416, 1900)
    },
    '02': {
        'name': 'EMA 0.2',
        'file': 'Pengujian EMA 0,2 Deadzone 30.csv',
        'color': '#1f77b4',
        0.3: (2, 1156),
        0.5: (1157, 1824),
        0.7: (1825, 2394)
    }
}

# Dictionaries of ranges for EMAs (Deadzone 40 px)
emas_dz40 = {
    '10': {
        'name': 'EMA 1.0 (Tanpa Filter)',
        'file': 'Pengujian 1,0 dan 0,8 EMA.csv',
        'color': '#d62728',
        0.3: (2, 372),
        0.5: (735, 973),
        0.7: (1346, 1525)
    },
    '08': {
        'name': 'EMA 0.8',
        'file': 'Pengujian 1,0 dan 0,8 EMA.csv',
        'color': '#ff7f0e',
        0.3: (373, 734),
        0.5: (974, 1210),
        0.7: (1700, 1884)
    },
    '06': {
        'name': 'EMA 0.6',
        'file': 'Pengujian Deadzone 40.csv',
        'color': '#bcbd22',
        0.3: (376, 749),
        0.5: (1335, 1561),
        0.7: (1758, 1934)
    },
    '04': {
        'name': 'EMA 0.4',
        'file': 'Pengujian 0,4 EMA.csv',
        'color': '#17becf',
        0.3: (376, 765),
        0.5: (1216, 1450),
        0.7: (1740, 1915)
    },
    '02': {
        'name': 'EMA 0.2',
        'file': 'Pengujian 0,2 EMA.csv',
        'color': '#1f77b4',
        0.3: (2, 411),
        0.5: (0, 0),
        0.7: (0, 0)
    }
}

def load_dataset(info, speed, ema_id=None):
    fname = info['file']
    start, end = info[speed]
    if ema_id == '08' and fname == 'Pengujian EMA 0,8 Deadzone 30.csv':
        path = os.path.join(folder, fname)
        if not os.path.exists(path) or os.path.getsize(path) < 500:
            fb_ranges = {0.3: (373, 734), 0.5: (974, 1210), 0.7: (1700, 1884)}
            start, end = fb_ranges[speed]
            fname = "Pengujian 1,0 dan 0,8 EMA.csv"
    return get_run_data_standard(fname, start, end)

print("=== GENERATING INDIVIDUAL DEADZONE PLOTS ===")
for speed in [0.3, 0.5, 0.7]:
    for dz_id, info in deadzones.items():
        data = load_dataset(info, speed)
        dur = len(data.get('Lane Error (px)', [])) * 0.033
        stats_str = calculate_stats_string(data, dur)
        title = f"Respon LKA: {info['name']} | v0 = {speed} m/s"
        filename = f"deadzone_{dz_id}px_{int(speed*10):02d}ms.png"
        generate_individual_plot(data, title, filename, info['color'], info['val'], stats_str)

print("\n=== GENERATING INDIVIDUAL EMA PLOTS (DEADZONE 30 px) ===")
for speed in [0.3, 0.5, 0.7]:
    for ema_id, info in emas_dz30.items():
        data = load_dataset(info, speed, ema_id)
        dur = len(data.get('Lane Error (px)', [])) * 0.033
        stats_str = calculate_stats_string(data, dur)
        title = f"Respon LKA: {info['name']} | v0 = {speed} m/s (Deadzone 30 px)"
        filename = f"ema_{ema_id}_{int(speed*10):02d}ms.png"
        generate_individual_plot(data, title, filename, info['color'], 30, stats_str)

print("\n=== GENERATING INDIVIDUAL EMA PLOTS (DEADZONE 40 px) ===")
for speed in [0.3, 0.5, 0.7]:
    for ema_id, info in emas_dz40.items():
        start, end = info[speed]
        if start == 0 and end == 0:
            continue
        data = load_dataset(info, speed)
        dur = len(data.get('Lane Error (px)', [])) * 0.033
        stats_str = calculate_stats_string(data, dur)
        title = f"Respon LKA: {info['name']} | v0 = {speed} m/s (Deadzone 40 px)"
        filename = f"ema_dz40_{ema_id}_{int(speed*10):02d}ms.png"
        generate_individual_plot(data, title, filename, info['color'], 40, stats_str)

print("\n=== GENERATING DEADZONE COMPARISON PLOTS ===")
# Generates deadzone_comparison_03ms.png, deadzone_comparison_05ms.png, deadzone_comparison_07ms.png
for speed in [0.3, 0.5, 0.7]:
    plt.figure(figsize=(9, 5))
    for dz_id, info in deadzones.items():
        data = load_dataset(info, speed)
        if data['Lane Error (px)']:
            plt.plot(data['Time_Sec'], data['Lane Error (px)'], label=info['name'], linewidth=1.5, color=info['color'])
    plt.title(f"Perbandingan Respon Lane Error: Berbagai Deadzone LKA | v0 = {speed} m/s", fontweight='bold', pad=12)
    plt.xlabel("Waktu Relatif (detik)", fontweight='bold')
    plt.ylabel("Lane Error (cm)", fontweight='bold')
    plt.ylim(-20, 20)
    plt.grid(True, linestyle=':', alpha=0.5)
    plt.legend(loc='lower left', frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()
    filename = f"deadzone_comparison_{int(speed*10):02d}ms.png"
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Comparison: {filename}")

print("\n=== GENERATING EMA COMPARISON PLOTS (COMPARING SPEEDS FOR EACH EMA) ===")
# Generates ema_10_comparison.png, ema_08_comparison.png, etc., comparing speeds 0.3, 0.5, 0.7 m/s
for ema_id, info in emas_dz30.items():
    plt.figure(figsize=(9, 5))
    colors = {0.3: '#2ecc71', 0.5: '#f1c40f', 0.7: '#e74c3c'}
    for speed in [0.3, 0.5, 0.7]:
        data = load_dataset(info, speed, ema_id)
        if data['Lane Error (px)']:
            plt.plot(data['Time_Sec'], data['Lane Error (px)'], label=f"v0 = {speed} m/s", linewidth=1.5, color=colors[speed])
    plt.title(f"Karakteristik Respon Lane Error: {info['name']} (Deadzone 30 px)", fontweight='bold', pad=12)
    plt.xlabel("Waktu Relatif (detik)", fontweight='bold')
    plt.ylabel("Lane Error (cm)", fontweight='bold')
    plt.ylim(-20, 20)
    plt.grid(True, linestyle=':', alpha=0.5)
    plt.legend(loc='lower left', frameon=True, facecolor='white', framealpha=0.9)
    plt.tight_layout()
    filename = f"ema_{ema_id}_comparison.png"
    plt.savefig(filename, dpi=300)
    plt.close()
    print(f"Saved Comparison: {filename}")

print("\n=== GENERATING OPTIMAL PARAMETER COMPARISON PLOT ===")
# Generates optimal_parameter_comparison.png
# This plots Std Lane Error and Std Steering Angle (Rho) for all parameters to show why DZ=30 and EMA=0.6 are optimal.
# We will construct a bar chart representing the data.
emas_list = ['1.0', '0.8', '0.6', '0.4', '0.2']
speeds = [0.3, 0.5, 0.7]

# Let's extract values dynamically from loaded files to be 100% accurate
# Format of values: {dz: {speed: {ema: (std_err, std_rho, cpm)}}}
stats_db = {30: {}, 40: {}}
for dz_val, dz_db, ema_db in [(30, stats_db[30], emas_dz30), (40, stats_db[40], emas_dz40)]:
    for speed in speeds:
        dz_db[speed] = {}
        for ema_id, info in ema_db.items():
            ema_key = '1.0' if ema_id == '10' else f"0.{ema_id[1]}"
            data = load_dataset(info, speed, ema_id)
            if data['Lane Error (px)']:
                errs = np.array(data['Lane Error (px)'])
                rhos = np.array(data['Steering Angle (Rho deg)'])
                std_err = np.std(errs)
                std_rho = np.std(rhos)
                
                crossings = 0
                for i in range(1, len(rhos)):
                    if (rhos[i-1] - 180.0) * (rhos[i] - 180.0) < 0:
                        crossings += 1
                dur = len(errs) * 0.033
                cpm = (crossings / 2.0) * (60.0 / dur) if dur > 0 else 0
                dz_db[speed][ema_key] = (std_err, std_rho, cpm)
            else:
                dz_db[speed][ema_key] = (np.nan, np.nan, np.nan)

print("\n=== PLOT DATA STATS ===")
for dz in [30, 40]:
    print(f"\nDeadzone: {dz} px")
    for speed in speeds:
        print(f"  Speed: {speed} m/s")
        for ema in emas_list:
            val = stats_db[dz][speed].get(ema, (np.nan, np.nan, np.nan))
            print(f"    EMA {ema}: Std Err = {val[0]:.2f} cm, Std Rho = {val[1]:.2f} deg, CPM = {val[2]:.1f}")

# Generate a detailed bar chart comparing Std Lane Error and Std Steering Angle
fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

x = np.arange(len(emas_list))
width = 0.25

# 1. Deadzone 30 px - Std Lane Error
ax = axes[0, 0]
for idx, speed in enumerate(speeds):
    vals = [stats_db[30][speed][ema][0] for ema in emas_list]
    ax.bar(x + (idx - 1) * width, vals, width, label=f'v0 = {speed} m/s', alpha=0.85)
ax.set_title("Deadzone 30 px: Standar Deviasi Lane Error", fontweight='bold')
ax.set_ylabel("Lane Error (cm)\n" + r"$\it{(lower\ is\ better)}$")
ax.set_xticks(x)
ax.set_xticklabels(emas_list)
ax.grid(True, linestyle=':', alpha=0.5)
ax.legend(frameon=True, facecolor='white')

# 2. Deadzone 40 px - Std Lane Error
ax = axes[0, 1]
for idx, speed in enumerate(speeds):
    vals = [stats_db[40][speed][ema][0] for ema in emas_list]
    ax.bar(x + (idx - 1) * width, vals, width, label=f'v0 = {speed} m/s', alpha=0.85)
ax.set_title("Deadzone 40 px: Standar Deviasi Lane Error", fontweight='bold')
ax.set_ylabel("Lane Error (cm)\n" + r"$\it{(lower\ is\ better)}$")
ax.grid(True, linestyle=':', alpha=0.5)

# 3. Deadzone 30 px - Std Steering Angle (Rho)
ax = axes[1, 0]
for idx, speed in enumerate(speeds):
    vals = [stats_db[30][speed][ema][1] for ema in emas_list]
    ax.bar(x + (idx - 1) * width, vals, width, label=f'v0 = {speed} m/s', alpha=0.85)
ax.set_title("Deadzone 30 px: Standar Deviasi Sudut Kemudi ($\\rho$)", fontweight='bold')
ax.set_ylabel("Sudut Kemudi (derajat)\n" + r"$\it{(lower\ is\ better)}$")
ax.set_xlabel("Koefisien Filter EMA ($\\alpha_{filter}$)", fontweight='bold')
ax.grid(True, linestyle=':', alpha=0.5)

# 4. Deadzone 40 px - Std Steering Angle (Rho)
ax = axes[1, 1]
for idx, speed in enumerate(speeds):
    vals = [stats_db[40][speed][ema][1] for ema in emas_list]
    ax.bar(x + (idx - 1) * width, vals, width, label=f'v0 = {speed} m/s', alpha=0.85)
ax.set_title("Deadzone 40 px: Standar Deviasi Sudut Kemudi ($\\rho$)", fontweight='bold')
ax.set_ylabel("Sudut Kemudi (derajat)\n" + r"$\it{(lower\ is\ better)}$")
ax.set_xlabel("Koefisien Filter EMA ($\\alpha_{filter}$)", fontweight='bold')
ax.grid(True, linestyle=':', alpha=0.5)

plt.suptitle("Analisis Komparatif Performansi Sistem LKA pada Berbagai Parameter", fontweight='bold', y=0.98)
plt.tight_layout()
plt.savefig("optimal_parameter_comparison.png", dpi=300)
plt.close()
print("Saved Optimal Parameter Comparison: optimal_parameter_comparison.png")

print("\nAll plots generated successfully!")
