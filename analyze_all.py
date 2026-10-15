import os
import csv
import numpy as np

folder = "Hasil Pengujian"

deadzones = {
    '00': {'file': 'Pengujian Tanpa Deadzone.csv', 'val': 0, 0.3: (760, 1205), 0.5: (567, 759), 0.7: (1206, 1366)},
    '15': {'file': 'Pengujian Deadzone 15(1).csv', 'val': 15, 0.3: (2, 372), 0.5: (806, 1037), 0.7: (1038, 1214)},
    '30': {'file': 'Pengujian Deadzone 30.csv', 'val': 30, 0.3: (459, 846), 0.5: (1306, 1541), 0.7: (1985, 2180)},
    '40': {'file': 'Pengujian Deadzone 40.csv', 'val': 40, 0.3: (376, 749), 0.5: (1335, 1561), 0.7: (1758, 1934)},
    '50': {'file': 'Pengujian Deadzone 50.csv', 'val': 50, 0.3: (2, 373), 0.5: (1001, 1240), 0.7: (1641, 1820)}
}

emas_dz30 = {
    '10': {'file': 'Pengujian EMA 1.0 Deadzone 30.csv', 0.3: (2, 378), 0.5: (379, 1097), 0.7: (1098, 1895)},
    '08': {'file': 'Pengujian EMA 0,8 Deadzone 30.csv', 0.3: (2, 100), 0.5: (101, 200), 0.7: (201, 300)}, # fallback used in generate_plots
    '06': {'file': 'Pengujian EMA 0,6 Deadzone 30.csv', 0.3: (2, 2413), 0.5: (2414, 3302), 0.7: (3303, 3840)},
    '04': {'file': 'Pengujian EMA 0,4 Deadzone 30.csv', 0.3: (2, 740), 0.5: (741, 1415), 0.7: (1416, 1900)},
    '02': {'file': 'Pengujian EMA 0,2 Deadzone 30.csv', 0.3: (2, 1156), 0.5: (1157, 1824), 0.7: (1825, 2394)}
}

emas_dz40 = {
    '10': {'file': 'Pengujian 1,0 dan 0,8 EMA.csv', 0.3: (2, 372), 0.5: (735, 973), 0.7: (1346, 1525)},
    '08': {'file': 'Pengujian 1,0 dan 0,8 EMA.csv', 0.3: (373, 734), 0.5: (974, 1210), 0.7: (1700, 1884)},
    '06': {'file': 'Pengujian Deadzone 40.csv', 0.3: (376, 749), 0.5: (1335, 1561), 0.7: (1758, 1934)},
    '04': {'file': 'Pengujian 0,4 EMA.csv', 0.3: (376, 765), 0.5: (1216, 1450), 0.7: (1740, 1915)},
    '02': {'file': 'Pengujian 0,2 EMA.csv', 0.3: (2, 411), 0.5: (0, 0), 0.7: (0, 0)}
}

def get_clean_data(file_name, start_row, end_row):
    path = os.path.join(folder, file_name)
    errors = []
    rhos = []
    
    if not os.path.exists(path) or os.path.getsize(path) < 500:
        return np.array([]), np.array([])
        
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            headers = next(reader)
        except StopIteration:
            return np.array([]), np.array([])
            
        lane_err_idx = headers.index("Lane Error (px)") if "Lane Error (px)" in headers else -1
        rho_idx = headers.index("Steering Angle (Rho deg)") if "Steering Angle (Rho deg)" in headers else -1
        
        last_good_err = 0.0
        for idx, row in enumerate(reader):
            file_row_num = idx + 2
            if file_row_num >= start_row and file_row_num <= end_row:
                if lane_err_idx != -1 and lane_err_idx < len(row):
                    try:
                        err = float(row[lane_err_idx]) * (41.0 / 350.0)
                        if abs(err) > 41.0:
                            err = last_good_err
                        else:
                            last_good_err = err
                    except:
                        err = 0.0
                else:
                    err = 0.0
                
                if rho_idx != -1 and rho_idx < len(row):
                    try:
                        rho = float(row[rho_idx])
                    except:
                        rho = 180.0
                else:
                    rho = 180.0
                
                errors.append(err)
                rhos.append(rho)
            elif file_row_num > end_row:
                break
                
    return np.array(errors), np.array(rhos)

def analyze_run(fname, start, end):
    errs, rhos = get_clean_data(fname, start, end)
    if len(errs) == 0:
        return None
    dur = len(errs) * 0.033
    std_err = np.std(errs)
    std_rho = np.std(rhos)
    
    crossings = 0
    for i in range(1, len(rhos)):
        if (rhos[i-1] - 180.0) * (rhos[i] - 180.0) < 0:
            crossings += 1
    cpm = (crossings / 2.0) * (60.0 / dur) if dur > 0 else 0
    return std_err, std_rho, cpm, len(errs)

print("=== DEADZONE ANALYSES ===")
for speed in [0.3, 0.5, 0.7]:
    print(f"\nSpeed: {speed} m/s")
    print(f"{'DZ':<6} | {'Std Err (cm)':<12} | {'Std Rho (deg)':<12} | {'CPM':<8} | {'Rows':<12}")
    print("-" * 60)
    for dz_id, info in deadzones.items():
        start, end = info[speed]
        res = analyze_run(info['file'], start, end)
        if res:
            err_str = f"{res[0]:.2f}"
            rho_str = f"{res[1]:.2f}"
            cpm_str = f"{res[2]:.1f}"
            print(f"{dz_id:<6} | {err_str:<12} | {rho_str:<12} | {cpm_str:<8} | {start}-{end}")
        else:
            print(f"{dz_id:<6} | {'N/A':<12} | {'N/A':<12} | {'N/A':<8} | {start}-{end}")

print("\n=== EMA ANALYSES (DEADZONE 30) ===")
for speed in [0.3, 0.5, 0.7]:
    print(f"\nSpeed: {speed} m/s")
    print(f"{'EMA':<6} | {'Std Err (cm)':<12} | {'Std Rho (deg)':<12} | {'CPM':<8} | {'Rows':<12}")
    print("-" * 60)
    for ema_id, info in emas_dz30.items():
        start, end = info[speed]
        # check fallback for EMA 0.8
        fname = info['file']
        if ema_id == '08':
            path = os.path.join(folder, fname)
            if not os.path.exists(path) or os.path.getsize(path) < 500:
                fb_ranges = {0.3: (373, 734), 0.5: (974, 1210), 0.7: (1700, 1884)}
                start, end = fb_ranges[speed]
                fname = "Pengujian 1,0 dan 0,8 EMA.csv"
        res = analyze_run(fname, start, end)
        if res:
            err_str = f"{res[0]:.2f}"
            rho_str = f"{res[1]:.2f}"
            cpm_str = f"{res[2]:.1f}"
            print(f"{ema_id:<6} | {err_str:<12} | {rho_str:<12} | {cpm_str:<8} | {start}-{end}")
        else:
            print(f"{ema_id:<6} | {'N/A':<12} | {'N/A':<12} | {'N/A':<8} | {start}-{end}")

print("\n=== EMA ANALYSES (DEADZONE 40) ===")
for speed in [0.3, 0.5, 0.7]:
    print(f"\nSpeed: {speed} m/s")
    print(f"{'EMA':<6} | {'Std Err (cm)':<12} | {'Std Rho (deg)':<12} | {'CPM':<8} | {'Rows':<12}")
    print("-" * 60)
    for ema_id, info in emas_dz40.items():
        start, end = info[speed]
        res = analyze_run(info['file'], start, end)
        if res:
            err_str = f"{res[0]:.2f}"
            rho_str = f"{res[1]:.2f}"
            cpm_str = f"{res[2]:.1f}"
            print(f"{ema_id:<6} | {err_str:<12} | {rho_str:<12} | {cpm_str:<8} | {start}-{end}")
        else:
            print(f"{ema_id:<6} | {'N/A':<12} | {'N/A':<12} | {'N/A':<8} | {start}-{end}")
