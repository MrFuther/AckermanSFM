import os
import csv
import numpy as np

folder = r"Hasil Pengujian"

def get_clean_data(file_name, start_row, end_row):
    path = os.path.join(folder, file_name)
    errors = []
    rhos = []
    
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        headers = next(reader)
        lane_err_idx = headers.index("Lane Error (px)")
        rho_idx = headers.index("Steering Angle (Rho deg)")
        
        last_good_err = 0.0
        for idx, row in enumerate(reader):
            file_row_num = idx + 2
            if file_row_num >= start_row and file_row_num <= end_row:
                try:
                    err = float(row[lane_err_idx])
                    if abs(err) > 350.0:
                        err = last_good_err
                    else:
                        last_good_err = err
                except:
                    err = 0.0
                
                try:
                    rho = float(row[rho_idx])
                except:
                    rho = 180.0
                
                errors.append(err)
                rhos.append(rho)
            elif file_row_num > end_row:
                break
                
    return np.array(errors), np.array(rhos)

def analyze(name, fname, start, end):
    errs, rhos = get_clean_data(fname, start, end)
    dur = (end - start + 1) * 0.033
    
    std_err = np.std(errs)
    std_rho = np.std(rhos)
    
    # Let's count oscillations: how many times steering switches sign of velocity, or how many times it crosses 180.
    crossings = 0
    for i in range(1, len(rhos)):
        if (rhos[i-1] - 180.0) * (rhos[i] - 180.0) < 0:
            crossings += 1
            
    # Cycles per minute
    cpm = (crossings / 2.0) * (60.0 / dur)
    
    # Alternate way: count local extrema of rho with at least 1.0 degree deviation to filter tiny noise
    peaks = 0
    for i in range(1, len(rhos) - 1):
        if (rhos[i] > rhos[i-1] and rhos[i] > rhos[i+1] and abs(rhos[i]-180) > 1.0) or \
           (rhos[i] < rhos[i-1] and rhos[i] < rhos[i+1] and abs(rhos[i]-180) > 1.0):
            peaks += 1
    extrema_cpm = (peaks / 2.0) * (60.0 / dur)
    
    print(f"{name} ({fname} {start}-{end}):")
    print(f"  Std Lane Error: {std_err:.2f} px")
    print(f"  Std Rho: {std_rho:.2f} deg")
    print(f"  Crossings CPM: {cpm:.1f}")
    print(f"  Extrema CPM: {extrema_cpm:.1f}")

print("--- TANPA DEADZONE RUNS ---")
analyze("Tanpa Deadzone Run 3", "Pengujian Tanpa Deadzone.csv", 567, 759)

print("\n--- DEADZONE 15 RUNS ---")
analyze("Deadzone 15 Run 2", "Pengujian Deadzone 15(1).csv", 373, 576)
analyze("Deadzone 15 Run 3", "Pengujian Deadzone 15(1).csv", 577, 805)
analyze("Deadzone 15 Run 4", "Pengujian Deadzone 15(1).csv", 806, 1037)

print("\n--- DEADZONE 30 RUNS ---")
analyze("Deadzone 30 Run 3", "Pengujian Deadzone 30.csv", 847, 1095)
analyze("Deadzone 30 Run 4", "Pengujian Deadzone 30.csv", 1096, 1305)
analyze("Deadzone 30 Run 5", "Pengujian Deadzone 30.csv", 1306, 1541)
analyze("Deadzone 30 Run 6", "Pengujian Deadzone 30.csv", 1542, 1779)

print("\n--- DEADZONE 50 RUNS ---")
analyze("Deadzone 50 Run 1", "Pengujian Deadzone 50.csv", 757, 1000)
analyze("Deadzone 50 Run 2", "Pengujian Deadzone 50.csv", 1001, 1240)
analyze("Deadzone 50 Run 3", "Pengujian Deadzone 50.csv", 1241, 1465)
