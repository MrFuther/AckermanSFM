import os
import csv

folder = r"Hasil Pengujian"

def check_run_errors(file_name, start_row, end_row, run_name):
    path = os.path.join(folder, file_name)
    errs = []
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        headers = next(reader)
        lane_err_idx = headers.index("Lane Error (px)")
        
        for idx, row in enumerate(reader):
            file_row_num = idx + 2
            if file_row_num >= start_row and file_row_num <= end_row:
                if lane_err_idx < len(row):
                    try:
                        errs.append(float(row[lane_err_idx]))
                    except:
                        pass
            elif file_row_num > end_row:
                break
                
    if errs:
        print(f"{file_name} - {run_name} (Rows {start_row}-{end_row}):")
        print(f"  Min Error: {min(errs):.2f} px | Max Error: {max(errs):.2f} px")
    else:
        print(f"{file_name} - {run_name}: No data")

print("Checking Tanpa Deadzone 0.5 m/s runs:")
check_run_errors("Pengujian Tanpa Deadzone.csv", 567, 759, "Run 3")

print("\nChecking Deadzone 15 0.5 m/s runs:")
check_run_errors("Pengujian Deadzone 15(1).csv", 373, 576, "Run 2")
check_run_errors("Pengujian Deadzone 15(1).csv", 577, 805, "Run 3")
check_run_errors("Pengujian Deadzone 15(1).csv", 806, 1037, "Run 4")

print("\nChecking Deadzone 30 0.5 m/s runs:")
check_run_errors("Pengujian Deadzone 30.csv", 847, 1095, "Run 3")
check_run_errors("Pengujian Deadzone 30.csv", 1096, 1305, "Run 4")
check_run_errors("Pengujian Deadzone 30.csv", 1306, 1541, "Run 5")
check_run_errors("Pengujian Deadzone 30.csv", 1542, 1779, "Run 6")

print("\nChecking Deadzone 40 0.5 m/s runs:")
check_run_errors("Pengujian Deadzone 40.csv", 1111, 1334, "Run 4")
check_run_errors("Pengujian Deadzone 40.csv", 1335, 1561, "Run 5")
