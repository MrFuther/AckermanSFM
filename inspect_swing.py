import os
import csv

path = r"Hasil Pengujian/Pengujian 1,0 dan 0,8 EMA.csv"
with open(path, 'r', newline='') as f:
    reader = csv.reader(f)
    headers = next(reader)
    lane_err_idx = headers.index("Lane Error (px)")
    timestamp_idx = headers.index("Timestamp")
    real_vel_idx = headers.index("Real Velocity (m/s)")
    
    rows = list(reader)
    
    # Run 4 peak is around Row 1209
    print("=== Run 4 (0.5m/s, EMA 0.8) Swing profile around peak ===")
    for r_num in range(1195, 1211):
        if r_num - 2 < len(rows):
            row = rows[r_num - 2]
            print(f"Row {r_num} | Time: {row[timestamp_idx]} | Error: {float(row[lane_err_idx]):.2f} px | Velocity: {float(row[real_vel_idx]):.2f} m/s")
            
    # Run 2 peak is around Row 595
    print("\n=== Run 2 (0.3m/s, EMA 0.8) Swing profile around peak ===")
    for r_num in range(580, 605):
        if r_num - 2 < len(rows):
            row = rows[r_num - 2]
            print(f"Row {r_num} | Time: {row[timestamp_idx]} | Error: {float(row[lane_err_idx]):.2f} px | Velocity: {float(row[real_vel_idx]):.2f} m/s")
