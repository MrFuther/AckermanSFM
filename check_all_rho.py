import os
import csv

folder = r"Hasil Pengujian"
files = [f for f in os.listdir(folder) if f.endswith('.csv')]

for file_name in files:
    path = os.path.join(folder, file_name)
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            headers = next(reader)
        except StopIteration:
            continue
            
        if "Steering Angle (Rho deg)" in headers:
            rho_idx = headers.index("Steering Angle (Rho deg)")
            rows = list(reader)
            rho_vals = []
            for r in rows:
                if rho_idx < len(r):
                    try:
                        rho_vals.append(float(r[rho_idx]))
                    except:
                        pass
            if rho_vals:
                unique_rho = set(rho_vals)
                print(f"{file_name}: Unique rho values = {unique_rho} (Total: {len(unique_rho)})")
                if len(unique_rho) > 1:
                    print(f"  Min: {min(rho_vals)} | Max: {max(rho_vals)}")
            else:
                print(f"{file_name}: No valid rho data")
        else:
            print(f"{file_name}: No Steering Angle (Rho deg) column")
