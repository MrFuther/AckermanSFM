import os
import csv

folder = "Hasil Pengujian"
files = [
    "Pengujian Deadzone 30.csv",
    "Pengujian Deadzone 40.csv",
    "Pengujian 1,0 dan 0,8 EMA.csv",
    "Pengujian 0,4 EMA.csv"
]

for file_name in files:
    path = os.path.join(folder, file_name)
    if not os.path.exists(path):
        print(f"File {file_name} tidak ditemukan.")
        continue
        
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            headers = next(reader)
        except StopIteration:
            continue
            
        # Cari indeks kolom FPS
        # Kolom yang tercatat di data_log_integrasi: LKA Instant FPS, LKA Avg FPS
        fps_inst_idx = -1
        fps_avg_idx = -1
        
        for idx, h in enumerate(headers):
            if "lka" in h.lower() and "fps" in h.lower():
                if "inst" in h.lower():
                    fps_inst_idx = idx
                elif "avg" in h.lower() or "rata" in h.lower():
                    fps_avg_idx = idx
                else:
                    # Fallback jika nama kolom berbeda sedikit
                    if fps_inst_idx == -1:
                        fps_inst_idx = idx
                    elif fps_avg_idx == -1:
                        fps_avg_idx = idx

        if fps_inst_idx == -1 and fps_avg_idx == -1:
            print(f"Header pada {file_name}: {headers}")
            print(f"Kolom FPS tidak ditemukan di {file_name}.\n")
            continue
            
        fps_inst_vals = []
        fps_avg_vals = []
        
        for row in reader:
            if fps_inst_idx != -1 and fps_inst_idx < len(row):
                try:
                    val = float(row[fps_inst_idx])
                    if val > 0: # Saring data ketika otonom aktif saja (LKA aktif)
                        fps_inst_vals.append(val)
                except ValueError:
                    pass
            if fps_avg_idx != -1 and fps_avg_idx < len(row):
                try:
                    val = float(row[fps_avg_idx])
                    if val > 0:
                        fps_avg_vals.append(val)
                except ValueError:
                    pass
                    
        print(f"=== Analisis FPS pada {file_name} ===")
        if fps_inst_vals:
            print(f"  Instant FPS: Min = {min(fps_inst_vals):.2f} Hz | Max = {max(fps_inst_vals):.2f} Hz | Avg = {sum(fps_inst_vals)/len(fps_inst_vals):.2f} Hz")
        else:
            print("  Instant FPS: Tidak ada data valid")
        if fps_avg_vals:
            print(f"  Average FPS: Min = {min(fps_avg_vals):.2f} Hz | Max = {max(fps_avg_vals):.2f} Hz | Avg = {sum(fps_avg_vals)/len(fps_avg_vals):.2f} Hz")
        else:
            print("  Average FPS: Tidak ada data valid")
        print()
