import csv
import os

folder = r"Hasil Pengujian"

def count_spikes(file_name, start_row, end_row):
    path = os.path.join(folder, file_name)
    spikes = 0
    total = 0
    with open(path, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)
        lane_err_idx = headers.index("Lane Error (px)")
        for idx, row in enumerate(reader):
            file_row_num = idx + 2
            if file_row_num >= start_row and file_row_num <= end_row:
                try:
                    val = float(row[lane_err_idx])
                    total += 1
                    if abs(val) > 350.0:
                        spikes += 1
                except:
                    pass
    print(f"{file_name} ({start_row}-{end_row}): {spikes} spikes out of {total} rows ({spikes/total*100:.1f}%)")

print("Spikes check:")
count_spikes("Pengujian Tanpa Deadzone.csv", 567, 759)
count_spikes("Pengujian Deadzone 15(1).csv", 806, 1037)
count_spikes("Pengujian Deadzone 30.csv", 847, 1095)
count_spikes("Pengujian Deadzone 30.csv", 1096, 1305)
count_spikes("Pengujian Deadzone 30.csv", 1306, 1541)
count_spikes("Pengujian Deadzone 30.csv", 1542, 1779)
count_spikes("Pengujian Deadzone 50.csv", 757, 1000)
count_spikes("Pengujian Deadzone 50.csv", 1001, 1240)
count_spikes("Pengujian Deadzone 50.csv", 1241, 1465)
