path = r"Hasil Pengujian/Pengujian 1,0 dan 0,8 EMA.csv"
with open(path, 'r', newline='') as f:
    lines = f.readlines()
    print(f"Total lines: {len(lines)}")
    for idx, line in enumerate(lines):
        if "Timestamp" in line:
            print(f"Header found at line {idx+1}: {line.strip()[:100]}")
