def increment_positions(start, end, step):
    if start < end:
        step = abs(step)
    else:
        step = -abs(step)
        
    positions = []
    for pos in range(start, end + step, step):
        if (step < 0 and pos < end) or (step > 0 and pos > end):
            break
        positions.append(pos)
        
    return positions

start_position = 2050
end_position = 1800
step_size = -10  # Decrease by 10 units per step

positions = increment_positions(start_position, end_position, step_size)

for position in positions:
    turn = position
    print(turn)
