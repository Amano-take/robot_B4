distance = math.sqrt((vx) ** 2 + (vy) ** 2)
    t = distance / Triangle_r_v.Robot_speed
    hx += vx * t
    hy += vy * t