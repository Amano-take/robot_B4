import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

class Triangle_r_v():

    Robot_speed = 0.6
    Near_zero = 0.01

    def __init__(self):
        return 
    
    @staticmethod
    def dest(x, y, vx, vy):
        a = vx ** 2 + vy ** 2 - Triangle_r_v.Robot_speed ** 2
        b = 2 * (x * vx + y * vy)
        c = x ** 2 + y ** 2
        if abs(a) > Triangle_r_v.Near_zero:
            k = Triangle_r_v.solv_quadratic_equation(a, b, c)
        else:
            k = Triangle_r_v.solv_linear_equation(b, c)
        
        if k < 0:
            raise ValueError("can't reach!!")
        dest_x = x + k * vx
        dest_y = y + k * vy

        return dest_x, dest_y
    
    @staticmethod
    def solv_quadratic_equation(a, b, c):
        if (b**2 - 4*a*c) < 0:
            return -10
        
        D = (b**2 - 4*a*c) ** (1/2)
        x_1 = (-b + D) / (2 * a)
        x_2 = (-b - D) / (2 * a)
        
        if x_2 >= 0:
            return x_2
        return x_1
    
    def solv_linear_equation(b, c):
        return - c / b
    
def first_goal(robx, roby, hx, hy, vx, vy):
    try:
        x = hx - robx
        y = hy - roby
        goalx, goaly = Triangle_r_v.dest(x, y, vx, vy)
        angle = math.atan2(vy, vx) + math.pi
        return goalx+robx, goaly+roby, angle
    except Exception as e:
        return None

def setabsvec(abs, vx, vy):
    absvec = math.sqrt(vx ** 2 + vy ** 2)
    return abs * vx / absvec, abs * vy / absvec

def second_goal(robx, roby, hx, hy, vx, vy):
    gx = robx + vx
    gy = roby + vy
    distance = math.sqrt((vx) ** 2 + (vy) ** 2)
    t = distance / Triangle_r_v.Robot_speed
    hx += vx * t
    hy += vy * t
    angle = math.atan2(hy - gy, hx - gx)
    return gx, gy, angle

def third_goal(robx, roby, hx, hy, vx, vy):
    #to avoid
    gx, gy = setabsvec(Triangle_r_v.Robot_speed, robx-hx, roby - hy)
    gx += robx
    gy += roby
    t = Triangle_r_v.Robot_speed / math.sqrt(vx ** 2 + vy ** 2)
    hx += vx * t
    hy += vy * t
    angle = math.atan2(hy - gy, hx - gx)
    return gx, gy, angle

def selector(robx, roby, hx, hy, vx, vy):
    absvec = math.sqrt(vx ** 2 + vy ** 2)
    hprimex = hx + 1.2 * vx / absvec
    hprimey = hy + 1.2 * vy / absvec
    G1 = first_goal(robx, roby, hprimex, hprimey, vx, vy)
    if G1 is None:
        frag = False
    else:
        frag = True
    
    G2 = second_goal(robx, roby, hx, hy, vx, vy)
    G3 = third_goal(robx, roby, hx, hy, vx, vy)

    conangle = ((robx - hx) * vx + (roby - hy) * vy) / (math.sqrt((robx - hx) ** 2 + (roby - hy) ** 2) * math.sqrt(vx ** 2 + vy ** 2))
    distance = math.sqrt((robx - hx) ** 2 + (roby - hy) ** 2)
    if distance < 1.2:
        return G3
    elif distance < 3.5:
        return G2
    elif frag:
        return G1
    else:
        return robx, roby, None
    
def rob_movement(robx, roby, G, deltat):
    Gx, Gy, angle = G
    if Gx is None:
        return robx, roby, None
    rob_speed = Triangle_r_v.Robot_speed
    x = Gx - robx
    y = Gy - roby
    absvec = math.sqrt(x ** 2 + y ** 2)
    if absvec == 0:
        return robx, roby, None
    mult = rob_speed * deltat / absvec
    return robx + x * mult, roby + y * mult, angle

def main():
    robx = 0
    roby = 0
    hx = -5.0
    hy = 2
    vx = 1.0
    vy = -0.1
    deltat = 0.1
    ims = []
    fig, ax = plt.subplots()
    ax.set_xlim(-6, 14)
    ax.set_ylim(-7, 5)
    v_angle = math.atan2(vy, vx)
    robangle = math.pi * 2/ 3
    for i in range(200):
        temp = []
        p = ax.scatter(hx, hy, c='b', marker='o')
        temp.append(p)
        im = patches.Wedge((hx, hy), 1.2, 0, 360, color='teal', alpha=0.5)
        p = ax.add_patch(im)
        temp.append(p)
        im = patches.Wedge((hx, hy), 3.5, 0, 360, color='cyan', alpha=0.8)
        p = ax.add_patch(im)
        temp.append(p)
        p = ax.scatter(robx, roby, c='r', marker='o')
        temp.append(p)
        if i %10 == 0:
            Gx, Gy, angle = selector(robx, roby, hx, hy, vx, vy)
        absgoal = math.sqrt((Gx-robx)**2 + (Gy-roby)**2)
        if absgoal == 0:
            vecx = 0
            vecy = 0
        else:
            vecx = (Gx-robx) / absgoal
            vecy = (Gy-roby) / absgoal
        arrow = ax.quiver(hx, hy, 2 * vx/math.sqrt(vx **2 + vy**2),  2 * vy/ math.sqrt(vx**2 + vy**2), angles='xy', scale_units='xy', scale=1, color='b')
        temp.append(arrow)
        arrow = ax.quiver(robx, roby,  2 *vecx,  2 *vecy, angles='xy', scale_units='xy', scale=1, color='r')
        temp.append(arrow)
        arrow = ax.quiver(robx, roby,   2 *math.cos(robangle), 2 * math.sin(robangle), angles='xy', scale_units='xy', scale=1, color='black')
        temp.append(arrow)
    
        ims.append(temp)
        robx, roby, angle = rob_movement(robx, roby, (Gx, Gy, angle), deltat)
        # bring the robangle closer to angle
        if angle is not None:
            if angle - robangle > math.pi:
                robangle += 2 * math.pi
            elif angle - robangle < -math.pi:
                robangle -= 2 * math.pi
            dis = abs(angle - robangle)
            if angle - robangle > 0:
                robangle += min(0.08, dis)
            else:
                robangle -= min(0.08, dis)

        hx += vx * deltat
        hy += vy * deltat
    ani = animation.ArtistAnimation(fig, ims, interval=1/deltat)
    plt.show()
main()

