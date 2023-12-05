import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
import bisect
import heapq


class decide_angle():
    MEET_s = 10
    AVOID_s = 10
    TIME_SLICE = 1/30
    ANGLE_NUM =10
    
    def calculate(self, robotV:float, robotW:float, human_verocities:list, humanpositions:list, targetid:int):
        ac = Avoid_collision()
        candidate = ac.simulate_rotate_meet(robotV, robotW, human_verocities[targetid][0], human_verocities[targetid][1], humanpositions[targetid][0], humanpositions[targetid][1],decide_angle.ANGLE_NUM,  decide_angle.TIME_SLICE, decide_angle.MEET_s)
        ans = []
        for _, angle in candidate:
            flag = True
            for i in range(len(human_verocities)):
                if i == targetid:
                    pass
                else:
                    if not ac.angle_Simulation_rotate(robotV, robotW, human_verocities[i][0], human_verocities[i][1], humanpositions[i][0], humanpositions[i][1],angle,  decide_angle.TIME_SLICE, decide_angle.AVOID_s):
                        flag = False
            if flag:
                ans.append(angle)
        return ans
    
    def illustrate(self, min_t, max_t):
        robX, robY = 0, 0
        humanverocities =[(-1, -0.1), [0, 0.5], [-1, 0]]
        humanpositions = [(5, -2), [0, -0.88], [1.5, -1.2]]
        robotV = 1
        robotW = 100000
        fig, ax = plt.subplots(figsize=(10, 10))
        ims = []
        for i in range(int(max_t/min_t)):
            t = i * min_t
            humanpr =[]
            tempims = []
            for j in range(len(humanpositions)):
                x, y = humanpositions[j][0] - robX, humanpositions[j][1] - robY
                vx, vy = humanverocities[j]
                x += t * vx
                y += t * vy
                r = (x ** 2 + y ** 2) ** (1/2)
                theta = math.atan2(y, x)
                humanpr.append((r, theta))
                c = patches.Circle((x+robX, y+robY), Avoid_collision.human_size)
                im = ax.add_patch(c)
                tempims.append(im)
            angle = self.calculate(robotV, robotW, humanverocities, humanpr, 0)

            im = plt.text(0, 5, str(len(angle)), size="medium")
            tempims.append(im)
            if len(angle) == 0:
                pass
            else:
                angle = angle[0]
                robX += robotV * math.cos(angle) * min_t
                robY += robotV * math.sin(angle) * min_t
            im = plt.plot(robX, robY, marker=".", markersize= 10, color = "red")
            tempims.extend(im)
            im = plt.text(5, 5, str(t)[0:3], size="medium")
            tempims.append(im)
            ims.append(tempims)
        plt.xlim([-5.5, 5.5])
        plt.ylim([-5.5, 5.5])
        plt.grid()
        ani = animation.ArtistAnimation(fig, ims, interval=min_t*1000)
        plt.show()
        path_dir = "./animation_non_rotate.gif"
        ani.save(path_dir, writer="pillow")
        






#
class Avoid_collision():
    #人のサイズ（半径）
    human_size = 0.5
    meet_t = 0.75

    def hanchoku(self, vx, vy, cx, cy):
        if vx * cx + vy * cy < 0:
            return cx ** 2 + cy ** 2
        else:
            return (cx ** 2 + cy ** 2) - ((cx * vx + cy * vy)**2) / (vx ** 2 + vy ** 2)


    def calculate(self, robotV, humanvx, humanvy, humanpositionR, humanpositiontheta):
        """
        相対位置と人の速度、ロボットの最大速度を受け取って、衝突する角度を求める。（点扱い）
        """
        vx = humanvx
        vy = humanvy
        r = humanpositionR
        theta = humanpositiontheta
        X = r * math.cos(theta)
        Y = r * math.sin(theta)

        sin_theta_phi = ( Y * vx - X * vy) / ( r * robotV)
        if sin_theta_phi > 1 or sin_theta_phi < -1:
            return None
        else:
            theta_phi = math.asin(sin_theta_phi)
            phi = theta - theta_phi
            if (robotV * math.cos(phi) - vx) * X < 0:
                if phi > math.pi:
                    ans =  phi - math.pi
                else:
                    ans =  phi + math.pi
            else:
                ans = phi % (math.pi * 2)
            
            return X / (robotV * math.cos(ans) - vx) > 10, ans
                
    
    def calculate2(self, robotV, humanvx, humanvy, x, y):
        r = (x**2 + y**2) ** (1/ 2) 
        theta = math.atan2(y, x)
        return self.calculate(robotV, humanvx, humanvy, r, theta)
            
    def ref_human_size(self, robotV, humanvx, humanvy, humanpositionR, humanpositiontheta):
        X = humanpositionR * math.cos(humanpositiontheta)
        Y = humanpositionR * math.sin(humanpositiontheta)
        hs = Avoid_collision.human_size
        four_corner = [(X + hs, Y+ hs), (X-hs, Y+hs), (X+hs, Y-hs), (X-hs, Y-hs)]
        phis = []
        ts = []
        for x, y in four_corner:
            try:
                t, phi = self.calculate2(robotV, humanvx, humanvy, x, y)
                phis.append(phi)
                ts.append(t)
            except:
                continue
        if len(phis) == 0:
            return None
        
        if min(ts) < 10:
            return min(phis), max(phis)
        else:
            return None
    
    def illustrate(self):
        x = 2
        y = -2
        vx = -1
        vy = -0.1
        robotV = 1
        minphi, maxphi = self.ref_human_size(robotV, vx, vy, (x**2+ y**2) ** (1/ 2), math.atan2(y, x))
        hs = Avoid_collision.human_size
        four_corner = [(x + hs, y+ hs), (x-hs, y+hs), (x+hs, y-hs), (x-hs, y-hs)]
        
        plt.figure(figsize=(10,10))
        plt.plot(x, y, marker='.', markersize=10)
        for t, w in four_corner:
            plt.plot(t, w, marker=".", markersize=5, color="black")
        plt.plot(0, 0, marker='.', markersize = 30)    
        plt.text(0, 0.3, "robot", size=70)
        plt.text(x, y+0.3, 'human', size=70)
        plt.quiver(0, 0, robotV*math.cos(minphi), robotV*math.sin(minphi), angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.quiver(0, 0, robotV*math.cos(maxphi), robotV*math.sin(maxphi), angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.quiver(x, y, vx, vy, angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.xlim([-2.5, 2.5])
        plt.ylim([-2.5, 2.5])
        plt.grid()
        plt.show()
        self.animation(x, y, vx, vy, maxphi, robotV)
        self.animation(x, y, vx, vy, minphi, robotV)

    def test(self):
        human_verocities = [(-1, -0.1)]
        x, y = 2, -2
        human_position = [((x**2+ y**2) **(1/2), math.atan2(y, x))]
        angle_num = 36
        target_angle = 2    
        print(self.candidate_Angle(1, human_verocities, human_position, angle_num, target_angle))
                          

    def animation(self, x, y, vx, vy, angle, robotV):
        sec = 4
        anim = 30 * sec
        wx = robotV * math.cos(angle)
        wy = robotV * math.sin(angle)
        fig = plt.figure(figsize=(10, 10))
        ims = []
        ims2 = []

        for i in range(anim):
            t = i * sec / (anim) 
            hs = Avoid_collision.human_size
            four_corner = [(x + hs, y+ hs), (x-hs, y+hs), (x+hs, y-hs), (x-hs, y-hs)]
            tempims = []
            for l, w in four_corner:
                im = plt.plot(l + t*vx, w+t*vy, marker=".", markersize=5, color="black")
                tempims.extend(im)
            im = plt.plot(t*wx, t*wy, marker=".", markersize= 10, color = "black")
            tempims.extend(im)
            ims.append(tempims)
        plt.xlim([-5.5, 5.5])
        plt.ylim([-5.5, 5.5])
        plt.grid()
        ani = animation.ArtistAnimation(fig, ims, interval=sec*1000/anim)
        plt.show()

    def animationC(self, x, y, vx, vy, angle, robotV, robotW):
        sec = 4
        anim = 30 * sec
        wx = robotV * math.cos(angle)
        wy = robotV * math.sin(angle)
        fig, ax = plt.subplots(figsize=(10, 10))
        ims = []


        for i in range(anim):
            t = i * sec / (anim) 
            hs = Avoid_collision.human_size
            tempims = []
            c = patches.Circle((x + t*vx, y+t*vy), Avoid_collision.human_size)
            im = ax.add_patch(c)
            tempims.append(im)
            if t * robotW > angle:
                im = plt.plot((t- abs(angle)/robotW)*wx, (t- abs(angle)/robotW)*wy, marker=".", markersize= 10, color = "black")
            else:
                im = plt.plot(0, 0,  marker=".", markersize= 10, color = "red")
            tempims.extend(im)
            im = plt.text(5, 5, str(t)[0:3], size="medium")
            tempims.append(im)
            ims.append(tempims)
        plt.xlim([-5.5, 5.5])
        plt.ylim([-5.5, 5.5])
        plt.grid()
        ani = animation.ArtistAnimation(fig, ims, interval=sec*1000/anim)
        plt.show()
    
    def candidate_Angle(self, robotV:float, human_verocities:list, humanpositions:list, angle_num:int, target_angle: float):
        colllisionAngle = []
        for (vx, vy), (r, theta) in zip(human_verocities, humanpositions):
            ans = self.ref_human_size(robotV, vx, vy, r, theta)
            if ans is not None:
                colllisionAngle.append(ans)
        candidate = [i * math.pi * 2 / angle_num for i in range(angle_num)]
        imos = [0 for _ in range(angle_num)]
        for min, max in colllisionAngle:
            t = bisect.bisect_left(candidate, min)
            l = bisect.bisect_right(candidate, max)
            imos[t] -= 1
            imos[l] += 1
        for i in range(1, angle_num):
            imos[i] += imos[i-1]
        a = bisect.bisect_left(candidate, target_angle)
        b = a - 1
        for i in range(angle_num // 2):
            if abs(candidate[a] - target_angle) >= math.pi / 3 and abs(candidate[b] - target_angle) >= math.pi / 3:
                break
            if imos[a] == 0 and imos[b] == 0:
                if abs(candidate[a] - target_angle) <= abs(candidate[b] - target_angle):
                    return candidate[a]
                else:
                    return candidate[b]
            elif imos[a] == 0:
                return candidate[a]
            elif imos[b] == 0:
                return candidate[b]
            else:
                a = (a+1) % angle_num
                b = (b-1) % angle_num
        return None
    
    def ok_go_straight():
        pass

    def angle_Simulation(self, robotV, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t):
        """
        シミュレーションしてやっていく、、衝突しない場合True
        """
        X = humanpositionR * math.cos(humanpositiontheta)
        Y = humanpositionR * math.sin(humanpositiontheta)
        for i in range(int(max_t // min_t)):
            t = i * min_t
            posX = X + humanvx * t
            posY = Y + humanvy * t
            robX = robotV * math.cos(angle) * t
            robY = robotV * math.sin(angle) * t
            dis = (posX - robX) ** 2 + (posY - robY) ** 2
            if dis < Avoid_collision.human_size ** 2:
                return False
        hs = Avoid_collision.human_size
        return True
    
    def angle_Simulation_rotate(self, robotV, robotW, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t):
        """
        シミュレーションしてやっていく、、衝突しない場合True\n
        回転を考慮する
        """
        X = humanpositionR * math.cos(humanpositiontheta)
        Y = humanpositionR * math.sin(humanpositiontheta)
        for i in range(int(max_t // min_t)):
            t = i * min_t
            posX = X + humanvx * t
            posY = Y + humanvy * t
            if t - abs(angle)/robotW < 0:
                robX = 0
                robY = 0
            else:
                robX = robotV * math.cos(angle) * (t - abs(angle) / robotW)
                robY = robotV * math.sin(angle) * (t - abs(angle) / robotW)
            dis = (posX - robX) ** 2 + (posY - robY) ** 2
            if dis < Avoid_collision.human_size ** 2:
                return False
        return True
    
    def angle_Simulation_rotate_meet(self, robotV, robotW, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t):
        """
        衝突せずに会えるTrue、衝突するFalse、衝突しないけど会えないFalse\n
        合う時間をt
        """
        X = humanpositionR * math.cos(humanpositiontheta)
        Y = humanpositionR * math.sin(humanpositiontheta)
        humanV = ((humanvx - robotV* math.cos(angle))** 2 + (humanvy - robotV * math.sin(angle)) ** 2) ** (1/2)
        if X * (humanvx - robotV*math.cos(angle)) > 0 or Y * (humanvy - robotV * math.sin(angle)) > 0:
            return False, 0
        for i in range(int(max_t // min_t)):
            t = i * min_t
            posX = X + humanvx * t
            posY = Y + humanvy * t
            if t - abs(angle)/robotW < 0:
                robX = 0
                robY = 0
                dis = (posX - robX) ** 2 + (posY - robY) ** 2
            else:
                robX = robotV * math.cos(angle) * (t - abs(angle) / robotW)
                robY = robotV * math.sin(angle) * (t - abs(angle) / robotW)
                dis = (posX - robX) ** 2 + (posY - robY) ** 2
            if dis > Avoid_collision.human_size ** 2:
                targetX = posX + humanvx * Avoid_collision.meet_t
                targetY = posY + humanvy * Avoid_collision.meet_t
                dis0 = self.hanchoku(humanvx, humanvy, robX-targetX, robY-targetY)
                if dis0 < (Avoid_collision.human_size/2) ** 2:
                    return True, (t + (targetX**2 + targetY ** 2) **(1/2) / (humanV))
            else:
                return False, 0
        return False, 0
    
    def simulate(self, robotV, humanvx, humanvy, humanpositionR, humanpositiontheta, angle_num, min_t, max_t):
        """
        衝突する角度をreturnする
        """
        angles = [i * math.pi * 2 / angle_num for i in range(angle_num)]
        ans = []
        flag = False
        for angle in angles:
            if not self.angle_Simulation(robotV, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t):
                if not flag:
                    flag = True
                ans.append(angle)
            elif flag:
                break
        return ans
    
    def simulate_rotate(self, robotV, robotW,  humanvx, humanvy, humanpositionR, humanpositiontheta, angle_num, min_t, max_t):
        """
        衝突する角度をreturnする\n 衝突する
        回転を考慮
        """
        angles = [i * math.pi * 2 / angle_num for i in range(angle_num)]
        ans = []
        for angle in angles:
            if not self.angle_Simulation_rotate(robotV, robotW, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t):
                ans.append(angle)
        return ans

    def simulate_rotate_meet(self, robotV, robotW,  humanvx, humanvy, humanpositionR, humanpositiontheta, angle_num, min_t, max_t):
        """
        会える角度をreturnする\n
        回転を考慮
        """
        x, y = humanpositionR * math.cos(humanpositiontheta), humanpositionR * math.sin(humanpositiontheta)
        if  (x + humanvx * Avoid_collision.meet_t) ** 2 + (y - humanvy * Avoid_collision.meet_t) ** 2 <= (Avoid_collision.human_size*2/3) ** 2:
            return [(0, math.atan2(humanvy, humanvx))]
        angles = [i * math.pi * 2 / angle_num for i in range(angle_num)]
        ans = []
        for angle in angles:
            bl, t = self.angle_Simulation_rotate_meet(robotV, robotW, humanvx, humanvy, humanpositionR, humanpositiontheta, angle, min_t, max_t)
            if bl:
                ans.append((t, angle))
        ans.sort(key=lambda x: x[0])
        return ans

    
    def illust_simulation(self):
        x = 5
        y = -3
        vx = -1
        vy = 0
        robotV = 1
        robotW = 1.7
        angles = self.simulate_rotate_meet(robotV, robotW, vx, vy, (x**2+ y**2) ** (1/ 2), math.atan2(y, x), 128, 1/10, 10)
        minphi, maxphi = angles[0], angles[-1]
        hs = Avoid_collision.human_size
        
        
        fig, ax = plt.subplots(figsize=(10,10))
        c = patches.Circle((x, y), hs)
        ax.add_patch(c)
        
        plt.plot(0, 0, marker='.', markersize = 30)    
        plt.text(0, 0.3, "robot", size=70)
        plt.text(x, y+0.3, 'human', size=70)
        plt.quiver(0, 0, robotV*math.cos(minphi), robotV*math.sin(minphi), angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.quiver(0, 0, robotV*math.cos(maxphi), robotV*math.sin(maxphi), angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.quiver(x, y, vx, vy, angles='xy', scale_units='xy', scale=2, width=0.01)
        plt.xlim([-5.5, 5.5])
        plt.ylim([-5.5, 5.5])
        plt.grid()
        plt.show()
        print(len(angles))
        for i in range(0, len(angles), len(angles)//3):
            self.animationC(x, y, vx, vy, angles[i], robotV, robotW)
        """self.animationC(x, y, vx, vy, maxphi, robotV, robotW)
        self.animationC(x, y, vx, vy, minphi, robotV, robotW)"""


ac = decide_angle()
ac.illustrate(1/30, 7)