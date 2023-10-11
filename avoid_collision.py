import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Avoid_collision():
    human_size = 0.25

    #t が10秒いないとかの制限をつけるのもいいかも。。
    @staticmethod
    def calculate(robotV, humanvx, humanvy, humanpositionR, humanpositiontheta):
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
                ans = phi
            
            return X / (robotV * math.cos(ans) - vx) > 10, ans
                
    
    @staticmethod
    def calculate2(robotV, humanvx, humanvy, x, y):
        r = (x**2 + y**2) ** (1/ 2) 
        theta = math.atan2(y, x)
        return Avoid_collision.calculate(robotV, humanvx, humanvy, r, theta)
            
    def ref_human_size(self, robotV, humanvx, humanvy, humanpositionR, humanpositiontheta):
        X = humanpositionR * math.cos(humanpositiontheta)
        Y = humanpositionR * math.sin(humanpositiontheta)
        hs = Avoid_collision.human_size
        four_corner = [(X + hs, Y+ hs), (X-hs, Y+hs), (X+hs, Y-hs), (X-hs, Y-hs)]
        phis = []
        ts = []
        for x, y in four_corner:
            try:
                t, phi = Avoid_collision.calculate2(robotV, humanvx, humanvy, x, y)
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
        x = -2
        y = -2
        vx = 1
        vy = 0.3
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

    def animation(self, x, y, vx, vy, angle, robotV):
        anim = 100
        sec = 3
        wx = robotV * math.cos(angle)
        wy = robotV * math.sin(angle)
        fig = plt.figure()
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


ac = Avoid_collision()
ac.illustrate()