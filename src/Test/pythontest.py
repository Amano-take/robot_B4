import time
import rospy


class Test():
    def loop():
        while True:
            time.sleep(1)
            print("yaa")

rospy.init_node("test")
try:
    Test.loop()
except:
    print("finish")