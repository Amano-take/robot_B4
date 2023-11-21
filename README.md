# robot_B4
kanda-kenn

#How To Start
roslaunch amano_b4 ht_camera_from_bag.launch

Click the human on imageviewer's screen, the edge would become thicker and publish goal pose on the topic of **/goal_pose**.

If you want to change this, look at publish_positon.py file and change the 29line `self._pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)`.

And now, the system could go to a human who don't walk. However, it probably can't follow a human who walk.
In order to fix it, I need the rosbag file in which some people walk naturally.