#! /usr/bin/env python
import roslaunch
import rospy

uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/user/catkin_ws/src/DiLoMo/launch/start_dilomo_with_controller.launch"])

launch.start()

rospy.sleep(60)
# 3 seconds later
launch.shutdown()