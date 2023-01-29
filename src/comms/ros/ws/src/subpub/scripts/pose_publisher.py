import rospy 
from geometry_msgs.msg import PoseArray, Pose

pub = rospy.Publisher("/tag_poses", PoseArray, queue_size=1)
rospy.init_node("pub_node")
r = rospy.Rate(100)
i=0
while not rospy.is_shutdown():
    pose = Pose()
    posearray = PoseArray()
    posearray.header.seq = i
    i += 1
    posearray.poses.append(pose)
    posearray.poses.append(pose)
    pub.publish(posearray)
    print(i)
    r.sleep()
