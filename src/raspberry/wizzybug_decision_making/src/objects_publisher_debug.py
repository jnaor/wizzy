import rospy
import matplotlib
from wizzybug_msgs.msg import obstacle, obstacleArray




def obj1_creator():
    obj1 = obstacle()
    obj1.depth = 0.5
    obj1.height = 1.0
    obj1.width = 1.0
    obj1.x = 3
    obj1.y = 1
    obj1.yaw = 0.3

    return obj1



if __name__ == '__main__':

    rospy.init_node('objects_publisher_debug')
    obj_pub = rospy.Publisher('wizzy/obstacle_list', obstacleArray)
    obj_array = obstacleArray()
    obj_array.data.append(obj1_creator())
    obj_pub.publish(obj_array)