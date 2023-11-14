#!/usr/bin/env python
import rospy
from morai_msgs.msg import ObjectStatusList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
import tf.transformations as tf_trans


def object_status_callback(data):
    print("---------------------------------------------------------")
    print("---------------------------------------------------------")
    data.header.frame_id = "map"
    if data.num_of_obstacle != 0 :
        for obstacle in data.obstacle_list:
            marker = Marker(
                header=data.header,
                ns=obstacle.name,
                id=obstacle.unique_id,
                type=Marker.CUBE,
                action=Marker.ADD,
                pose=Pose(position=Point(x=obstacle.position.x, y=obstacle.position.y, z=obstacle.position.z)),
                scale=Point(1.0, 1.0, 1.0),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                lifetime=rospy.Duration(1.0),
                frame_locked=False,
                points=[],
                colors=[],
                text="",
                mesh_resource="",
                mesh_use_embedded_materials=False
            )

            yaw_rad = obstacle.heading * 3.141592 / 180.0
            quaternion = tf_trans.quaternion_from_euler(0.0,0.0,yaw_rad)

            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]

            if obstacle.name == "OBJ_Kia_K7":
                marker.type = Marker.CUBE
                marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                marker.scale = Point(4.96, 2.06, 2.0)
            elif obstacle.name == "RedBarrel":
                marker.type = Marker.CYLINDER
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
                marker.scale = Point(1.0, 1.0, 1.0)

            marker_pub.publish(marker)
    else :
        print("No parking obstacle detected!")
    

def main():
    rospy.init_node('parking_object_visualizer', anonymous=True)

    rospy.Subscriber("Object_topic", ObjectStatusList, object_status_callback)

    global marker_pub
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=30)

    rospy.spin()

if __name__ == '__main__':
    main()
