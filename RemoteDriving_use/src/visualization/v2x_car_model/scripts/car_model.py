#!/usr/bin/env python
import rospy
import math
import tf_conversions
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import Object, ObjectArray

FRAME_ID='map'
MESH_RES="package://vehicle_description/mesh/CAR_original.dae"

def publish_car_model(model_pub, data):
    id=0
    marker = Marker()
    markarray=MarkerArray()
    
    # data = Object()
    # for data in dataArray:

    marker.header.frame_id=FRAME_ID
    marker.header.stamp=rospy.Time.now()

    id=id+1
    marker.id=id
    marker.lifetime=rospy.Duration()
    marker.type=Marker.MESH_RESOURCE
    marker.mesh_resource=MESH_RES


    marker.pose.position.x=data.x_pos
    marker.pose.position.y=data.y_pos
    marker.pose.position.z=-1
    q = tf_conversions.transformations.quaternion_from_euler(math.pi/2,0,0)
    marker.pose.orientation.x=q[0]
    marker.pose.orientation.y=q[1]
    marker.pose.orientation.z=q[2]
    marker.pose.orientation.w=q[3]

    marker.color.r=1.0
    marker.color.g=1.0
    marker.color.b=1.0
    marker.color.a=1.0

    marker.scale.x=0.5
    marker.scale.y=0.5
    marker.scale.z=0.5

        # markarray.markers.append(marker)

    model_pub.publish(marker)


if __name__=='__main__':
    rospy.init_node('v2x_car_model',anonymous=True)
    model_pub=rospy.Publisher('car_model', Marker, queue_size=10)
    loop_rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        v2x_object_data = rospy.wait_for_message("/ass_object", Object, timeout=None)
        publish_car_model(model_pub, v2x_object_data)
        loop_rate.sleep()
