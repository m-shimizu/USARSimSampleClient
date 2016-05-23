#!/usr/bin/python

'''This file has hardcoded publishing for the table collision positions in USARSim, to help out the arm navigation planner.
  In the future, it will be far more reasonable to publish this information by reading it from the database.'''

import roslib
roslib.load_manifest('usarsim_tools')
import rospy


from arm_navigation_msgs.msg import CollisionObjectOperation, CollisionObject, Shape
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import copy

rospy.init_node("collision_add")

table1 = CollisionObject()
table2 = CollisionObject()
cShape = Shape()
cShape.type = Shape.BOX
cShape.dimensions = (3.968, 2.880, 1.088)

table1.poses.append(Pose())
table1.poses[0].position.x = 1.468
table1.poses[0].position.y = 1.108
table1.poses[0].position.z = .8
#cPose.position.x = -4.416
#cPose.position.y = 0.992
#cPose.position.z = .8
table1.poses[0].orientation.x = 0.0
table1.poses[0].orientation.y = 0.0
table1.poses[0].orientation.z = 0.0
table1.poses[0].orientation.w = 1.0

table2.poses.append(Pose())
table2.poses[0].position.x = -4.416
table2.poses[0].position.y = 0.992
table2.poses[0].position.z = .8
table2.poses[0].orientation.x = 0.0
table2.poses[0].orientation.y = 0.0
table2.poses[0].orientation.z = 0.0
table2.poses[0].orientation.w = 1.0

table1.shapes.append(cShape)
table2.shapes.append(cShape)

table1.id = "table1_collide"
table1.header.frame_id = "/odom"
table1.header.stamp = rospy.Time.now()
table1.operation.operation = CollisionObjectOperation.ADD

table2.id = "table2_collide"
table2.header.frame_id = "/odom"
table2.header.stamp = rospy.Time.now()
table2.operation.operation = CollisionObjectOperation.ADD

marker1 = Marker()
marker1.pose = table1.poses[0]
marker1.lifetime = rospy.Duration()
marker1.action = Marker.ADD
marker1.type = Marker.CUBE
marker1.scale.x = cShape.dimensions[0]
marker1.scale.y = cShape.dimensions[1]
marker1.scale.z = cShape.dimensions[2]
marker1.color.r = 1.0
marker1.color.g = 0.0
marker1.color.b = 0.0
marker1.color.a = 1.0
marker1.header.frame_id = "/odom"
marker1.header.stamp = rospy.Time.now()
marker1.id = 0

marker2 = copy.deepcopy(marker1)
marker2.pose = table2.poses[0]
marker2.id = 1

cPublisher = rospy.Publisher("/collision_object", CollisionObject)
mPublisher = rospy.Publisher("/collision_object/marker", Marker)
rospy.sleep(0.5)
cPublisher.publish(table1)
cPublisher.publish(table2)

while not rospy.is_shutdown():
  marker1.header.stamp = rospy.Time.now()
  marker2.header.stamp = rospy.Time.now()
  mPublisher.publish(marker1)
  mPublisher.publish(marker2)

