#!/usr/bin/env python
#Composer : Dongbin Kim
#Date :2019-06-23
#Title : Motion Capture Data streaming for ROS Melodic
#@Copyright : Dongbin Kim.
#Email : dongbin.kim@unlv.edu
#@Motive : Local Interface 10.18.81.163, Z Up, Multicast

import rospy, os
import numpy as np

# Optitrack
import socket
import optirx as rx

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from optitrack.msg import RigidBody, RigidBodyArray


class RigidBodiesPublisher(object):
  
  def __init__(self):
   
    # Setup Publishers
    mocap_pub = rospy.Publisher('/optitrack/rigid_bodies', RigidBodyArray, queue_size=3)
    mc_to_px4 = rospy.Publisher('/optitrack/mc_to_px4', PoseStamped, queue_size=3)
    # Connect to the optitrack system
    version = (2, 9, 0, 0)                     # the compatible SDK version for Motive 1.10.1 in DASL
                                               # NatNet SDK version is 2.9.0.0, not 2.10.0.0 from the motive software
    optitrack = rx.mkdatasock('239.255.42.99')  #Type the Multicast Ip Address from Motion capture system
    
    rospy.loginfo('Successfully connected to optitrack')         
    
    while not rospy.is_shutdown():
      try:
        data = optitrack.recv(rx.MAX_PACKETSIZE)
      except socket.error:
        if rospy.is_shutdown():  
          return
        else:
          rospy.logwarn('Failed to receive packet from optitrack')
          
      packet = rx.unpack(data, version=version)

      #print(packet) #this is where you get the "RAW" data

      if type(packet) is rx.SenderData:
        version = packet.natnet_version
        rospy.loginfo('NatNet version received: ' + str(version))
      if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
        # Optitrack gives the position of the centroid.
        
        
        total_msg = RigidBodyArray()
        
        for i, rigid_body in enumerate(packet.rigid_bodies):
          
          total_msg.header.stamp = rospy.Time.now()
          total_msg.header.frame_id = 'Optitrack'
         
          body_msg = RigidBody()
          body_msg.id = rigid_body.id
          body_msg.tracking_valid = rigid_body.tracking_valid
          body_msg.mrk_mean_error = rigid_body.mrk_mean_error
          
          oripose = Pose()
          oripose.position = Point(*rigid_body.position)
          oripose.orientation = Quaternion(*rigid_body.orientation)
          body_msg.pose = oripose
          
          px4_pose = PoseStamped()
          
          px4_pose.header.frame_id = ''
          px4_pose.header.stamp = rospy.Time.now()
          
          px4_pose.pose.position.x = oripose.position.x   #ENU frame for px4 position
          px4_pose.pose.position.y = oripose.position.y
          px4_pose.pose.position.z = oripose.position.z
          
          px4_pose.pose.orientation.w = oripose.orientation.w #base_link frame for px4 orientation
          px4_pose.pose.orientation.x = oripose.orientation.x
          px4_pose.pose.orientation.y = oripose.orientation.y
          px4_pose.pose.orientation.z = oripose.orientation.z
                              
          for marker in rigid_body.markers:
            body_msg.markers.append(Point(*marker))
            
          total_msg.bodies.append(body_msg)

        mocap_pub.publish(total_msg)
        mc_to_px4.publish(px4_pose)


if __name__ == '__main__':
      
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  opti_node = RigidBodiesPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)
