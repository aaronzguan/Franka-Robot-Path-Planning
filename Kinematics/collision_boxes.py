#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion

from math import sin

server = None
menu_handler = MenuHandler()

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )

    print(feedback.pose.position.x)
    print(feedback.pose.position.y)
    print(feedback.pose.position.z)
    print(feedback.pose.orientation.x)
    print(feedback.pose.orientation.y)
    print(feedback.pose.orientation.z)
    print(feedback.pose.orientation.w)
    server.applyChanges()

def makeBox( msg , size):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * size[0]
    marker.scale.y = msg.scale * size[1]
    marker.scale.z = msg.scale * size[2]
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg , size):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg, size) )
    msg.controls.append( control )
    return control


#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, size, name, show_6dof = False, orientation=None):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "panda_link0"
    int_marker.pose.position = position
    if orientation is not None:
        int_marker.pose.orientation = orientation
    int_marker.scale = 1

    int_marker.name = name
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker, size)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


if __name__=="__main__":
    rospy.init_node("basic_controls")

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    
    position1 = Point(-0.04, 0, 0.05)
    size1 = [0.23, 0.2, 0.1]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position1, size1, "box1", True)

    position2 = Point(-0.009, 0, 0.15)
    size2 = [0.13, 0.12, 0.1]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position2, size2, "box2", True)

    position3 = Point(0, -0.0318194814026, 0.250960588455)
    size3 = [0.12, 0.1, 0.2]
    orientation3 = Quaternion(0.307908415794, 0, 0, 0.951416134834)
    make6DofMarker( False, InteractiveMarkerControl.NONE, position3, size3, "box3", True, orientation3)

    position4 = Point(-0.0008, 0, 0.333)
    size4 = [0.12, 0.27, 0.11]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position4, size4, "box4", True)

    position5 = Point(0, 0.042, 0.42)
    size5 = [0.12, 0.1, 0.2]
    orientation5 = Quaternion(0.307908415794, 0, 0, 0.951416134834)
    make6DofMarker( False, InteractiveMarkerControl.NONE, position5, size5, "box5", True, orientation5)

    position6 = Point(0.00687, 0, 0.53)
    size6 = [0.13, 0.12, 0.22]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position6, size6, "box6", True)

    position7 = Point(0.0745, 0, 0.653)
    size7 = [0.13, 0.23, 0.12]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position7, size7, "box7", True)

    position8 = Point(0.00422, 0.00367, 0.77)
    size8 = [0.12, 0.12, 0.22]
    make6DofMarker( False, InteractiveMarkerControl.NONE, position8, size8, "box8", True)

    position9 = Point(0, 0.0745, 0.9077)
    size9 = [0.12, 0.12, 0.2]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position9, size9, "box9", True)

    position10 = Point(0.00328, 0.0176, 1.0275)
    size10 = [0.13, 0.23, 0.12]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position10, size10, "box10", True)

    position11 = Point(0.0744, -0.0092, 1.0247)
    size11 = [0.12, 0.12, 0.2]
    make6DofMarker( True, InteractiveMarkerControl.NONE, position11, size11, "box11", True)

    position12 = Point(0.0744, -0.0092, 0.9223)
    size12 = [0.10, 0.23, 0.15]
    orientation12 = Quaternion(0.0189154259861, -0.0100818071514, 0.470246970654, 0.882274568081)
    make6DofMarker( True, InteractiveMarkerControl.NONE, position12, size12, "box12", True, orientation12)

    server.applyChanges()

    rospy.spin()

