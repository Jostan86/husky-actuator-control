#!/usr/bin/env python3
import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32
from functools import partial

class ScanService:
    default_values = {
        1: [3, 1, 840, 50],
        2: [3, 2, 340, 50]
    }
    def __init__(self):

        self.vert_act_enabled = 0
        self.horz_act_enabled = 0
        self.vert_act_position = 0
        self.horz_act_position = 0

        self.status_received = False

        self.v

        # Actuator control publisher, sends a 4 number array, zeroth is command type, first is which actuator it's for
        # (1 for vertical actuator, 2 for horizontal actuator), second is position to move to in mm (only used for move
        # command, actuator must be enabled, positions outside of actuator bounds are ignored), fourth is speed to move
        # acuator at in mm/s (max is 50)
        # Commands:
        # 0 - disable
        # 1 - enable
        # 2 - stop
        # 3 - move
        # 4 - update
        self.pub = rospy.Publisher('actuator_control', Int16MultiArray, queue_size=10)

        # Actuator status from arduino, 5 numbers are sent, the zeroth is not used, the first and second are the
        # enable/disable status of the vertical (act1) and horizontal (act2) actuators (1 for enabled, 0 for disabled).
        # The third and fourth aren't used, they just tell whether the actuator is homed, but at this point that just
        # means the same thing as enabled/disabled.
        self.status_sub = rospy.Subscriber('actuator_status', Int16MultiArray, self.status_callback)

        # Actuator position message for vertical actuator (act1), given in number of micrometers from zero position
        self.V_pos_sub = rospy.Subscriber('V_Actuator_position', Int32, self.update_vert_position)
        # Actuator position message for horizontal actuator (act2), given in number of micrometers from zero position
        self.H_pos_sub = rospy.Subscriber('H_Actuator_position', Int32, self.update_horz_position)

        self.vert_scan = rospy.Service('vert_act_scan', Trigger, partial(self.handle_scan, 1))
        self.horz_scan = rospy.Service('horz_act_scan', Trigger, partial(self.handle_scan, 2))


    def handle_scan(self, device_id, _):
        self.status_received = False
        array_msg = Int16MultiArray()
        array_msg.data = [4, 0, 0, 0]
        self.pub.publish(array_msg)

        while self.status_received == False:
            rospy.sleep(0.1)

        data = self.default_values[device_id]

        if self.status_received and self.vert_act_enabled:
            array_msg = Int16MultiArray()
            array_msg.data = data
            self.pub.publish(array_msg)

            while self.horz_act_position < 840000:
                rospy.sleep(.1)

            array_msg = Int16MultiArray()
            array_msg.data = [3, 1, 10, 50]
            self.pub.publish(array_msg)

            while self.horz_act_position > 10000:
                rospy.sleep(.1)

            result = "Scan complete"
            success = True
            return success, result

        else:
            result = "Actuator Disabled"
            success = False
            return success, result

    def handle_horz_scan(self, req):
        ...

    def status_callback(self, status_msg):
        # Callback for updating status of actuators based on status message from arduino, the first number
        # (status_info[0] is ignored because of a bug (or confusion...). Only the second and third really matter and
        # they just tell the program whether the acuators are enabled. This is helpful to figure out when it has
        # successfully homed itself, and for determining the state of the actuators when this program is started.
        status_info = status_msg.data
        # If act1 sends a message that it's disabled
        if status_info[1] == 0:
            self.vert_act_enabled = False
        else:
        # If act1 sends a message that it's enabled
            self.vert_act_enabled = True

        if status_info[2] == 0:
            # If act2 sends a message that it's disabled, update that flag and set the enable/disable button to enable
            self.horz_act_enabled = False
        else:
            # If act2 sends a message that it's enabled, update that flag and set the enable/disable button to disable
            self.horz_act_enabled = True

        self.status_received = True

    def update_horz_position(self, horz_position_msg):
        # Callback for receiving position message for horizontal actuator (act2)
        self.horz_act_position = horz_position_msg.data


    def update_vert_position(self, vert_position_msg):
        # Callback for recieving position message for vertical actuator (act1)
        self.vert_act_position = vert_position_msg.data




if __name__ == "__main__":
    rospy.init_node('actuator_scan_server')
    scan_service = ScanService()
    rospy.spin()