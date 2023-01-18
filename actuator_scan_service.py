#!/usr/bin/env python3
import time
import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32
from functools import partial


class ActuatorStatus:
    def __init__(self, device_id, home_dist, units_per_mm=1000, friendly_id=None):
        self.device_id = device_id
        self.home_dist = home_dist
        self.units_per_mm = units_per_mm
        self.friendly_id = friendly_id or str(device_id)

        # State variables
        self.active = False
        self.current_position = 0

    @property
    def homing_array(self):
        return [3, self.device_id, self.home_dist, 50]

    def update_active(val):
        self.active = val

    def update_position(msg):
        self.current_position = msg.data / self.units_per_mm



class ScanService:
    def __init__(self):

        self.devices = {
            1: ActuatorStatus(1, 840, friendly_id='V'),      # Vertical
            2: ActuatorStatus(2, 340, friendly_id='H'),      # Horizontal
        }

        self.last_status_received = rospy.Time()

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

        for device_id in self.devices:
            rospy.Subscriber(f'{status.friendly_id}_Actuator_position', Int32, self.devices[device_id].update_position)
            rospy.Service(f'scan_actuator_{status.friendly_id}', Trigger, partial(self.handle_scan, device_id))


    def request_update(self, block=True):
        last_msg_stamp = self.last_status_received
        msg = Int16MultiArray()
        msg.data = [4, 0, 0, 0]
        self.pub.publish(msg)
        if block:
            while self.last_status_received == last_msg_stamp:
                rospy.sleep(0.1)

    def publish_array_cmd(array):
        cmd_msg = Int16MultiArray()
        cmd_msg.data = array
        self.pub.publish(cmd_msg)

    def await_device_threshold(device_id, dist, more_than=True):
        while True:
            if more_than:
                cond = self.devices[device_id].current_position > dist
            else:
                cond = self.devices[device_id].current_position < dist
            if cond:
                return
            rospy.sleep(0.1)

    def handle_scan(self, device_id, _):

        self.request_update(block=True)
        device = self.devices[device_id]
        if not device.active:
            return False, 'Device is not active'

        # Forward
        self.publish_array_cmd([3, device_id, device.home_dist, 50])
        self.await_device_threshold(device_id, device.home_dist, more_than=True)

        # Backwards
        reverse_thres = 10
        self.publish_array_cmd([3, device_id, reverse_thres, 50])
        self.await_device_threshold(device_id, reverse_thres, more_than=False)

        return True, "Scan complete"


    def status_callback(self, status_msg):
        # Callback for updating status of actuators based on status message from arduino, the first number
        # (status_info[0] is ignored because of a bug (or confusion...). Only the second and third really matter and
        # they just tell the program whether the acuators are enabled. This is helpful to figure out when it has
        # successfully homed itself, and for determining the state of the actuators when this program is started.
        status_info = status_msg.data

        # TODO: This is hardcoded, could theoretically be made more general
        self.devices[1].update_active(bool(status_info[1]))
        self.devices[2].update_active(bool(status_info[2]))

        self.last_status_received = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node('actuator_scan_server')
    scan_service = ScanService()
    rospy.spin()