
from PyQt5 import QtCore, QtWidgets, QtSerialPort
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QSpinBox
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32

class Widget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)

        rospy.init_node('PyQt_app_node', anonymous=True)

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

        label_width = 240

        # Flags for keeping track of actuator status
        self.act1_enabled = False
        self.act2_enabled = False
        # These 2 (below) are not really used
        self.act1_homed = False
        self.act2_homed = False


        self.update_button = QPushButton(text="Update Status", clicked=self.update_button_clicked)
        self.actuator1_label = QLabel('Vertical Actuator')
        self.actuator1_label.setFont(QFont('Arial', 16))
        self.actuator1_pos_label = QLabel('Position: 0mm')
        # self.actuator1_pos = QLabel('0')
        self.actuator1_ena_disa_button = QtWidgets.QPushButton(text="Enable", clicked=self.enable_disable_act1)
        self.actuator1_stop_button = QtWidgets.QPushButton(text="Stop", clicked=self.stop_act1)
        self.actuator1_move_button = QtWidgets.QPushButton(text="Move", clicked=self.move_act1)

        self.actuator1_pos_slider_label = QLabel('Position: 0mm')
        self.actuator1_pos_slider_label.setFixedWidth(label_width)

        self.actuator1_pos_slider = QSlider(Qt.Horizontal)
        self.actuator1_value_range = [0, 850]
        self.act1_num_steps = self.actuator1_value_range[1] - self.actuator1_value_range[0]
        self.actuator1_pos_slider.setMinimum(self.actuator1_value_range[0])
        self.actuator1_pos_slider.setMaximum(self.actuator1_value_range[1])
        self.actuator1_pos_slider.setTickInterval(round((self.actuator1_value_range[1] - self.actuator1_value_range[0]) / 10))
        self.actuator1_pos_slider.setTickPosition(QSlider.TicksBothSides)
        self.actuator1_pos_slider.setValue(0)
        self.actuator1_pos_slider.valueChanged.connect(self.actuator1_pos_slider_change)

        self.actuator1_vel_slider_label = QLabel('Velocity: 0mm/s')
        self.actuator1_vel_slider_label.setFixedWidth(label_width)

        self.actuator1_vel_slider = QSlider(Qt.Horizontal)
        self.actuator1_value_range = [0, 50]
        self.act1_num_steps = self.actuator1_value_range[1] - self.actuator1_value_range[0]
        self.actuator1_vel_slider.setMinimum(self.actuator1_value_range[0])
        self.actuator1_vel_slider.setMaximum(self.actuator1_value_range[1])
        self.actuator1_vel_slider.setTickInterval(
            round((self.actuator1_value_range[1] - self.actuator1_value_range[0]) / 10))
        self.actuator1_vel_slider.setTickPosition(QSlider.TicksBothSides)
        self.actuator1_vel_slider.setValue(0)
        self.actuator1_vel_slider.valueChanged.connect(self.actuator1_vel_slider_change)


        self.actuator2_label = QLabel('Horizontal Actuator')
        self.actuator2_label.setFont(QFont('Arial', 16))
        self.actuator2_pos_label = QLabel('Position: 0mm')
        self.actuator2_ena_disa_button = QtWidgets.QPushButton(text="Enable", clicked=self.enable_disable_act2)
        self.actuator2_stop_button = QtWidgets.QPushButton(text="Stop", clicked=self.stop_act2)
        self.actuator2_move_button = QtWidgets.QPushButton(text="Move", clicked=self.move_act2)

        self.actuator2_pos_slider_label = QLabel('Position: 0mm')
        self.actuator2_pos_slider_label.setFixedWidth(label_width)

        self.actuator2_pos_slider = QSlider(Qt.Horizontal)
        self.actuator2_value_range = [0, 350]
        self.act2_num_steps = self.actuator2_value_range[1] - self.actuator2_value_range[0]
        self.actuator2_pos_slider.setMinimum(self.actuator2_value_range[0])
        self.actuator2_pos_slider.setMaximum(self.actuator2_value_range[1])
        self.actuator2_pos_slider.setTickInterval(
            round((self.actuator2_value_range[1] - self.actuator2_value_range[0]) / 10))
        self.actuator2_pos_slider.setTickPosition(QSlider.TicksBothSides)
        self.actuator2_pos_slider.setValue(0)
        self.actuator2_pos_slider.valueChanged.connect(self.actuator2_pos_slider_change)

        self.actuator2_vel_slider_label = QLabel('Velocity: 0mm/s')
        self.actuator2_vel_slider_label.setFixedWidth(label_width)

        self.actuator2_vel_slider = QSlider(Qt.Horizontal)
        self.actuator2_value_range = [0, 50]
        self.act2_num_steps = self.actuator2_value_range[1] - self.actuator2_value_range[0]
        self.actuator2_vel_slider.setMinimum(self.actuator2_value_range[0])
        self.actuator2_vel_slider.setMaximum(self.actuator2_value_range[1])
        self.actuator2_vel_slider.setTickInterval(
            round((self.actuator2_value_range[1] - self.actuator2_value_range[0]) / 10))
        self.actuator2_vel_slider.setTickPosition(QSlider.TicksBothSides)
        self.actuator2_vel_slider.setValue(0)
        self.actuator2_vel_slider.valueChanged.connect(self.actuator2_vel_slider_change)

        self.output_textWindow = QtWidgets.QTextEdit(readOnly=True)

        self.overall_layout = QtWidgets.QVBoxLayout(self)

        self.actuator1_layout_info = QHBoxLayout()
        self.actuator1_layout_buttons = QHBoxLayout()
        self.actuator1_layout_position = QHBoxLayout()
        self.actuator1_layout_velocity = QHBoxLayout()

        self.actuator1_layout_info.addWidget(self.actuator1_label)
        self.actuator1_layout_info.addWidget(self.actuator1_pos_label)

        self.actuator1_layout_buttons.addWidget(self.actuator1_move_button)
        self.actuator1_layout_buttons.addWidget(self.actuator1_stop_button)
        self.actuator1_layout_buttons.addWidget(self.actuator1_ena_disa_button)

        self.actuator1_layout_position.addWidget(self.actuator1_pos_slider_label)
        self.actuator1_layout_position.addWidget(self.actuator1_pos_slider)

        self.actuator1_layout_velocity.addWidget(self.actuator1_vel_slider_label)
        self.actuator1_layout_velocity.addWidget(self.actuator1_vel_slider)


        self.actuator2_layout_info = QHBoxLayout()
        self.actuator2_layout_buttons = QHBoxLayout()
        self.actuator2_layout_position = QHBoxLayout()
        self.actuator2_layout_velocity = QHBoxLayout()

        self.actuator2_layout_info.addWidget(self.actuator2_label)
        self.actuator2_layout_info.addWidget(self.actuator2_pos_label)

        self.actuator2_layout_buttons.addWidget(self.actuator2_move_button)
        self.actuator2_layout_buttons.addWidget(self.actuator2_stop_button)
        self.actuator2_layout_buttons.addWidget(self.actuator2_ena_disa_button)



        self.actuator2_layout_position.addWidget(self.actuator2_pos_slider_label)
        self.actuator2_layout_position.addWidget(self.actuator2_pos_slider)

        self.actuator2_layout_velocity.addWidget(self.actuator2_vel_slider_label)
        self.actuator2_layout_velocity.addWidget(self.actuator2_vel_slider)

        spacing = 10
        self.overall_layout.addWidget(self.update_button)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator1_layout_info)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator1_layout_buttons)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator1_layout_position)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator1_layout_velocity)
        self.overall_layout.addSpacing(50)
        self.overall_layout.addLayout(self.actuator2_layout_info)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator2_layout_buttons)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator2_layout_position)
        self.overall_layout.addSpacing(spacing)
        self.overall_layout.addLayout(self.actuator2_layout_velocity)
        self.overall_layout.addSpacing(50)
        self.overall_layout.addWidget(self.output_textWindow)


    def status_callback(self, status_msg):
        # Callback for updating status of actuators based on status message from arduino, the first number
        # (status_info[0] is ignored because of a bug (or confusion...). Only the second and third really matter and
        # they just tell the program whether the acuators are enabled. This is helpful to figure out when it has
        # successfully homed itself, and for determining the state of the actuators when this program is started.
        status_info = status_msg.data
        # If act1 sends a message that it's disabled, update that flag and set the enable/disable button to enable
        if status_info[1] == 0:
            self.act1_enabled = False
            self.actuator1_ena_disa_button.setText("Enable")
        else:
        # If act1 sends a message that it's enabled, update that flag and set the enable/disable button to disable
            self.act1_enabled = True
            self.actuator1_ena_disa_button.setText("Disable")

        if status_info[2] == 0:
            # If act2 sends a message that it's disabled, update that flag and set the enable/disable button to enable
            self.act2_enabled = False
            self.actuator2_ena_disa_button.setText("Enable")
        else:
            # If act2 sends a message that it's enabled, update that flag and set the enable/disable button to disable
            self.act2_enabled = True
            self.actuator2_ena_disa_button.setText("Disable")

        if status_info[3] == 0:
            self.act1_homed = False
        else:
            self.act1_homed = True

        if status_info[4] == 0:
            self.act2_homed = False
        else:
            self.act2_homed = True


    def update_horz_position(self, horz_position_msg):
        # Callback for receiving position message for horizontal actuator (act2)
        position = horz_position_msg.data
        self.actuator2_pos_label.setText('Position: ' + str(position/1000) + 'mm')


    def update_vert_position(self, vert_position_msg):
        # Callback for recieving position message for vertical actuator (act1)
        position = vert_position_msg.data
        self.actuator1_pos_label.setText('Position: ' + str(position / 1000) + 'mm')

    def enable_disable_act1(self):
        # If act1 is enabled, then disable it, otherwise, enable it
        if self.act1_enabled:
            array_msg = Int16MultiArray()
            array_msg.data = [0, 1, 0, 0]
            self.pub.publish(array_msg)
            self.output_textWindow.append(str(array_msg.data))
        else:
            array_msg = Int16MultiArray()
            array_msg.data = [1, 1, 0, 0]
            self.pub.publish(array_msg)
            self.output_textWindow.append(str(array_msg.data))

    def enable_disable_act2(self):
        # If act2 is enabled, disable it, otherwise enable it
        if self.act2_enabled:
            array_msg = Int16MultiArray()
            array_msg.data = [0, 2, 0, 0]
            self.pub.publish(array_msg)
            self.output_textWindow.append(str(array_msg.data))
        else:
            array_msg = Int16MultiArray()
            array_msg.data = [1, 2, 0, 0]
            self.pub.publish(array_msg)
            self.output_textWindow.append(str(array_msg.data))

    def stop_act1(self):
        # Send stop message for act1, note this won't stop it while it's homing
        array_msg = Int16MultiArray()
        array_msg.data = [2, 1, 0, 0]
        self.pub.publish(array_msg)
        self.output_textWindow.append(str(array_msg.data))

    def stop_act2(self):
        # Send stop message for act2, note this won't stop it while it's homing
        array_msg = Int16MultiArray()
        array_msg.data = [2, 2, 0, 0]
        self.pub.publish(array_msg)
        self.output_textWindow.append(str(array_msg.data))

    def move_act1(self):
        # Send move message to act1, get the position to move to, and the speed to move at, from the sliders
        array_msg = Int16MultiArray()
        array_msg.data = [3, 1, self.actuator1_pos_slider.value(), self.actuator1_vel_slider.value()]
        self.pub.publish(array_msg)
        self.output_textWindow.append(str(array_msg.data))

    def move_act2(self):
        # Send move message to act1, get the position to move to, and the speed to move at, from the sliders
        array_msg = Int16MultiArray()
        array_msg.data = [3, 2, self.actuator2_pos_slider.value(), self.actuator2_vel_slider.value()]
        self.pub.publish(array_msg)
        self.output_textWindow.append(str(array_msg.data))

    def update_button_clicked(self):
        # Send message asking arduino to send a status update
        array_msg = Int16MultiArray()
        array_msg.data = [4, 0, 0, 0]
        self.pub.publish(array_msg)
        self.output_textWindow.append(str(array_msg.data))

    def actuator1_pos_slider_change(self):
        self.actuator1_pos_slider_label.setText('Position: ' + str(self.actuator1_pos_slider.value()) + 'mm')

    def actuator2_pos_slider_change(self):
        self.actuator2_pos_slider_label.setText('Position: ' + str(self.actuator2_pos_slider.value()) + 'mm')

    def actuator1_vel_slider_change(self):
        self.actuator1_vel_slider_label.setText('Velocity: ' + str(self.actuator1_vel_slider.value()) + 'mm/s')

    def actuator2_vel_slider_change(self):
        self.actuator2_vel_slider_label.setText('Velocity: ' + str(self.actuator2_vel_slider.value()) + 'mm/s')




if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = Widget()
    w.show()

    sys.exit(app.exec_())
    rospy.spin()