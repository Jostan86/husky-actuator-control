#include <AccelStepper.h>
#include <ezButton.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>

// Struct for linear actuator, not all of it is actually used
struct LinearActuator {
  AccelStepper stepper;
  ezButton limit_switch;
  int travel_length;
  long total_steps;
  int move_speed;
  int home_speed;
  float travel_per_step;
  int steps_per_mm;
  bool homed;
  bool enabled;
  bool moving;
};

// The Stepper pins for act1 (vertical actuator)
#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2
#define STEPPER1_ENABLE_PIN 4
#define STEPPER1_LIMIT_PIN 5

// The Stepper pins for act2 (horizontal actuator)
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 6
#define STEPPER2_ENABLE_PIN 8
#define STEPPER2_LIMIT_PIN 9

// Setup publisher for vertical actuator
std_msgs::Int32 V_pos_msg;
ros::Publisher V_Actuator_position("V_Actuator_position", &V_pos_msg);

// Setup publisher for horizontal actuator
std_msgs::Int32 H_pos_msg;
ros::Publisher H_Actuator_position("H_Actuator_position", &H_pos_msg);

// Setup publisher for status message
std_msgs::Int16MultiArray status_msg;
ros::Publisher status("actuator_status", &status_msg);

// Initiate node
ros::NodeHandle nh;

// Initiate control message
std_msgs::Int16MultiArray control_msg;


// Actuator Parameters
int actuator1_travel_length = 1000;                      //mm
int actuator2_travel_length = 350;                       //mm
int steps_per_rev = 200;                                 //steps
float travel_per_rev = 8.0;                              //mm
float travel_per_step = travel_per_rev / steps_per_rev;  //mm
int steps_per_mm = 25;                                   //steps
long total_steps_1 = actuator1_travel_length * steps_per_mm;  //steps
long total_steps_2 = actuator2_travel_length * steps_per_mm;  //steps

int home_speed = -30; //mm/s
int camera_initial_speed = 20; //mm/s

// Define steppers, switches, and actuators
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
ezButton limitSwitch1(STEPPER1_LIMIT_PIN);

LinearActuator linAc1 = { stepper1, limitSwitch1, actuator1_travel_length, total_steps_1, camera_initial_speed, home_speed, travel_per_step, steps_per_mm, false, false, false };

AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
ezButton limitSwitch2(STEPPER2_LIMIT_PIN);

LinearActuator linAc2 = { stepper2, limitSwitch2, actuator2_travel_length, total_steps_2, camera_initial_speed, home_speed, travel_per_step, steps_per_mm, false, false, false };

// Variables for controlling when position messages are sent
unsigned long last_message = 0;
// Time between position messages
int update_wait_ms = 20; //ms
bool state = 1;

// Callback function for when a control message is received, expects an array with 4 numbers, zeroth is command 
// type, first is which actuator it's for (1 for vertical actuator, 2 for horizontal actuator), second is position
// to move to in mm (only used for move command, actuator must be enabled, positions outside of actuator bounds 
// are ignored), fourth is speed to move acuator at in mm/s (max is 50). 
// Commands:
//    0 - disable
//    1 - enable
//    2 - stop
//    3 - move
//    4 - send update
void messageCb(const std_msgs::Int16MultiArray &control_msg) {
    if (control_msg.data[0] == 0) {
      if (control_msg.data[1] == 1 && linAc1.enabled == 1) {
        disableLinAc(linAc1);
      } else if (control_msg.data[1] == 2 && linAc2.enabled == 1) {
        disableLinAc(linAc2);
      }
    } else if (control_msg.data[0] == 1) {
      if (control_msg.data[1] == 1 && linAc1.enabled == 0) {
        enableLinAc(linAc1);
      } else if (control_msg.data[1] == 2 && linAc2.enabled == 0) {
        enableLinAc(linAc2);
      }
    } else if (control_msg.data[0] == 4) {
      sendStatus();
    } else if (control_msg.data[0] == 2) {
      if (control_msg.data[1] == 1) {
        delay(10);
        linAc1.stepper.move(0);
      } else if (control_msg.data[1] == 2) {
        delay(10);
        linAc2.stepper.move(0);
      }
    } else if (control_msg.data[0] == 3) {
      if (control_msg.data[1] == 1 && linAc1.homed == 1) {
        long moveToStep = steps_per_mm * control_msg.data[2];
        int speed = control_msg.data[3];
        linAc1.stepper.moveTo(moveToStep);
        if (moveToStep > linAc1.stepper.currentPosition()) {
          linAc1.move_speed = speed;
        } else {
          linAc1.move_speed = -speed;
        }
      } else if (control_msg.data[1] == 2 && linAc2.homed == 1) {
        long moveToStep = steps_per_mm * control_msg.data[2];
        int speed = control_msg.data[3];

        linAc2.stepper.moveTo(moveToStep);
        if (moveToStep > linAc2.stepper.currentPosition()) {
          linAc2.move_speed = speed;
        } else {
          linAc2.move_speed = -speed;

        }
      }
    }
}
// setup subscriber to actuator controller
ros::Subscriber<std_msgs::Int16MultiArray> control("actuator_control", &messageCb);

void setup() {
  // Initialize ROS stuff
  nh.initNode();
  nh.subscribe(control);
  nh.advertise(V_Actuator_position);
  nh.advertise(H_Actuator_position);
  nh.advertise(status);

  // Setup actuators
  linAc1.stepper.setEnablePin(STEPPER1_ENABLE_PIN);
  linAc1.stepper.setPinsInverted(false, false, true);
  disableLinAc(linAc1);
  linAc1.stepper.setMaxSpeed(1500.0); // eqivalent to 60mm/s
  linAc1.stepper.setAcceleration(1000.0);
  linAc1.limit_switch.setDebounceTime(50);

  linAc2.stepper.setEnablePin(STEPPER2_ENABLE_PIN);
  linAc2.stepper.setPinsInverted(false, false, true);
  disableLinAc(linAc2);
  linAc2.stepper.setMaxSpeed(1500.0); // eqivalent to 60mm/s
  linAc2.stepper.setAcceleration(1000.0);
  linAc2.limit_switch.setDebounceTime(50);
}

void loop() {
  // Spin ROS node
  nh.spinOnce();

  // Move actuators if needed
  if (linAc1.stepper.distanceToGo() != 0 && linAc1.enabled == 1) {

    if (linAc1.stepper.currentPosition() < linAc1.total_steps) {
      setMotorSpeed(linAc1.move_speed, linAc1.stepper);
      linAc1.stepper.runSpeed();
    } else if (linAc1.move_speed < 0) {
      setMotorSpeed(linAc1.move_speed, linAc1.stepper);
      linAc1.stepper.runSpeed();
    }
  }

  if (linAc2.stepper.distanceToGo() != 0 && linAc2.enabled == 1) {

    if (linAc2.stepper.currentPosition() < linAc2.total_steps) {
      setMotorSpeed(linAc2.move_speed, linAc2.stepper);
      linAc2.stepper.runSpeed();
    } else if (linAc2.move_speed < 0) {
      setMotorSpeed(linAc2.move_speed, linAc2.stepper);
      linAc2.stepper.runSpeed();
    }
  }

  // Send update message if needed
  if (millis() - last_message > update_wait_ms) {
    last_message = millis();
    sendPos();
  }
}

void setMotorSpeed(int move_speed, AccelStepper &stepper) {
  int motor_speed = move_speed * steps_per_mm;
  stepper.setSpeed(motor_speed);
}

void disableLinAc(LinearActuator &linAc) {
  linAc.stepper.disableOutputs();
  linAc.homed = false;
  linAc.enabled = false;
  sendStatus();
}

void enableLinAc(LinearActuator &linAc) {
  linAc.stepper.enableOutputs();
  linAc.enabled = true;
  homeActuator(linAc);
}

void homeActuator(LinearActuator &linAc) {
  delay(50);
  linAc.limit_switch.loop();
  delay(50);

  state = linAc.limit_switch.getStateRaw();

  while (state == HIGH) {
    linAc.limit_switch.loop();
    state = linAc.limit_switch.getState();

    linAc.stepper.move(10);
    setMotorSpeed(linAc.home_speed, linAc.stepper);
    linAc.stepper.runSpeed();

  }

  linAc.stepper.setCurrentPosition(0);
  linAc.stepper.move(0);
  linAc.homed = true;
  sendStatus();
}

void sendStatus() {
  int statusArray[5] = {1, linAc1.enabled, linAc2.enabled, linAc1.homed, linAc2.homed};
  status_msg.data_length = 5;

  // Not sure why it has to be done this way, but this is the only way it works.
  // The first number seems to randomly change so that can't be used for information
  status_msg.data[0] = statusArray[0];
  status_msg.data[1] = statusArray[1];
  status_msg.data[2] = statusArray[2];
  status_msg.data[3] = statusArray[3];
  status_msg.data[4] = statusArray[4];

  status.publish(&status_msg);
  nh.spinOnce();
  
}

void sendPos() {
  // Convert the current position from step number to number of micrometers
  long act1_cur_position = (1000*linAc1.stepper.currentPosition())/ (steps_per_mm);
  long act2_cur_position = (1000*linAc2.stepper.currentPosition())/ (steps_per_mm);
  
  //Setup and send message for each actuator position
  V_pos_msg.data = act1_cur_position;
  H_pos_msg.data = act2_cur_position;

  V_Actuator_position.publish( &V_pos_msg );
  H_Actuator_position.publish( &H_pos_msg );

}
