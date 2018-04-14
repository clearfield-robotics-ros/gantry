#import <AccelStepper.h>
#include <ros.h>
#include <ros/time.h>
#include <gantry/to_gantry_msg.h>
#include <gantry/gantry_status.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


//******** Define Variables ********//

///Motors
#define X_Motor 29
#define Y_Motor 30
extern int zeroSpeed = 90;

//Sweep speeds
extern volatile int speed_X = zeroSpeed;
extern volatile int speed_Y = zeroSpeed;
extern volatile double sweepRight = 20;
extern volatile double sweepLeft = 200;

//Encoder X
#define X_channelAPin 15
#define X_channelBPin 16
volatile bool X_channelAVal;
volatile bool X_channelBVal;
extern volatile double X_encoderTicks = 0;

//Encoder Y
#define Y_channelAPin 38
#define Y_channelBPin 39
volatile bool Y_channelAVal;
volatile bool Y_channelBVal;
extern volatile double Y_encoderTicks = 0;

//Rotational Stepper Motor
#define Enable 18
AccelStepper stepper_rot(1, 20, 17);
extern volatile double stepper_pos = 0;


//Relays
#define relay_Y 3
#define relay_X 6

//Switches
#define state_but 33
//Limit Switches
//Y axis switches
#define limSwitch1 34 //Y_max
#define limSwitch2 35 //Y_min
//X_max axis switches
#define limSwitch3 14
#define limSwitch4 7
//X_min axis switches
#define limSwitch5 8
#define limSwitch6 22

extern volatile int but_interrupt_flag = 1;

//LEDs 
#define LED1 25
#define LED2 24
#define LED3 26

//State
extern volatile int STATE = 0; //Start in a Hold state
extern volatile bool arrived = false;
extern volatile bool arrived_X = true;
extern volatile bool arrived_Y = true;
extern volatile bool arrived_R = true;
extern volatile int init_state = 0;
extern volatile bool Initialization_Flag = false;
String incoming = "";
double Y_desired = 2000;
double X_desired = 2000;
double R_desired = 0;
bool PID_switch = true;

//Gantry Params
extern volatile double X_max = 0;
extern volatile double Y_max = 0;

//ROS setup
ros::NodeHandle nh;
gantry::gantry_status gantry_status;
ros::Publisher gantryStatus("gantry_current_status", &gantry_status);

//Timing variables
long int time;
double dt_pub = 0;
long int lastPubTime = 0;
int dataRate = 100; // ms or 10Hz

//Debug mode
extern bool Debug = false;


//******** Setup for Main ********//
void setup() {
  
  if (Debug) { Serial.begin(9600);}
  
  rosSetup();

  motor_setup();

  switch_setup();

  //Relays
  pinMode(relay_X, OUTPUT);
  pinMode(relay_Y, OUTPUT);
  digitalWrite(relay_X, LOW);
  digitalWrite(relay_Y, LOW);

  //Switches
  pinMode(state_but, INPUT);
  attachInterrupt(digitalPinToInterrupt(state_but), stateChange, RISING);

  //LEDs
  pinMode(LED1, INPUT);
  digitalWrite(LED1, HIGH);
  pinMode(LED2, INPUT);
  digitalWrite(LED2, HIGH);
  pinMode(LED3, INPUT);
  digitalWrite(LED3, HIGH);
  
}

void loop() {
  
  if (!Debug) { 
    time = millis();
    dt_pub = (time - lastPubTime);
    if (dt_pub > dataRate) {
      lastPubTime = time;
      readyGantryMsg();
      gantryStatus.publish(&gantry_status);
      gantry_tf();
      nh.spinOnce();
      delay(1);
    }
  }
  
  if (STATE == 0 || STATE == 4) {
    set_speed(zeroSpeed, zeroSpeed);
  }
  
  if (STATE == 1) {
    initialize();
  }
  
  if (STATE == 2) {
    sweep();
//    cmX = X_encoderTicks*ticksToCmX;
//    cmY = Y_encoderTicks*ticksToCmY;
//    Serial.print("X   ");Serial.print(cmX);Serial.print("Y  "); Serial.println(cmY);
    PID_switch = true;
  }

  if (STATE == 3) {

    if (Debug) {
      if (PID_switch) {
      Y_desired = Y_encoderTicks;
      X_desired = X_encoderTicks;
      set_speed(zeroSpeed, zeroSpeed);
      PID_switch = false;
      }
      if (Serial.available() > 0 ) {
        incoming = Serial.readString();
        if (incoming.startsWith("Y")) {
          arrived_Y = false;
          incoming.remove(0,1);
          Y_desired = incoming.toInt();
          incoming = "";
        }
        
        if (incoming.startsWith("X")) {
          arrived_X = 0;
          incoming.remove(0,1);
          X_desired = incoming.toInt();
          incoming = "";
        }
        if (incoming.startsWith("R")) {
          incoming.remove(0,1);
          R_desired = incoming.toInt();
          if (R_desired > 1000 || R_desired < -1000) {
            Serial.println("R Value out of range");
            R_desired = 0;
          }
          stepper_rot.runToNewPosition(R_desired);
          arrived_R = true;
          incoming = "";
        }
      }
      
      Serial.print ("Y:  "); Serial.print(Y_desired); Serial.println(arrived_Y);
      Serial.print ("X:  ");Serial.print(X_desired); Serial.println(arrived_X);
      Serial.println(arrived);
    
    
      if (incoming == "") { 
        if (arrived_Y) {
          Y_desired = Y_encoderTicks; 
        }
        if (arrived_X) {
          X_desired = X_encoderTicks;
        }
      }
    }

  PIDControl(X_desired, Y_desired);
  }
  
  delay(1);
}

void stateChange() {
  if (but_interrupt_flag){
    if (digitalRead(state_but) == HIGH){
      but_interrupt_flag = 0;
      debounce(state_but);
      
     switch(STATE) {
        case 4:
          if (Debug) {Serial.println("State: Hold");}
          STATE = 0;
          break;
          
        case 0:
          if (Debug) {Serial.println("State: Initialize");}
          STATE = 1;
          init_state = 0;
          Initialization_Flag = false;
          break;
          
        case 1:
          if (Debug) {Serial.println("State: Sweep");}
          STATE = 2;
          set_speed(zeroSpeed, 10);
          break;
          
        case 2:
          if (Debug) {Serial.println("State: PID");}
          STATE = 3;
          break;

        case 3:
          if (Debug) {Serial.println("State: WAITING");}
          STATE = 4;
          break;
      } 
    }
  }
}
