//setup subscriber
ros::Subscriber<gantry::to_gantry_msg> sub("gantry_cmd", messageCb);

double cmToTicksY = 5.61121;  //52.8562;
double ticksToCmY = 1/cmToTicksY;

double cmToTicksX = 81.68966; //805.53;
double ticksToCmX = 1/cmToTicksX;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char sensor_head[] = "/sensor_head";
char gantry_frame[] = "/gantry";

void rosSetup() {
  
  delay(1000);
  
  if (!Debug) {
    nh.getHardware()->setBaud(115200); //115200
    nh.initNode();
    broadcaster.init(nh);
  }
  
  nh.advertise(gantryStatus);
  nh.subscribe(sub);
  
}


/**************************************************************************/
/*
    PUBLISHER
    Assign values to gantry_status messages
   * gantry_status.state <== STATE
   * gantry_status.sweep_speed <== speed_Y
   * gantry_status.x <==  X_encoderTicks
   * gantry_status.y <==  Y_encoderTicks
   * gantry_status.yaw <== 
   * gantry_status.probe_angle <== 
   * gantry_status.position_reached <== arrived
*/
/**************************************************************************/
void readyGantryMsg()
{
   gantry_status.state = STATE;
   gantry_status.sweep_speed = speed_Y - 90;
   gantry_status.x =  X_encoderTicks * ticksToCmX ;
   gantry_status.y =  Y_encoderTicks * ticksToCmY;
   gantry_status.yaw = -3.14159 * stepper_pos/2000;
   gantry_status.calibration_flag = Initialization_Flag;
   gantry_status.position_reached = arrived;
   gantry_status.x_min = 250 * ticksToCmX;
   gantry_status.x_max = (X_max - 250) * ticksToCmX;
   gantry_status.y_min = 250 * ticksToCmY;
   gantry_status.y_max = (Y_max - 250) * ticksToCmY;
}

/**************************************************************************/
/*
    PUBLISHER
    Assign values to tf messages
   * 
*/
/**************************************************************************/
void gantry_tf()
{
  t.header.frame_id = gantry_frame;
  t.child_frame_id = sensor_head;
  t.transform.translation.x = X_encoderTicks * ticksToCmX ;
  t.transform.translation.y = Y_encoderTicks * ticksToCmY ;
  t.transform.rotation = tf::createQuaternionFromYaw(-3.14159 * stepper_pos/2000);
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
}

/**************************************************************************/
/*
    SUBSCRIBER
    Assign values to gantry_cmd_send messages
   * to_gantry_msg.state_desired ==> STATE
   * to_gantry_msg.sweep_speed_desired ==> sweepRight/sweepLeft
   * to_gantry_msg.x_desired ==> X_desired
   * to_gantry_msg.y_desired ==> Y_desired
   * to_gantry_msg.yaw_desired ==> R_desired
*/
/**************************************************************************/
void messageCb( const gantry::to_gantry_msg& gantry_cmd){
  STATE = gantry_cmd.state_desired;
  sweepRight = 90 - gantry_cmd.sweep_speed_desired;
  sweepLeft = 110 + gantry_cmd.sweep_speed_desired;
  if (sweepRight < 0) {sweepRight == 0;} 
  if (sweepLeft > 255) {sweepLeft == 255;} 
  X_desired = gantry_cmd.x_desired * cmToTicksX ;
  Y_desired = gantry_cmd.y_desired * cmToTicksY;
  R_desired = -gantry_cmd.yaw_desired * 2000 / 3.14159;
}
