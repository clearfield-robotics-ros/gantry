//saftey buffer
double buffer = 500;
double slowZone = 2000;
double slowSpeed;

void sweep() {
//  if (Initialization_Flag) {
    stepper_pos = 0;
    stepper_rot.runToNewPosition(stepper_pos);
    if (speed_Y == zeroSpeed && Y_encoderTicks > buffer && Y_encoderTicks < Y_max - buffer) {
      set_speed(zeroSpeed, sweepLeft);
    }
    if (Y_encoderTicks < buffer) {
      set_speed(zeroSpeed, sweepLeft);
      if (Debug) {Serial.println("going left");}
    }
    else if (Y_encoderTicks > Y_max-buffer){
      set_speed(zeroSpeed, sweepRight);
      if (Debug) {Serial.println("going right");}
    }
    else if (Y_encoderTicks < slowZone && speed_Y < 90) {
      slowSpeed = sweepRight + .01*(slowZone - Y_encoderTicks);
      set_speed(zeroSpeed, slowSpeed);
    }
    else if (Y_encoderTicks > Y_max - slowZone && speed_Y > 90) {
      slowSpeed = (sweepLeft + .02*((Y_max-slowZone) - Y_encoderTicks));
      set_speed(zeroSpeed, slowSpeed);
    }
//  }
}

