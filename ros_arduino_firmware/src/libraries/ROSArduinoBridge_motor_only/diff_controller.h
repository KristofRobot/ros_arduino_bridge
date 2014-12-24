/* 
*  Functions and type-defs for PID control.
*
*  Based on the Beginner PID's series by Brett Beauregard - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*  Adapted to use ideal velocity form or position form.
*
*  Originally adapted from Mike Ferguson's ArbotiX code which lives at:
*   
*  http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#ifdef VELOCITY_PID
  /* PID setpoint info for a Motor */
  typedef struct {
    int targetTicksPerFrame;    // target speed in ticks per frame
    long encoder;                  // encoder count
    long prevEnc;                  // last encoder count
    int prevInput;                 // last input
    int prevPrevInput;             // input before last input
    int prevError;                 // last error
    int output;                   // last motor setting
  }
  SetPointInfo;
#elif defined POSITION_PID
  /* PID setpoint info For a Motor */
  typedef struct {
    int targetTicksPerFrame;    // target speed in ticks per frame
    long encoder;                  // encoder count
    long prevEnc;                  // last encoder count
  
    /*
    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */
    int prevInput;                // last input
    //int prevErr;                   // last error
  
    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    long iTerm;                    //integrated term
    int output;                    // last motor setting
  }
  SetPointInfo;
#endif

SetPointInfo leftPID, rightPID;

int last_right_pwm = 0;
int last_left_pwm = 0;

/* PID Parameters 
* Do not SET these directly here, unless you know what you are doing 
* Use setPIDParameters() instead
*/
int Kp = 0;    
int Kd = 0;
int Ki = 0;      
int Ko = 1; 

boolean isPIDEnabled = false; //is PID enabled?
boolean isSafeStop = false; //are we trying to stop safely?

/*
* General rate limiter.
* Based on email from davida at smu.edu to hbrobotics group
*/
int rate_limit(int val,int last,int rate)
{
        int x;
        x = val-last;
        if (abs(x) > rate) {
                if (x < 0) {
                        x = last - rate;
                } else  {
                        x = last + rate;
                }
        } else  {
                x = val;
        }
        return x;
}

/*
* PWM rate limiter to safeguard the motor control signal.
*/
int pwm_rate_limit(int val_pwm,int last_pwm)
{
    if (val_pwm*last_pwm < 0){
      //switching from forward to backward or vice versa
      val_pwm = rate_limit(val_pwm,last_pwm,RATE_LIMIT);
      //make sure we stay out of ]-MIN_PWM, MIN_PWM[
      if (abs(val_pwm) < MIN_PWM){
        if (last_pwm > 0)
          //going from positive to negative, so jump immediately to -MIN_PWM
          val_pwm = -MIN_PWM;
        else if (last_left_pwm < 0)
          //going from negative to positive, so jump immediately to MIN_PWM
          val_pwm = MIN_PWM;
      }
    } else if (abs(val_pwm) > MIN_PWM) {
      //normal rate limiting
      val_pwm = rate_limit(val_pwm,last_pwm,RATE_LIMIT);
      if (abs(val_pwm) < MIN_PWM){
        //make sure we have at least -MIN_PWM
        if (val_pwm > 0)
          val_pwm = MIN_PWM;
        else if (val_pwm < 0)
          val_pwm = -MIN_PWM;
      }
    } else {
       //should never get here - put on zero to be safe
       val_pwm = 0;
    }
    return val_pwm;
}

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and prevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
#ifdef VELOCITY_PID
   leftPID.targetTicksPerFrame = 0;
   leftPID.encoder = readEncoder(0);
   leftPID.prevEnc = leftPID.encoder;
   leftPID.output = 0;
   leftPID.prevInput = 0;
   leftPID.prevPrevInput = 0;
   leftPID.prevError = 0;

   rightPID.targetTicksPerFrame = 0;
   rightPID.encoder = readEncoder(1);
   rightPID.prevEnc = rightPID.encoder;
   rightPID.output = 0;
   rightPID.prevInput = 0;
   rightPID.prevPrevInput = 0;
   rightPID.prevError = 0;
#elif defined POSITION_PID
   leftPID.targetTicksPerFrame = 0;
   leftPID.encoder = readEncoder(0);
   leftPID.prevEnc = leftPID.encoder;
   leftPID.output = 0;
   leftPID.prevInput = 0;
   leftPID.iTerm = 0;

   rightPID.targetTicksPerFrame = 0;
   rightPID.encoder = readEncoder(1);
   rightPID.prevEnc = rightPID.encoder;
   rightPID.output = 0;
   rightPID.prevInput = 0;
   rightPID.iTerm = 0;
#endif
}

#ifdef VELOCITY_PID
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  /*
  * Use velocity form rather than position form.
  *
  * Avoid derivative kick (derivative on measurement):
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  *
  * see https://groups.google.com/forum/#!topic/diy-pid-control/1Tkwp9e_8co for full derivation 
  */
  output = p->output + (Kp * (error - p->prevError) + Ki * error - Kd * (input - 2*p->prevInput + p->prevPrevInput)) / Ko;
  
  /*version robust against too aggressive setpoint tracking (with proportional on measurement)*/
  //output = p->output + (Kp * (p->prevInput - input) + Ki * error - Kd * (input - 2*p->prevInput + p->prevPrevInput)) / Ko;
  
  /*
  * Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0 && output < MIN_PWM) 
    output = MIN_PWM;
  else if (p->targetTicksPerFrame < 0 && output > -MIN_PWM) 
    output = -MIN_PWM;
  else if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;
    
  p->output = output;
  p->prevEnc = p->encoder;
  p->prevPrevInput = p->prevInput;
  p->prevInput = input;
  p->prevError = error;
  
  /*Serial.print(output);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  Serial.println(input);*/
}
#elif defined POSITION_PID
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  p->iTerm += (Ki * error) / Ko;
  if (p->iTerm > (MAX_PWM-MIN_PWM)) p->iTerm = MAX_PWM;
  else if (p->iTerm < (-MAX_PWM+MIN_PWM)) p->iTerm = -MAX_PWM;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (((long)Kp) * error - Kd * (input - p->prevInput))/ Ko + p->iTerm;
  p->prevEnc = p->encoder;

  /*
  * Accumulate Integral error *or* Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0){
    output += MIN_PWM;
    if (output < MIN_PWM) output = MIN_PWM;
  } else if (p->targetTicksPerFrame < 0){
    output += -MIN_PWM;
    if (output > -MIN_PWM) output = -MIN_PWM;
  } 
  
  if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;

  p->output = output;
  p->prevInput = input;
}
#endif

/* Read the encoder values and call the PID routine */
void updatePID() {
  int right_pwm = 0;
  int left_pwm = 0;

  /* Read the encoders */
  leftPID.encoder = readEncoder(0);
  rightPID.encoder = readEncoder(1);
    
  /* If we're not moving there is nothing more to do */
  if (!isPIDEnabled){
    //are we trying to stop safely?
    if (isSafeStop){
      /* rate limit output to 0 */
      left_pwm = rate_limit(0,last_left_pwm,RATE_LIMIT);
      right_pwm = rate_limit(0,last_right_pwm,RATE_LIMIT);

      if (abs(left_pwm) < MIN_PWM) left_pwm = 0;
      if (abs(right_pwm) < MIN_PWM) right_pwm = 0;

      if (left_pwm == 0 && right_pwm == 0)
        //safely stopped - end safeStopping condition
        isSafeStop = false;

    } else {
       /*
      * Reset PIDs, to prevent startup spikes,
      * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
      * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
      */
      if (leftPID.prevEnc != leftPID.encoder || rightPID.prevEnc != rightPID.encoder) {
        resetPID();
        last_right_pwm = 0;
        last_left_pwm = 0;
      }
      return;
    }

  } else {
    /* Compute PID update for each motor */
    doPID(&rightPID);
    doPID(&leftPID);

    /* Rate limit output */
    left_pwm = pwm_rate_limit(leftPID.output, last_left_pwm);
    right_pwm = pwm_rate_limit(rightPID.output, last_right_pwm);
  }

  //save values
  last_right_pwm = right_pwm;
  last_left_pwm = left_pwm;

  /* Set the motor speeds accordingly */
  setMotorSpeeds(left_pwm, right_pwm);

}


/* Set PID parameters */
//Assuming pid_rate and Kx parameters all given in s
//Doing some effort to keep things in integer math, through use of Ko
void setPIDParams(int newKp, int newKd, int newKi, int newKo, int pidRate){
    Kp = newKp * pidRate;
    Ki = newKi;
    Kd = newKd * pidRate * pidRate;  
    Ko = newKo * pidRate;
}




