/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Extended by Kristof Robot with:
    - DEBUG routines (incl. free ram detection, and logic analyzer debug pins)
    - WATCHDOG timer
    - Motorfault detection and motor coasting stop
    - Onboard wheel encoder counters
    - Two types of PID controllers (position and velocity)
    - Removed SERVO code and commands
    - Removed Sensors code and commands
    - Added SEND_PWM command
    - Added motor pwm rate limiting

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define DEBUG
#undef DEBUG

#define WATCHDOG
//#undef WATCHDOG

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   #define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* TinyQED encoder counters */
   //#define TINYQED

   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   //#define POSITION_PID
   #define VELOCITY_PID
#endif

/* Serial port baud rate */
//#define BAUDRATE     57600
#define BAUDRATE     115200

/* Maximum PWM signal */
#define MAX_PWM        400   //can go to 400, but can be set lower to protect motors
#define MIN_PWM        50    //lowest PWM before motors start moving reliably
#define RATE_LIMIT     25    //max increase in PWM allowed per cycle

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef WATCHDOG
  #include <avr/wdt.h>
#endif

/* Include definition of serial commands */
#include "commands.h"

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Initial PID Parameters */
  const int INIT_KP = 1;
  const int INIT_KD = 0;
  const int INIT_KI = 30;
  const int INIT_KO = 5;

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;

  boolean isMotorDisabled=false;
  boolean isEmergencyStop=false;   //set to true in case of motor fault
#endif

/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID(); //also need to reset PID
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
    }

    if (arg1 == 0 && arg2 == 0) {
      //setMotorSpeeds(0, 0); //hard stop
      isSafeStop = true; //gradual stop
      isPIDEnabled = false;
    } else {
      isPIDEnabled = true;
      isSafeStop = false;
    }
    leftPID.targetTicksPerFrame = arg1;
    rightPID.targetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
    Serial.println("OK");
    break;
  case CHECK_FAULT:
    if (isEmergencyStop)
      Serial.println(1); //in emergency stop
    else
      Serial.println(0);
    break;
 #ifdef DEBUG
  case SEND_PWM:
    { //need brackets to restrict scope of newly created variables to this case statement
      lastMotorCommand =  millis();

      if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
      }

      int leftSpeed = arg1;
      int rightSpeed = arg2;

      //ensure speeds are below MAX_PWM speed
      if (leftSpeed > MAX_PWM) leftSpeed = MAX_PWM;
      else if (leftSpeed < -MAX_PWM) leftSpeed = -MAX_PWM;
      if (rightSpeed > MAX_PWM) rightSpeed = MAX_PWM;
      else if (rightSpeed < -MAX_PWM) rightSpeed = -MAX_PWM;

      setMotorSpeeds(leftSpeed, rightSpeed);
      isPIDEnabled = false; //no need to do PID
      isSafeStop = false;
      Serial.println("OK");
      break;
    }
  #endif
#endif
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK");
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Run emergency commands only.  Commands are defined in commands.h */
void runEmergencyCommand() {
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  case CHECK_FAULT:
    if (isEmergencyStop)
      Serial.println(1); //in emergency stop
    else
      Serial.println(0);
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {

  Serial.begin(BAUDRATE);

#ifdef DEBUG
  Serial.println("Starting up...");
  pinMode(A2, OUTPUT); //cpu measurement pin for logic analyzer
  pinMode(A3, OUTPUT); //PID frequency measurement pin for logic analyzer
  Serial.println("CPU and PID frequency measurement PINs active.");
#endif

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);

    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);

    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B); // tell pin change mask to listen to left encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B); // tell pin change mask to listen to right encoder pins

    PCICR |= (1 << PCIE1) | (1 << PCIE2);   // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  #endif

  initMotorController();
  //init PID
  setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  resetPID();
#endif

#ifdef WATCHDOG
  wdt_reset();
  wdt_enable(WDTO_1S);  //reset after 1s of inactivity
#endif

#ifdef DEBUG
    Serial.print("Free Mem:");
    Serial.println(freeRam());
#endif

}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {

#ifdef WATCHDOG
    //watchdog protection; if we dont reset this every X ms
    //arduino will reset
    wdt_reset();
#endif


  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = 0;
      else if (arg == 2) argv2[index] = 0;
      if (isEmergencyStop)
        runEmergencyCommand();
      else
        runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = 0;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    nextPID = millis() + PID_INTERVAL;
  #ifdef DEBUG
    PINC = (1<<PC3);   //toggle for PID interval measurement with logic analyzer, pin A3
  #endif
    updatePID();
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    //setMotorSpeeds(0, 0);
    //coast to a stop
    setMotorEnableFlag(false);
    isMotorDisabled=true;
    isPIDEnabled = false;
    isSafeStop = false;
  }

  #ifndef DEBUG
  //detect motor faults
  //if motor is disabled, motor fault is active; so excluding that case
  if (!isMotorDisabled && isMotorFault())
  {
    Serial.println("FATAL ERROR: Motor fault - stopping");
    setMotorEnableFlag(false);
    isMotorDisabled=true;
    isPIDEnabled = false;
    isSafeStop = false;
    //activate emergencyStop
    isEmergencyStop = true;
  }
  #endif


  #ifdef DEBUG
    //toggle cpu measurement for logic analyzer on pin 5
    PINC = (1<<PC2); //pin A2
  #endif

#endif
}

#ifdef DEBUG
/* Report free ram */
/* From http://jeelabs.org/2011/05/22/atmega-memory-use/ */
int freeRam () {

  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);

}
#endif



