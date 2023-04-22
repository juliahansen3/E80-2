/********
Default E80 Code
Current Author:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
Previous Contributors:
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/


#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ZStateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <SurfaceControl.h>
#include <DepthControl.h>
#define UartSerial Serial1
#define DELAY 0 //change here for surface delay, maybe 15 minutes 
#include <GPSLockLED.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
XYStateEstimator state_estimator;
ZStateEstimator z_state_estimator;
SurfaceControl surface_control;
DepthControl depth_control;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
ErrorFlagSampler ef;
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
volatile bool EF_States[NUM_FLAGS] = {1,1,1};
// GPS Waypoints
const int number_of_waypoints = 1;
const int waypoint_dimensions = 2;       // waypoint dimensions (x,y)                                                                                                                                                                                                                                                            ts are set to have two pieces of information, x then y.
double waypoints [] = { 0, 0};   // listed as x0,y0,x1,y1, ... etc. //desired location at 0, or could move to different x,y points, have plus hour for time 

//Depth Control Points
int diveDelay = 40000; // how long robot will stay at depth waypoint before continuing (ms)
//25 total seconds at each depth
const int num_depth_waypoints = 10;
double depth_waypoints [] = {0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3} ; // listed as z0,z1,... etc.
////////////////////////* Setup *////////////////////////////////

void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&state_estimator);
  //logger.include(&XYStateEstimator);
  logger.include(&z_state_estimator);
  logger.include(&surface_control);
  logger.include(&depth_control);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&ef);
  logger.include(&button_sampler);
  //logger.include(currentTime);
  logger.init();

  printer.init();
  ef.init();
  button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();

  surface_control.init(number_of_waypoints, waypoints, DELAY);
  depth_control.init(num_depth_waypoints, depth_waypoints, diveDelay);
  z_state_estimator.init();
  state_estimator.init(); 

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime         = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime             = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  adc.lastExecutionTime             = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  ef.lastExecutionTime              = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  button_sampler.lastExecutionTime  = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + XY_STATE_ESTIMATOR_LOOP_OFFSET;
  z_state_estimator.lastExecutionTime  = loopStartTime - LOOP_PERIOD + Z_STATE_ESTIMATOR_LOOP_OFFSET;
  depth_control.lastExecutionTime      = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
  surface_control.lastExecutionTime        = loopStartTime - LOOP_PERIOD + SURFACE_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime          = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}



//////////////////////////////* Loop */////////////////////////
void EFA_Detected(void){
  EF_States[0] = 0;
}

void EFB_Detected(void){
  EF_States[1] = 0;
}

void EFC_Detected(void){
  EF_States[2] = 0;
}
void loop() {
  currentTime=millis();
  if (currentTime-loopStartTime <1){
    printer.printMessage("delayed", 1);
    delay(60000);
  }
  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample()); 
    printer.printValue(1,ef.printStates());
    printer.printValue(2,logger.printState());
    printer.printValue(3,gps.printState());   
    printer.printValue(4,state_estimator.printState());    
    printer.printValue(5,z_state_estimator.printState());      
    printer.printValue(6,depth_control.printWaypointUpdate());
    printer.printValue(7,depth_control.printString()); 
    printer.printValue(8,surface_control.printWaypointUpdate());
    printer.printValue(9,surface_control.printString());
    printer.printValue(10,motor_driver.printState());
    printer.printValue(11,imu.printRollPitchHeading());        
    printer.printValue(12,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  //if ( currentTime-gps.lastExecutionTime > LOOP_PERIOD ) {
  //  gps.lastExecutionTime = currentTime;
    gps.read(&GPS); // blocking UART calls
  //}

  //if ( currentTime-state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    //printer.printMessage("State estimator time:", 10);
    //printer.printValue(13, state_estimator.lastExecutionTime);
    //state_estimator.lastExecutionTime = currentTime;
    state_estimator.updateState(&imu.state, &gps.state);
  //}


  //only to test
  //motor_driver.drive(0,0,10);
  if ( currentTime-surface_control.lastExecutionTime > LOOP_PERIOD ) {
    surface_control.lastExecutionTime = currentTime;
    surface_control.navigate(&state_estimator.state, &gps.state, DELAY);
    motor_driver.drive(surface_control.uL,surface_control.uR,0);
  }
  /* ROBOT CONTROL Finite State Machine */
  /*if ( currentTime-depth_control.lastExecutionTime > LOOP_PERIOD ) {
    depth_control.lastExecutionTime = currentTime;
    if ( depth_control.diveState ) {      // DIVE STATE //
      depth_control.complete = false;
      if ( !depth_control.atDepth ) {
        depth_control.dive(&z_state_estimator.state, currentTime);
      }
      else {
        depth_control.diveState = false; 
        depth_control.surfaceState = true;
      } 
      motor_driver.drive(0,0,depth_control.uV);
    }
    if ( depth_control.surfaceState ) {     // SURFACE STATE //
      if ( !depth_control.atSurface ) { 
        depth_control.surface(&z_state_estimator.state);
      }
      else if ( depth_control.complete ) { 
        delete[] depth_control.wayPoints;   // destroy depth waypoint array from the Heap
      }
      //ASK
      motor_driver.drive(0,0,depth_control.uV);
    }
    */
   if ( currentTime-depth_control.lastExecutionTime > LOOP_PERIOD ) {
    depth_control.lastExecutionTime = currentTime;
    if ( depth_control.diveState ) {      // DIVE STATE //
      depth_control.complete = false;
      if ( !depth_control.atDepth ) {
        depth_control.dive(&z_state_estimator.state, currentTime);
      }
      else {
        depth_control.diveState = false; 
        depth_control.surfaceState = true;
      }
      motor_driver.drive(0,0,depth_control.uV);
    }
    if ( depth_control.surfaceState ) {     // SURFACE STATE //
      if ( !depth_control.atSurface ) { 
        depth_control.surface(&z_state_estimator.state);
      }
      else if ( depth_control.complete ) { 
        delete[] depth_control.wayPoints;   // destroy depth waypoint array from the Heap
      }
      motor_driver.drive(0,0,depth_control.uV);
    }
  }
  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    delay(5);
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

 // uses the ButtonSampler library to read a button -- use this as a template for new libraries!
  if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
    button_sampler.lastExecutionTime = currentTime;
    button_sampler.updateState();
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }
  if ( currentTime-z_state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    z_state_estimator.lastExecutionTime = currentTime;
    z_state_estimator.updateState(analogRead(PRESSURE_PIN));}
 
  
  
  if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}


