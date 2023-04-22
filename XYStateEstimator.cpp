#include "XYStateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

XYStateEstimator::XYStateEstimator(void)
  : DataSource("x,y","float,float") // from DataSource
{}

void XYStateEstimator::init(void) {
 	state.x = 0;
  state.y = 0;
  state.yaw = 0;
}

void XYStateEstimator::updateState(imu_state_t * imu_state_p, gps_state_t * gps_state_p) {
  if (gps_state_p->num_sat >= N_SATS_THRESHOLD){


    // set the values of state.x, state.y, and state.yaw
    // It can make use of the constants RADIUS_OF_EARTH, origin_lat, origin_lon (see XYStateEstimator.h)
    // You can access the current GPS latitude and longitude readings with gps_state_p->lat and gps_state_p->lon
    // You can access the current imu heading with imu_state_p->heading
    // Also note that math.h is already included so you have access to trig functions [rad]

    ///////////////////////////////////////////////////////////////////////
    // write code here
    ///////////////////////////////////////////////////////////////////////
    // get x and y

    // get x and y
    float c_long = gps_state_p -> lon;
    float currentLongitude = c_long * PI/180;
    float cLat = gps_state_p -> lat;
    float currentLatitude = cLat * PI/180;
    float longitudeChange = currentLongitude - origin_lon*PI/180;
    float latitudeChange = currentLatitude - origin_lat*PI/180;
    float cosOrigLat = cos(origin_lat*PI/180);
    state.x = RADIUS_OF_EARTH_M*longitudeChange*cosOrigLat;
    state.y = RADIUS_OF_EARTH_M*latitudeChange;
    // get yaw 
    float heading_rad = imu_state_p->heading*PI/180.0; // convert to radians
    state.yaw = -1*heading_rad + (PI/2);
    ///////////////////////////////////////////////////////////////////////
    // don't change code past this point
    ///////////////////////////////////////////////////////////////////////
    gpsAcquired = 1;
  }
  else{
    gpsAcquired = 0;
  }
}

String XYStateEstimator::printState(void) {
  String currentState = "";
  int decimals = 2;
  if (!gpsAcquired){
    currentState += "XY_State: Waiting to acquire more satellites...";
  }
  else{
    currentState += "XY_State: x: ";
    currentState += String(state.x,decimals);
    currentState += "[m], ";
    currentState += "y: ";
    currentState += String(state.y,decimals);
    currentState += "[m], ";
    currentState += "Yaw:";
    currentState += String(state.yaw,decimals);
    currentState += "[rad]; ";
  }
  return currentState;
}

size_t XYStateEstimator::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  return idx + 2*sizeof(float);
}
