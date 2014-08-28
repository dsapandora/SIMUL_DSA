#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/display.h>
#include <stdio.h>
#include <string.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

// This needs to be changed if the .wbt model changes
#define WHEEL_DIAMETER 0.748
#define AXLES_DIST 2.995
#define AXLE_LENGTH 1.627

// devices
WbDeviceTag left_front_wheel, right_front_wheel;
WbDeviceTag left_steer, right_steer;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;


// speedometer
WbDeviceTag display;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3];
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = false;
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 

 */


int AvanzaFrente()
{
    compute_gps_speed(40, 0);
    
return 0;
}

int AvanzaAtras()
{
int i=0;
    compute_gps_speed(-40, 0);
return 0;
}

int GirarIzquierda()
{
int i=0;
    compute_gps_speed(0, 1);
return 0;
}
int GirarDerecha()
{
int i=0;
    compute_gps_speed(0, -1);
return 0;
}

int GirarAtrasDerecha()
{
int i=0;
    compute_gps_speed(-40, -1);
return 0;
}

int GirarAtrasIzquierda()
{
int i=0;
    compute_gps_speed(-40, 1);
return 0;
}

int GirarDelanteDerecha()
{
int i=0;
    compute_gps_speed(20, 1);
return 0;
}

int GirardelanteIzquierda()
{
int i=0;
    compute_gps_speed(20, 1);
return 0;
}

int Detenerse()
{
int i=0;

    wb_servo_set_velocity(left_front_wheel, 0);
    wb_servo_set_velocity(right_front_wheel, 0);  
return 0;
}


void compute_gps_speed(double speed, int manual_steering) {
  const double *coords = wb_gps_get_values(gps);
    double vel[3] = { coords[X] - gps_coords[X], coords[Y] - gps_coords[Y], coords[Z] - gps_coords[Z] };
   double dist = sqrt(vel[X] * vel[X] + vel[Y] * vel[Y] + vel[Z] * vel[Z]);
//   speed=-5.0;
  // store into global variables
    gps_speed = dist / TIME_STEP * 3600.0;
    memcpy(gps_coords, coords, sizeof(gps_coords));
  //  manual_steering += -1;   
    steering_angle = manual_steering * 0.05;
    wb_servo_set_position(left_steer, steering_angle);
    wb_servo_set_position(right_steer, steering_angle);

    double turning_radius = AXLES_DIST / tan(steering_angle);
    double differential_ratio = 1.0 - AXLE_LENGTH / turning_radius;
    double left_speed = 2 * speed / (1 + differential_ratio);
    double right_speed = left_speed * differential_ratio;
    // set motor rotation speed
    const double KMH_TO_RADS = 1000.0 / 3600.0 / WHEEL_DIAMETER * 2.0;
    wb_servo_set_velocity(left_front_wheel, left_speed * KMH_TO_RADS);
    wb_servo_set_velocity(right_front_wheel, right_speed * KMH_TO_RADS);   
    return;
    
}