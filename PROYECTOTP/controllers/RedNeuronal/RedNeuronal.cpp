#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/display.h>
#include <stdio.h>
#include <string.h>
//Ingresando librerias de ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//RED NEURONAL
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdexcept>
// OpenNN includes
#include "/usr/local/opennn/source/opennn.h"

using namespace OpenNN;
NeuralNetwork neural_network(2,8,1);

// to be used as array indices
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

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;
//RESULTADO
  double obstacle_angle=0.0;
  double yellow_line_angle = 0.0;
   double left_speed;
  double right_speed; 

// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = { 132, 119, 48 };  // road yellow
  int sumx = 0;  // summed x position oupf pixels
  int pixel_count = 0;  // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 3) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++; // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;
  
  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}



void update_display() {
  // (re)paint background
  wb_display_image_paste(display, speedometer_image, 0, 0);
  
  // draw speedometer needle
  const double NEEDLE_LENGTH = 55.0;
  double alpha = gps_speed / 260.0 * 3.72 - 0.27;
  int x2 = -NEEDLE_LENGTH * cos(alpha);
  int y2 = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x2, 95 + y2); 

  // draw text
  char txt[256];
  sprintf(txt, "GPS coords: %.1f %.1f\n"
               "GPS speed:  %.1f",
               gps_coords[X], gps_coords[Z], gps_speed);
  wb_display_draw_text(display, txt, 10, 130);
}


void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  double vel[3] = { coords[X] - gps_coords[X], coords[Y] - gps_coords[Y], coords[Z] - gps_coords[Z] };
  double dist = sqrt(vel[X] * vel[X] + vel[Y] * vel[Y] + vel[Z] * vel[Z]);
  
  // store into global variables
  gps_speed = dist / TIME_STEP * 3600.0;
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

double process_sick_data(const float *sick_data) { 
  const int HALF_AREA = 20;  // check 20 degrees wide middle area
  int sumx = 0;
  int collision_count = 0;
  int x;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = wb_camera_range_image_get_depth(sick_data, sick_width, x, 0);
    if (range < 20.0) {
      sumx += x;
      collision_count++;
    }
  }
  
  // if no obstacle was detected...
  if (collision_count == 0)
    return UNKNOWN;

  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

  
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
 {
     
    return;
 }
 
  

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  steering_angle = wheel_angle;
  wb_servo_set_position(left_steer, steering_angle);
  wb_servo_set_position(right_steer, steering_angle);
}

int main(int argc, char **argv)
{
  printf("Comenzo todo\n");
  wb_robot_init();
  printf("Iniciando el sistema\n");  
  ros::init(argc, argv, "NEURONAL");  
  ros::NodeHandle n,nh,nh2;  
  ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  
   neural_network.load("/root/research/PROYECTOTP/controllers/RedNeuronal/neural_network.xml");
  // find front wheels
  left_front_wheel = wb_robot_get_device("left_front_wheel");
  right_front_wheel = wb_robot_get_device("right_front_wheel");
  wb_servo_set_position(left_front_wheel, INFINITY);
  wb_servo_set_position(right_front_wheel, INFINITY);
  
  // get steering motors
  left_steer = wb_robot_get_device("left_steer");
  right_steer = wb_robot_get_device("right_steer");

  // camera device
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);

  
  // initialize gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
 
  
  // initialize display (speedometer)
  display = wb_robot_get_device("display");
  speedometer_image = wb_display_image_load(display, "/root/research/PROYECTOTP/controllers/destiny_controller/speedometer.png");
  wb_display_set_color(display, 0xffffff);
  
    // SICK sensor
  sick = wb_robot_get_device("lms291");
  wb_camera_enable(sick, TIME_STEP);
  sick_width = wb_camera_get_width(sick);
  sick_range = wb_camera_get_max_range(sick);
  sick_fov = wb_camera_get_fov(sick);
  
  // start engine
  speed=64.0; // km/h
  // main loop
  printf("Iniciando el ciclo de peticiones\n");
  
 // set_autodrive(false);z
  Vector<double> inputs(2);
  Vector<double> outputs;
  while (1) {
    
    const unsigned char *camera_image = wb_camera_get_image(camera);    
    const float *sick_data = wb_camera_get_range_image(sick);
    
    inputs[0] = process_sick_data(sick_data);
    inputs[1] = process_camera_image(camera_image);
    outputs = neural_network.calculate_outputs(inputs);
     
   set_steering_angle(outputs[0]);
    
    compute_gps_speed();
    update_display();        
    ros::spinOnce();

     wb_robot_step(TIME_STEP);
  }

   wb_robot_cleanup();

  return 0;  // ignored

}
