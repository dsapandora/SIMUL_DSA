// File:          AUTONOMO_DSA.java
// Date:          4 de mayo de 2013
// Description:   Controlador del Coche Autonomo para ser usado por la infraestructura 3T
// Author:        Ariel Vernaza Gonzalez
// Modifications: 

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.LED;
// or more simply:
//  import com.cyberbotics.webots.controller.*;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Servo;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;



// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
public enum coordenadas { X, Y, Z }

public class AUTONOMO_DSA extends Robot {
  
  // You may need to define your own functions or variables, like
  //  private LED led;
  


public static int TIME_STEP 50
public static double UNKNOWN 99999.99

// This needs to be changed if the .wbt model changes
public static double WHEEL_DIAMETER 0.748
public static double 2.995
public static double AXLE_LENGTH 1.627

// devices
private WbDeviceTag left_front_wheel, right_front_wheel;
private WbDeviceTag left_steer, right_steer;

// lights
WbDeviceTag brake_lights, backwards_lights, antifog_lights;
WbDeviceTag back_lights, front_turn_indicators, front_lights;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
WbDeviceTag display;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

void blink_lights() {
  int on = (int)wb_robot_get_time() % 2;
  wb_led_set(brake_lights, on);
  wb_led_set(backwards_lights, on);
  wb_led_set(antifog_lights, on);
  wb_led_set(back_lights, on);
  wb_led_set(front_turn_indicators, on);
  wb_led_set(front_lights, on);
}

void print_help() {
  printf("You can drive this car!\n");
  printf("Select the 3D window and then use the cursor keys to:\n");
  printf("[LEFT]/[RIGHT] - steer\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}

void set_autodrive(bool onoff) {
  if (autodrive == onoff) return;
  autodrive = onoff;
  switch (autodrive) {
  case false:
    printf("switching to manual drive...\n");
    printf("hit [A] to return to auto-drive.\n");
    break;
  case true:
    printf("switching to auto-drive...\n");
    break;
  }
}

// set target speed
void set_speed(double kmh) {
  
  // max speed
  if (kmh > 250.0)
    kmh = 250.0;
  
  speed = kmh;

  printf("setting speed to %g km/h\n", kmh);
  
  // compute wheels differential (limited slip)
  double turning_radius = AXLES_DIST / tan(steering_angle);
  double differential_ratio = 1.0 - AXLE_LENGTH / turning_radius;
  double left_speed = 2 * kmh / (1 + differential_ratio);
  double right_speed = left_speed * differential_ratio;
  
  // set motor rotation speed
  const double KMH_TO_RADS = 1000.0 / 3600.0 / WHEEL_DIAMETER * 2.0;
  wb_servo_set_velocity(left_front_wheel, left_speed * KMH_TO_RADS);
  wb_servo_set_velocity(right_front_wheel, right_speed * KMH_TO_RADS);
}

// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  steering_angle = wheel_angle;
  wb_servo_set_position(left_steer, steering_angle);
  wb_servo_set_position(right_steer, steering_angle);
}

void change_manual_steer_angle(int inc) {
  set_autodrive(false);
  manual_steering += inc;
  set_steering_angle(manual_steering * 0.02);

  if (manual_steering == 0)
    printf("going straight\n");
  else
    printf("turning %.2f rad (%s)\n", steering_angle, steering_angle < 0 ? "left" : "right");
}

void check_keyboard() {
  int key = wb_robot_keyboard_get_key();
  switch (key) {
    case WB_ROBOT_KEYBOARD_UP:
      set_speed(speed + 5.0);
      break;
    case WB_ROBOT_KEYBOARD_DOWN:
      set_speed(speed - 5.0);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT:
      change_manual_steer_angle(+1);
      break;
    case WB_ROBOT_KEYBOARD_LEFT:
      change_manual_steer_angle(-1);
      break;
    case 'A':
      set_autodrive(true);
      break;
  }
}

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
  const unsigned char REF[3] = { 48, 119, 132 }; // road yellow (BGR format)
  int sumx = 0;  // summed x position of pixels
  int pixel_count = 0;  // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
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

// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected
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
  
  // AUTONOMO_DSA constructor
  public AUTONOMO_DSA() {
      
    // call the Robot constructor
    super();
    
    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  led = getLED("ledName");
    
  }
    
  // User defined function for initializing and running
  // the AUTONOMO_DSA class
  public void run() {
    
    // Main loop:
    // Perform simulation steps of 64 milliseconds
    // and leave the loop when the simulation is over
    while (step(64) != -1) {
      
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      //  double val = distanceSensor.getValue();
      
      // Process sensor data here
      
      // Enter here functions to send actuator commands, like:
      //  led.set(1);
    };
    
    // Enter here exit cleanup code
  }

  // This is the main program of your controller.
  // It creates an instance of your Robot subclass, launches its
  // function(s) and destroys it at the end of the execution.
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {
    AUTONOMO_DSA controller = new AUTONOMO_DSA();
    controller.run();
  }
}
