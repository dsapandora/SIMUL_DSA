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

//OPENCV
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/CvBridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV GUI handling
#include <opencv2/opencv.hpp>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
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

//Funcion que va a hablar con ROS
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
 {
   //ROS_INFO("Received %f %f %f %f %f %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
  // speed=0;
   //manual_steering = 0;
   if(msg->linear.x==1)
   {
    printf("Acelerador Presionado\n");
    speed=40.0; // km/h      
   }
   if(msg->linear.x==5)
   {
    printf("Acelerador Presionado\n");
    speed=-20.0; // km/h      
   }
   if(msg->linear.x==-1)
   {
   printf("Freno Presionado \n");
   speed=0;
   }
   
   
    manual_steering += msg->angular.z;   
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
 
 
 
 IplImage* save_camera_image(const unsigned char *image){

  int imgPixel = 0;

  int row;
  int col;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  // read rgb pixel values from the camera

  //create image in OpenCV format with width and height of the webots camera image
  IplImage* imgOpencv = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

  //information about the imgOpencv
  int widthStep = imgOpencv->widthStep;
  int nChannels = imgOpencv->nChannels;
  uchar* data           = (uchar *)imgOpencv->imageData;

  //counts the number of the channel that is being accessed
  int numberChannel;

  // redChannel = 2;
  // greenChannel = 1;
  // blueChannel = 0;


  for(numberChannel = 0; numberChannel < 3; numberChannel++){
    for(col = 0; col<height; col++){
      for(row = 0; row<width; row++){
        //printf("\n(%d,%d,%d)\n",numberChannel,row,col);
        if(numberChannel == 0){
        imgPixel = wb_camera_image_get_blue(image,width,row,col);
        }
        if(numberChannel == 1){
        imgPixel = wb_camera_image_get_green(image,width,row,col);
        }
        if(numberChannel == 2){
        imgPixel = wb_camera_image_get_red(image,width,row,col);
        }
        data[col * widthStep + row * nChannels + numberChannel] = imgPixel;
      }
    }
  }

 return imgOpencv;  
  }

int main(int argc, char **argv)
{
  printf("Comenzo todo\n");
  wb_robot_init();
  printf("Iniciando el sistema\n");
  // Ros initialization
  ros::init(argc, argv, "Simulador");
  ros::init(argc, argv, "Imagenes_de_Simulador");
  ros::NodeHandle n,nh,nh2;
  
 // ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  
  ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  image_transport::ImageTransport it(nh);
	
	//OpenCV HighGUI call to destroy a display window on shut-down.
  
  image_transport::ImageTransport it2(nh2);
  image_transport::Publisher pub2 = it.advertise("/webot/camera", 1);
  
  //ros::spin();
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
  
  // start engine
  speed=64.0; // km/h

  //print_help();
  
  // allow to switch to manual control
  //wb_robot_keyboard_enable(TIME_STEP);

  // main loop
  printf("Iniciando el ciclo de peticiones\n");
  
 // set_autodrive(false);z
  while (1) {
    
    const unsigned char *camera_image = wb_camera_get_image(camera);
    
    compute_gps_speed();
    update_display();    
    //wb_camera_save_image (camera, "/tmp/imagen.png", 100);
    //cv::WImageBuffer3_b image(cvLoadImage("/tmp/imagen.png", CV_LOAD_IMAGE_COLOR));
    IplImage* imagen=save_camera_image(camera_image);
    sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(imagen, "bgr8");
    pub2.publish(msg);
    cvReleaseImage(&imagen );
     ros::spinOnce();

    // wb_servo_set_position(left_steer, 0);
    // wb_servo_set_position(right_steer, 0);
     wb_robot_step(TIME_STEP);
  }

  wb_robot_cleanup();

  return 0;  // ignored

}