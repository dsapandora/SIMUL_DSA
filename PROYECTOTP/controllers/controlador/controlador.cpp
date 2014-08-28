#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/display.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
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

#define TIME_STEP 32
#define UNKNOWN 99999.99

// This needs to be changed if the .wbt model changes
#define WHEEL_DIAMETER 0.748
#define AXLES_DIST 2.995
#define AXLE_LENGTH 1.627

// devices
WbDeviceTag left_front_wheel, right_front_wheel;
WbDeviceTag left_steer, right_steer;

// camera
WbDeviceTag camera,camera2;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;
//othercamara
int camera_width2 = -1;
int camera_height2 = -1;
double camera_fov2 = -1.0;


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


/* SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;


*/
///////Archivo de experimento
 FILE *archivo;
 //RESULTADO
  double obstacle_angle=0.0;
  double yellow_line_angle = 0.0;
   double left_speed;
  double right_speed; 
//Archivo de experimento int id_intereaccion=0;
const unsigned char *camera_image;
// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}
/*
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
*/
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

//Funcion que va a hablar con ROS
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
 {
  // ROS_INFO("Received %f %f %f %f %f %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
  // speed=0;
   //manual_steering = 0;
   if(msg->linear.x<0 && speed <= 40)
   {    
    speed+=10; // km/h      
   }
   if(msg->linear.x>=5000 && speed >= 40)
   {    
    speed-=10; // km/h      
   }
   if(msg->linear.x>=5000 && speed >= 40)
   {    
    speed-=10; // km/h      
   }
   if(msg->linear.x==0)
   {    
    speed=0; // km/h      
   }     
   if(msg->angular.z > 0 )    {  
       steering_angle += 0.078539816339745;
       //ROS_INFO("DERECHA");
   }
   if(msg->angular.z == 0)
   {
	   steering_angle = 0;
	   //    ROS_INFO("RECTO");
   }
   if(msg->angular.z < 0) {     
     //  ROS_INFO("IZQUIERDA");
       steering_angle -= 0.078539816339745;
       }
    //manual_steering += msg->angular.z;   
    //steering_angle = manual_steering * 0.05;
    wb_servo_set_position(left_steer, steering_angle);
    wb_servo_set_position(right_steer, steering_angle);
   // wb_robot_step(1);
    double turning_radius = AXLES_DIST / tan(steering_angle);
    double differential_ratio = 1.0 - AXLE_LENGTH / turning_radius;
    double left_speed = 2 * speed / (1 + differential_ratio);
    double right_speed = left_speed * differential_ratio;
    // set motor rotation speed
    const double KMH_TO_RADS = 1000.0 / 3600.0 / WHEEL_DIAMETER * 2.0;
    if(left_speed * KMH_TO_RADS < 156) 
       wb_servo_set_velocity(left_front_wheel, left_speed * KMH_TO_RADS);
    else
       wb_servo_set_velocity(left_front_wheel, 156);
    if(right_speed * KMH_TO_RADS < 156)    
        wb_servo_set_velocity(right_front_wheel, right_speed * KMH_TO_RADS);   
        else
         wb_servo_set_velocity(right_front_wheel, 156);   
   //  fprintf(archivo,"%g\t%g\t%g\t%g\n",obstacle_angle,yellow_line_angle, speed ,steering_angle);
    //Archivo de experimento char buffer[100];
    //Archivo de experimento sprintf(buffer,"/root/research/log/%d.png",id_intereaccion);
     //Archivo de experimento wb_camera_save_image(camera,buffer,50);
     //Archivo de experimento id_intereaccion++;
    return;
 }
 
 
 
 IplImage* save_camera_image(const unsigned char *image,WbDeviceTag tmp){

  int imgPixel = 0;

  int row;
  int col;

  int width = wb_camera_get_width(tmp);
  int height = wb_camera_get_height(tmp);

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


  for(numberChannel = 0; numberChannel < 3; ++numberChannel){
    for(col = 0; col<height; col++){
      for(row = 0 ; row< width-1; ++row){
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
  ros::NodeHandle n,nh,nh2,nh3;
  //INICIANDO ARCHIVO
     
  //
 // archivo=fopen("/root/research/log/prueba.dat","a");
 // ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  
  ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  image_transport::ImageTransport it(nh);
	
	//OpenCV HighGUI call to destroy a display window on shut-down.
  
  image_transport::ImageTransport it2(nh2);
  image_transport::Publisher pub2 = it.advertise("/webot/camera", 1);
  
  image_transport::ImageTransport it3(nh3);
  image_transport::Publisher pub3 = it3.advertise("/webot/camera2", 1);
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
  camera2 = wb_robot_get_device("camera2");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_enable(camera2, TIME_STEP);
  camera_width = wb_camera_get_width(camera);
  camera_height = wb_camera_get_height(camera);
  camera_fov = wb_camera_get_fov(camera);
  camera_width2 = wb_camera_get_width(camera2);
  camera_height2 = wb_camera_get_height(camera2);
  camera_fov2 = wb_camera_get_fov(camera2);
  
  
  /*SICK sensor
  sick = wb_robot_get_device("lms291");
  wb_camera_enable(sick, TIME_STEP);
  sick_width = wb_camera_get_width(sick);
  sick_range = wb_camera_get_max_range(sick);
  sick_fov = wb_camera_get_fov(sick);
  */
  // initialize gps
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
 
  
  // initialize display (speedometer)
  //display = wb_robot_get_device("display");
 // speedometer_image = wb_display_image_load(display, "/root/research/PROYECTOTP/controllers/destiny_controller/speedometer.png");
  //wb_display_set_color(display, 0xffffff);
  
  // start engine
  speed=0.0; // km/h

  //print_help();
  
  // allow to switch to manual control
  //wb_robot_keyboard_enable(TIME_STEP);

  // main loop
  printf("Iniciando el ciclo de peticiones\n");
  int i=0;
 // set_autodrive(false);z
  while (1) {
    
     camera_image = wb_camera_get_image(camera);
    // const float *sick_data = wb_camera_get_range_image(sick);

     //////
  //    obstacle_angle = process_sick_data(sick_data);
      yellow_line_angle = process_camera_image(camera_image);
     //////
     
     const unsigned char *camera_image2 = wb_camera_get_image(camera2);
     IplImage* imagen=save_camera_image(camera_image,camera);
     IplImage* imagen2=save_camera_image(camera_image2,camera2);
     sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(imagen, "bgr8");
     pub2.publish(msg);
     sensor_msgs::ImagePtr msg2 = sensor_msgs::CvBridge::cvToImgMsg(imagen2, "bgr8"); //COMENTADO PARA EXPERIMENTAR

     pub3.publish(msg2);
     cvReleaseImage(&imagen );
      cvReleaseImage(&imagen2 ); //COMENTADO PARA EXPERIMENTAR
     ros::spinOnce();
     wb_robot_step(64);
     
  }

  wb_robot_cleanup();
 // fclose(archivo);
  return 0;  // ignored

}
