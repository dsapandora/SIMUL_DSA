#include <webots/robot.h>
#include <webots/servo.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/display.h>
#include <webots/led.h>
#include <stdio.h>
#include <string.h>
#include "planes.h"

//Ingresando librerias de ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//OPENCV
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>

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


// returns approximate angle of obstacle
// or UNKNOWN if no obstacle was detected

//Funcion que va a hablar con ROS
 void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
 {
   ROS_INFO("Received %f %f %f %f %f %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
   if(msg->linear.x==10 && msg->angular.z==10)
   {
    //set_autodrive(true);
    return;
   }
  // if(msg->angular.z==0)
  // set_speed(speed + msg->linear.x);
   ///else
  // change_manual_steer_angle(msg->angular.z);
 }
 
 //This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	IplImage hsv_img,*thresholded_img,*overlay,buf;
	CvMoments moments;
	CvPoint nuevo=CvPoint();
	cv::Mat hsv_img2;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
        //convierte la imagen hsv(Hue, Saturation, Value) haciendo en teoria
        //mas facil de determinar el color que se va a seguir(
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);

		buf=cv_ptr->image;
		cvSmooth(&buf, &buf, CV_BLUR,3,0,0,0);

		hsv_img2=cv::Mat(cvCreateImage(cvGetSize(&buf), 8,3));
		//crea una imagen e inmeditamente la pasa a Mat pars evitar el legacy
		hsv_img=hsv_img2;//se la asigna para poder manipularla
		//cv_ptr->image=cv::Mat(&buf);
		//hsv_img2= cv::Mat(&hsv_img);//asignado para poder ser manupulado
		cvCvtColor( &buf,&hsv_img,CV_BGR2HSV);
		//cv_ptr->image=cv::Mat(&buf);//le regresa el valor a mat
		//hsv_img=hsv_img2;

		//reasignado el valor transformado
          //limit all pixels that don't match our criteria, in this case we are
         //looking for purple but if you want you can adjust the first value in
         //both turples which is the hue range(120,140).  OpenCV uses 0-180 as
         //a hue range for the HSV color model
	  thresholded_img =  cvCreateImage(cvGetSize(&buf), 8, 1);
	  cvInRangeS(&hsv_img, cvScalar(35, 200, 100), cvScalar(75, 255, 255), thresholded_img);
	  //error de puntero corregido
    //determine the objects moments and check that the area is large
    //enough to be our object
	  cvMoments( thresholded_img,&moments, 0);
      double area = cvGetCentralMoment(&moments,0,0);

          //there can be noise in the video so ignore objects with small areas
            if(area > 100000){
                //determine the x and y coordinates of the center of the object
                //we are tracking by dividing the 1, 0 and 0, 1 moments by the area

            	double x = cvGetSpatialMoment(&moments, 1, 0)/area;
                double y = cvGetSpatialMoment(&moments, 0, 1)/area;
                overlay = cvCreateImage(cvGetSize(&buf), 8, 3);

                nuevo.x=x;
                nuevo.y=y;
                cvCircle(overlay,nuevo, 2, cvScalar(255, 255, 255), 20);
                cvAdd(&buf, overlay, &buf);
                //cv_ptr->image=cv::Mat(&buf);
                //dd the thresholded image back to the img so we can see what was
                //left after it was applied

                cvMerge(thresholded_img, NULL, NULL, NULL, &buf);
                printf("Detectado\n");
                //cv_ptr->image=cv::Mat(&buf);
            }

	}
		catch (cv_bridge::Exception& e)
		{
			//if there is an error during conversion, display it

			ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}
	//Display the image using OpenCV
      	cv::imshow(WINDOW, cv_ptr->image);
      	cv::waitKey(3);
        pub.publish(cv_ptr->toImageMsg());

        cvReleaseImage(&thresholded_img);
        cvReleaseImage(&overlay);
          return;

}

int main(int argc, char **argv)
{
  wb_robot_init();
  printf("Iniciando\n");
  // Ros initialization
  ros::init(argc, argv, "webots");
  ros::init(argc, argv, "image_processor");
  ros::NodeHandle n,nh;
 // ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  
  ros::Subscriber chatter_sub = n.subscribe("/auto_controller/command", 100, chatterCallback);
  image_transport::ImageTransport it(nh);
	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
	//OpenCV HighGUI call to destroy a display window on shut-down.
	
  cv::destroyWindow(WINDOW);
  pub = it.advertise("camera/image_processed", 1); 
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
  
  // find lights

  
  // initialize display (speedometer)
  display = wb_robot_get_device("display");
  speedometer_image = wb_display_image_load(display, "speedometer.png");
  wb_display_set_color(display, 0xffffff);
  
  // start engine
  
  // allow to switch to manual control
  wb_robot_keyboard_enable(TIME_STEP);
  Detenerse();
  // main loop
  printf("Iniciando el ciclo de peticiones\n");
  while (wb_robot_step(TIME_STEP) != -1) {
    // get user input
   /* check_keyboard();
    
    // read sensors
    const unsigned char *camera_image = wb_camera_get_image(camera);
    const float *sick_data = wb_camera_get_range_image(sick);

    if (autodrive) {
      double obstacle_angle = process_sick_data(sick_data);
      double yellow_line_angle = process_camera_image(camera_image);
      
      // avoid obstacles and follow yellow line
      if (obstacle_angle != UNKNOWN)
        set_steering_angle(-0.005 / obstacle_angle);
      else if (yellow_line_angle != UNKNOWN)
        set_steering_angle(yellow_line_angle * 0.3);
    }
    
    // update stuff
    compute_gps_speed();
    update_display();
    blink_lights();*/
    GirarAtrasDerecha();
     ros::spinOnce();
  }

  wb_robot_cleanup();

  return 0;  // ignored

}
