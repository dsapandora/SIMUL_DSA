#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//OPENCV
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/CvBridge.h>
//Include headers for OpenCV GUI handling
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <opencv/cvaux.h>
//Librerias de SDL
#include <string.h>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#include <SDL/SDL_opengl.h>
//Enviar Informacion a webots
#include <geometry_msgs/Twist.h>


namespace enc = sensor_msgs::image_encodings;
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub2;
image_transport::Publisher pub3;


using namespace std;
SDL_Surface *screen,*retro,*imagen,*imagen2,*caratula=IMG_Load("/root/research/HOG/cvCropSrc/cvCrop/imagen.png");;	 
SDL_Surface* optimizedImage = NULL;
IplImage *destination;
IplImage *destination2;

SDL_Surface *ipl_to_surface (IplImage *opencvimg,int valor)
{
   int pitch = opencvimg->nChannels*opencvimg->width;
   SDL_Surface *surface = SDL_CreateRGBSurfaceFrom((void*)opencvimg->imageData,
                opencvimg->width*valor,
                opencvimg->height*valor,
                opencvimg->depth*opencvimg->nChannels,
                opencvimg->widthStep,
                0xff0000, 0x00ff00, 0x0000ff, 0
                );
    return surface;
}	 


IplImage* rotate(IplImage* image, float angle) {

IplImage *rotatedImage = cvCreateImage(cvSize(800,800), IPL_DEPTH_8U,image->nChannels);

CvPoint2D32f center;
center.x = 400;center.y = 400;
CvMat *mapMatrix = cvCreateMat( 2, 3, CV_32FC1 );

cv2DRotationMatrix(center, angle, 1.0, mapMatrix);
cvWarpAffine(image, rotatedImage, mapMatrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

//cvReleaseImage(&image);
cvReleaseMat(&mapMatrix);

return rotatedImage;
}
	
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  SDL_FreeSurface(imagen);
  cvReleaseImage(&destination);
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::CvBridge bridge;
   IplImage buf;
   cv::Mat img;
   cv::HOGDescriptor hog;
  try
  {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);  
      //   cv::imshow("view", cv_ptr->image);
	buf=cv_ptr->image;
	SDL_Rect dest;
	dest.x = 0;  
    double percent=110.00;
    //double percent=50.00;
    //destination = cvCreateImage(cvSize((int)((buf.width*2)) ,(int)(buf.height*2)),buf.depth, buf.nChannels);
    // cvResize(&buf, destination,CV_INTER_AREA);
//    imagen=ipl_to_surface(rotate(destination, -90.00));
    imagen=ipl_to_surface(&buf,1);
    SDL_BlitSurface (imagen,NULL, screen, &dest );	
	dest.x = 200;  
    SDL_BlitSurface (caratula,NULL, screen, &dest );

    
   
 // 
//  cvReleaseImage(buf);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
}

void imageCallback3(const sensor_msgs::ImageConstPtr& msg)
{
  SDL_FreeSurface(imagen2);
  cvReleaseImage(&destination);
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::CvBridge bridge;
   IplImage buf;
   cv::Mat img;
   cv::HOGDescriptor hog;
  try
  {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);  
         cv::imshow("view", cv_ptr->image);
	buf=cv_ptr->image;
	SDL_Rect dest;
	dest.x = 560;  
	dest.y = 50;  
    double percent=110.00;
    //double percent=50.00;
   destination = cvCreateImage(cvSize((int)((buf.width)) ,(int)(buf.height)),buf.depth, buf.nChannels);
     cvResize(&buf, destination,CV_INTER_AREA);
//    imagen=ipl_to_surface(rotate(destination, -90.00));
    imagen2=ipl_to_surface(destination,1);
      if(retro!=NULL)
        SDL_BlitSurface (imagen2,NULL, caratula, &dest );	

   
 // 
//  cvReleaseImage(buf);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
}

//This function is called everytime a new image is published
void imageCallback2(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
        SDL_FreeSurface(retro);
        cvReleaseImage(&destination2);
	cv_bridge::CvImagePtr cv_ptr;
	IplImage hsv_img, buf;
	CvMoments moments;
	CvPoint nuevo=CvPoint();
	cv::Mat hsv_img2;
        SDL_Rect dest;
	dest.x = 370;
        dest.y = 50;	
	try
	{

		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);

		buf=cv_ptr->image;

	}
		catch (cv_bridge::Exception& e)
		{
			//if there is an error during conversion, display it

			ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}
              double percent=1.00;
              destination2 = cvCreateImage(cvSize((int)(buf.width-455),(int)(buf.height-410)),buf.depth, buf.nChannels);
     cvResize(&buf, destination2);
       retro=ipl_to_surface(destination2,1);
       if(retro!=NULL)
        SDL_BlitSurface (retro,NULL, caratula, &dest );
      	cv::waitKey(3);
        pub2.publish(cv_ptr->toImageMsg());   
          return;

}

int main(int argc, char **argv)
{
  
 
  ros::init(argc, argv, "CALL_PLANNER");
  ros::NodeHandle nh,nh2, nh_,_nh;
  ros::Publisher cmd_vel_pub_;
  
  geometry_msgs::Twist base_cmd;
  SDL_Joystick *joystick;
  //Suscribirse al canal que envia la informacion de camara de webots
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/webot/camera", 1, imageCallback);
///////////////////////////////////////////////////////////////
  image_transport::ImageTransport it2(_nh);
   //OpenCV HighGUI call to create a display window on start-up.
   cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
   image_transport::Subscriber sub2 = it2.subscribe("/camera/rgb/image_color", 1, imageCallback2);
////////////////////////////////////////////////////////////////////////////////////

  image_transport::ImageTransport it3(nh2);
  image_transport::Subscriber sub3 = it3.subscribe("/webot/camera2", 1, imageCallback3);
  //Crea canal para la camara web

    cv::destroyWindow(WINDOW);
    pub2 = it2.advertise("camera/image_processed", 1); 
   //crear el canal que publicara la informacion del timon
   cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/auto_controller/command", 1);
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK)<0) {
    cout << "Failed SDL_Init " << SDL_GetError() << endl;
    return 1;
  }
   
  screen=SDL_SetVideoMode(1280,1280,32,SDL_HWSURFACE | SDL_DOUBLEBUF | SDL_RESIZABLE );  
  SDL_JoystickEventState(SDL_ENABLE);
  joystick = SDL_JoystickOpen(0);
  int number_of_buttons;
  number_of_buttons = SDL_JoystickNumButtons(joystick);

  if(screen==NULL) {
    cout << "Failed SDL_SetVideoMode: " << SDL_GetError() << endl;
    SDL_Quit();
    return 1;
  }  
   SDL_Event event;
   for(;;) {


   
    SDL_Flip(screen);   
    ros::spinOnce();
    base_cmd.angular.z = 0;
    base_cmd.linear.x = 0;  
        while(SDL_PollEvent(&event))
    {  
        switch(event.type)
        {  
             case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
         //  printf("%d\n",event.jbutton.button);
             if ( event.jbutton.button == 0 ) 
             {
                      base_cmd.linear.x = 1;
                      printf("Acelerar\n");

             }
             if ( event.jbutton.button == 1 ) 
            {
               base_cmd.linear.x = -1;
               printf("Frenar\n");
            }
            if(event.jbutton.button==9)
                     {
                      base_cmd.angular.z = 1;
			printf("Derecha\n");
                      }
            if(event.jbutton.button==6)
                     {
                      base_cmd.linear.x = 5;
			printf("Reversa\n");
                      }
             if(event.jbutton.button==8)
                     {
                      base_cmd.angular.z = -1;
			printf("Izquierda\n");
                      }
             break;
             case SDL_KEYDOWN:
                        switch (event.key.keysym.sym) {
                                case SDLK_q:
                                        printf("Bye Bye!\n");                                        
                                        exit(0);

                                case SDLK_SPACE:
                                        break;

                                default:
                                        break;
                        }
            case SDL_QUIT:
            exit(0);
            break;
        }
    }
     
  cmd_vel_pub_.publish(base_cmd);
  }
  SDL_Quit();
 
  cvDestroyWindow("view");
}
