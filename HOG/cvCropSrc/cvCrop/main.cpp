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
SDL_Surface *screen,*retro,*imagen,*imagen2,*caratula=IMG_Load("/root/research/HOG/cvCropSrc/cvCrop/imagen.png");	 
SDL_Surface* optimizedImage = NULL;
IplImage *destination;
IplImage *destination2;
IplImage* cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageCropped;
	CvSize size;

	if (img->width <= 0 || img->height <= 0
		|| region.width <= 0 || region.height <= 0) {
		//cerr << "ERROR in cropImage(): invalid dimensions." << endl;
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) {
		//cerr << "ERROR in cropImage(): image depth is not 8." << endl;
		exit(1);
	}

	// Set the desired region of interest.
	cvSetImageROI((IplImage*)img, region);
	// Copy region of interest into a new iplImage and return it.
	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageCropped);	// Copy just the region.

	return imageCropped;
}

IplImage* resizeImage(const IplImage *origImg, int newWidth,
	int newHeight, bool keepAspectRatio)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0
		|| origWidth <= 0 || origHeight <= 0) {
		//cerr << "ERROR: Bad desired image size of " << newWidth
		//	<< "x" << newHeight << " in resizeImage().\n";
		exit(1);
	}

	if (keepAspectRatio) {
		// Resize the image without changing its aspect ratio,
		// by cropping off the edges and enlarging the middle section.
		CvRect r;
		// input aspect ratio
		float origAspect = (origWidth / (float)origHeight);
		// output aspect ratio
		float newAspect = (newWidth / (float)newHeight);
		// crop width to be origHeight * newAspect
		if (origAspect > newAspect) {
			int tw = (origHeight * newWidth) / newHeight;
			r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
		}
		else {	// crop height to be origWidth / newAspect
			int th = (origWidth * newHeight) / newWidth;
			r = cvRect(0, (origHeight - th)/2, origWidth, th);
		}
		IplImage *croppedImg = cropImage(origImg, r);

		// Call this function again, with the new aspect ratio image.
		// Will do a scaled image resize with the correct aspect ratio.
		outImg = resizeImage(croppedImg, newWidth, newHeight, false);
		cvReleaseImage( &croppedImg );

	}
	else {

		// Scale the image to the new dimensions,
		// even if the aspect ratio will be changed.
		outImg = cvCreateImage(cvSize(newWidth, newHeight),
			origImg->depth, origImg->nChannels);
		if (newWidth > origImg->width && newHeight > origImg->height) {
			// Make the image larger
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_LINEAR: good at enlarging.
			// CV_INTER_CUBIC: good at enlarging.			
			cvResize(origImg, outImg, CV_INTER_LINEAR);
		}
		else {
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_AREA: good at shrinking (decimation) only.
			cvResize(origImg, outImg, CV_INTER_AREA);
		}

	}
	return outImg;
}

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

IplImage *rotatedImage = cvCreateImage(cvSize(2600, 1024), IPL_DEPTH_8U,image->nChannels);

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
	buf=cv_ptr->image;
	SDL_Rect dest;
	dest.x = 0;  
	dest.y = 0;  
    double percent=110.00;
    destination = cvCreateImage(cvSize((int)(1300),(int)(768)),buf.depth, buf.nChannels);
    cvResize(&buf, destination);
    imagen=SDL_DisplayFormat(ipl_to_surface( destination,1));
   // cvReleaseImage(&destination);
    SDL_BlitSurface (imagen,NULL, screen, &dest );
	dest.x = 550;  
    SDL_BlitSurface (caratula,NULL, screen, &dest );
  //  SDL_FreeSurface(imagen);
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
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
    buf=cv_ptr->image;
	SDL_Rect dest;
	dest.x = 560;  
	dest.y = 50;  
    double percent=110.00;
   destination = cvCreateImage(cvSize((int)(buf.width) ,(int)(buf.height)),buf.depth, buf.nChannels);
   cvResize(&buf, destination);
   imagen2=SDL_DisplayFormat(ipl_to_surface(destination,1));
   //cvReleaseImage(&destination);
   if(retro!=NULL)
      SDL_BlitSurface (imagen2,NULL, caratula, &dest );
   //SDL_FreeSurface(imagen2);	
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
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
		//	ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}
     double percent=1.00;
     destination2 = cvCreateImage(cvSize((int)(buf.width-450),(int)(buf.height-410)),buf.depth, buf.nChannels);
     cvResize(&buf, destination2 );
     retro=SDL_DisplayFormat(ipl_to_surface(/*resizeImage(*/destination2/*, 50, 50, false)*/,1));
   //  cvReleaseImage(&destination2);
     if(retro!=NULL)
        SDL_BlitSurface (retro,NULL, caratula, &dest );
     //pub2.publish(cv_ptr->toImageMsg());
     // SDL_FreeSurface(retro);
  
     return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Controlador");
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
   
  screen=SDL_SetVideoMode(1300,768,32,SDL_HWSURFACE); //| SDL_DOUBLEBUF Le he quitado porque indica de disimunuye el performance  
  SDL_JoystickEventState(SDL_ENABLE);
  joystick = SDL_JoystickOpen(0);
  int number_of_buttons;
  number_of_buttons = SDL_JoystickNumButtons(joystick);

  if(screen==NULL) {
    cout << "Failed SDL_SetVideoMode: " << SDL_GetError() << endl;
    SDL_Quit();
    return 1;
  }  
    printf("\n Evento tipo:SDL_ACTIVEEVENT  %d",SDL_ACTIVEEVENT);
    printf("\n Evento tipo: %d",SDL_KEYDOWN);
    printf("\n Evento tipo: SDL_KEYDOWN %d",SDL_MOUSEMOTION );
    printf("\n Evento tipo: SDL_MOUSEBUTTONDOWN %d",SDL_MOUSEBUTTONDOWN);
    printf("\n Evento tipo: SDL_JOYAXISMOTION %d",SDL_JOYAXISMOTION);
    printf("\n Evento tipo: SDL_JOYBALLMOTION %d",SDL_JOYBALLMOTION);
    printf("\n Evento tipo: SDL_JOYHATMOTION %d",SDL_JOYHATMOTION);
    printf("\n Evento tipo: SDL_JOYBUTTONDOWN %d",SDL_JOYBUTTONDOWN);
    printf("\n Evento tipo: SDL_SYSWMEVENT %d",SDL_SYSWMEVENT);
    printf("\n Evento tipo: SDL_USEREVENT %d",SDL_USEREVENT);
   SDL_Event event;
   int veloc=0;
   int accel=0;
   base_cmd.angular.z = 0;
   base_cmd.linear.x = 0; 
   for(;;) {
   SDL_EnableKeyRepeat(10, 1);
    SDL_Flip(screen);   
    ros::spinOnce();
    while(SDL_PollEvent(&event))
    {   
        switch(event.type)
        {  	 
			 case SDL_JOYBUTTONDOWN:
			 base_cmd.linear.y = event.jbutton.button;
			 break;
             case SDL_JOYAXISMOTION:  
             switch(event.jaxis.axis)
             {
				 case 0:  				         
				         base_cmd.angular.z = event.jaxis.value;				              						           
				         break;
				 case 1: //printf("embrague");
				          base_cmd.angular.y = event.jaxis.value*-1;				              		
				          break;
				 case 2: 
				          //printf("Acelerador");				          
                          base_cmd.linear.x = event.jaxis.value;				              				
				          break;
				 case 3: 
				         //printf("freno");
				         base_cmd.angular.x = event.jaxis.value*-1;				              		
				         base_cmd.linear.x = 0;
				         break;				 
			 }
             break;
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
