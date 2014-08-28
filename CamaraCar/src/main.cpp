//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
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

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
	* The name used here must be a base name, ie. it cannot have a / in it.
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
        ros::init(argc, argv, "image_processor");
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
        ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
	//OpenCV HighGUI call to create a display window on start-up.
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	/**
	* Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
	* In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
	* the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
	* subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
	* When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
	*/
        image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
        pub = it.advertise("camera/image_processed", 1);
	/**
	* In this application all user callbacks will be called from within the ros::spin() call.
	* ros::spin() will not return until the node has been shutdown, either through a call
	* to ros::shutdown() or a Ctrl-C.
	*/
        ros::spin();
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
