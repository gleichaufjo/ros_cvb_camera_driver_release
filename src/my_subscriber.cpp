//*****************************************************************************

//                        STEMMER IMAGING AG

//*****************************************************************************
//

// Copyright @ 2019 STEMMER IMAGING AG

//

// Permission is hereby granted, free of charge, to any person obtaining a copy

// of this file and associated documentation files (the ‘Software’), to

// deal in the Software without restriction, including without limitation the

// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or

// sell copies of the Software, and to permit persons to whom the Software is

// furnished to do so, subject to the following conditions:

//

// The above copyright notice and this permission notice shall be included in

// all copies or substantial portions of the Software.

//

// THE SOFTWARE IS PROVIDED ‚AS IS‘, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR

// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,

// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL

// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER

// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,

// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE

// SOFTWARE.


#include <iostream>
#include <cvb/cvb/device_factory.hpp>
#include <cvb/cvb/utilities/system_info.hpp>
#include <cvb/cvb/driver/stream.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

sensor_msgs::CameraInfo                 _camera_info;
//image_transport::CameraPublisher*       _camera_info_pub     = NULL;
camera_info_manager::CameraInfoManager* _camera_info_manager = NULL;

void callback(const sensor_msgs::ImageConstPtr& image)
{
	std::cout << "received an image" << std::endl;
};


sensor_msgs::Image toImageMsg(const Cvb::ImagePtr& cvbImg)
{
  // sensor msg
  sensor_msgs::Image rosImg;
  rosImg.header.frame_id = "camera";
  rosImg.height = cvbImg->Height();
  rosImg.width = cvbImg->Width();
  rosImg.encoding = sensor_msgs::image_encodings::RGB8; // workaround, needs proper encoding values.
  rosImg.is_bigendian = false; // always depends on the pfnc dataformat

  auto dataAccess = cvbImg->Plane(0).LinearAccess();

  rosImg.step = dataAccess.YInc();

//  size_t size = dataAccess.YInc();
  size_t size = rosImg.width * 3 * rosImg.height;
  unsigned char * start = reinterpret_cast<unsigned char*>(dataAccess.BasePtr());
  rosImg.data = std::vector<unsigned char>(start, start + size);
  return rosImg;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "something");
  ros::NodeHandle nh_("~");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  std::cout << "changes here " << std::endl;
  image_transport::Publisher pub = it.advertise("/jai_camera/image_raw", 1);
  ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/jai_camera/camera_info", 1);

    image_transport::Subscriber sub = it.subscribe("/jai_camera/image_raw", 1, callback);

    std::string camera_name;
     std::string camera_info_url;
     nh_.getParam("camera_name", camera_name);
     nh_.getParam("camera_info_url", camera_info_url);

     // initialize CameraInfoManager, providing set_camera_info service for geometric calibration
     // see http://wiki.ros.org/camera_info_manager
     camera_info_manager::CameraInfoManager cinfo_manager(n);
     _camera_info_manager = &cinfo_manager;

     if (!_camera_info_manager->setCameraName(camera_name))
     {
       // GUID is 16 hex digits, which should be valid.
       // If not, use it for log messages anyway.
       ROS_WARN_STREAM("[" << camera_name << "] name not valid" << " for camera_info_manger");
     }

     if (_camera_info_manager->validateURL(camera_info_url))
     {
       if ( !_camera_info_manager->loadCameraInfo(camera_info_url) )
       {
         ROS_WARN( "camera_info_url does not contain calibration data." );
       }
       else if ( !_camera_info_manager->isCalibrated() )
       {
         ROS_WARN( "Camera is not calibrated. Using default values." );
       }
     }
     else
     {
       ROS_ERROR_STREAM_ONCE( "Calibration URL syntax is not supported by CameraInfoManager." );
     }



  try
  {
std::cout << " Try " << std::endl; 
    auto stream = Cvb::DeviceFactory::Open(Cvb::ExpandPath(CVB_LIT("%CVB%/drivers/GenICam.vin")))->Stream();
    stream->Start();

   sensor_msgs::Image image;
   sensor_msgs::Image imageRos;
   cv_bridge::CvImagePtr frame;
   cv::Mat Input;
   cv::Mat cropped;
   cv::Mat Output;
   while(ros::ok())
   {
      // wait for an image with a timeout of 10 seconds
      auto waitResult = stream->WaitFor(std::chrono::milliseconds(10000));
      if (waitResult.Status == Cvb::WaitStatus::Ok)
      {
    	image = toImageMsg(waitResult.Image);
    	frame = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    	Input = frame->image;
    	// remove black stripes from fish eye camera image
    	cv::Rect cropArea(195, 0 , 1546, 1216);
    	cropped = Input(cropArea);
    	cv::resize(cropped, Output, cv::Size(), 1, 1);
    	cv_bridge::CvImage resizeRos;
    	resizeRos.encoding = "rgb8";
    	resizeRos.image = Output;
    	sensor_msgs::ImagePtr imagePtr = resizeRos.toImageMsg();
//    	imagePtr->header.stamp = ros::Time::now(); // NICHT verwenden! Sonst passen timestamps bei camera_info nicht dazu
//
    	resizeRos.toImageMsg(imageRos);
    	_camera_info = _camera_info_manager->getCameraInfo();
    	_camera_info.header = imageRos.header;
//    	_camera_info.header.stamp = ros::Time::now(); // NICHT verwenden! Sonst passen timestamps bei camera_info nicht dazu
    	_camera_info.height = imageRos.height;
    	_camera_info.width = imageRos.width;
    	camera_info_pub.publish(_camera_info);
    	pub.publish(imagePtr);
    	//        pub.publish(toImageMsg(waitResult.Image));
        ros::spinOnce();
      }
      else
      {
        throw std::runtime_error(std::string("cvb acq error ") + std::to_string(static_cast<int>(waitResult.Status)));
      }
    }
    // synchronously stop the stream
    stream->Stop();
  }
  catch (const std::exception& error)
  {
    std::cout << error.what() << std::endl;
  }
}
