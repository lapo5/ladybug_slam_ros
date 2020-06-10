#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <time.h>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Optimizer.h>
#include <System.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "utilities.h"

#include "ladybug_msgs/PanoImage.h"

#include "ORBVocabulary.h"

#include <include/ROSPublisher.h>

#include <boost/thread/thread.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace ros;
using namespace Ladybug_SLAM;


string strInnFilePath="/CalibrationResult/InnPara.txt";
string strExFilePath="/CalibrationResult/ExPara.txt";
string strInvInnFilePath="/CalibrationResult/InvInnPara.txt";

static const std::string OPENCV_WINDOW = "Image window";

static const int ImgWidth = 1232;
static const int ImgHeight = 1616;

string dataset_path = "/home/marco/Desktop/KASHIWA/";
string orb_voc_path = "/home/marco/Desktop/Ladybug_SLAM_ws/src/ladybug_slam_lib/Vocabulary/ORBvoc.bin";
string yaml_path = "/home/marco/Desktop/Ladybug_SLAM_ws/src/ladybug_slam_lib/Ladybug/Ladybug_SLAM.yaml";

static double frequency_octmap = 1.0;

class SLAM_Monitor
{
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  
  Ladybug_SLAM::System* SLAM;
  ROSPublisher* octmap_publisher;

  tf::TransformBroadcaster broadcaster;
  tf::Transform change_frame;
  tf::Quaternion frame_rotation;

public:
  SLAM_Monitor(Ladybug_SLAM::System* slam_ptr, ROSPublisher* octmap_publisher_ptr)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = nh_.subscribe("/pano_image", 10, &SLAM_Monitor::backCB, this);

    SLAM = slam_ptr;
    octmap_publisher = octmap_publisher_ptr;
  }

  void backCB(const ladybug_msgs::PanoImage& msg)
  {

    vector<cv::Mat> images(5);
    
    cv_bridge::CvImagePtr cv_ptr0;
    cv_ptr0 = cv_bridge::toCvCopy(msg.img0, sensor_msgs::image_encodings::RGB8);
    images[0]=cv_ptr0->image;

    cv_bridge::CvImagePtr cv_ptr1;
    cv_ptr1 = cv_bridge::toCvCopy(msg.img1, sensor_msgs::image_encodings::RGB8);
    images[1]=cv_ptr1->image;

    cv_bridge::CvImagePtr cv_ptr2;
    cv_ptr2 = cv_bridge::toCvCopy(msg.img2, sensor_msgs::image_encodings::RGB8);
    images[2]=cv_ptr2->image;

    cv_bridge::CvImagePtr cv_ptr3;
    cv_ptr3 = cv_bridge::toCvCopy(msg.img3, sensor_msgs::image_encodings::RGB8);
    images[3]=cv_ptr3->image;

    cv_bridge::CvImagePtr cv_ptr4;
    cv_ptr4 = cv_bridge::toCvCopy(msg.img4, sensor_msgs::image_encodings::RGB8);
    images[4]=cv_ptr4->image;

    cv::Mat Pos = SLAM->TrackPanobyFishEye(msg.ImgStringName, images, msg.tframe);
    octmap_publisher->Update(SLAM->GetTracking());

    if(Pos.cols == 4 && Pos.rows == 4){

/*
        change_frame.setOrigin(tf::Vector3(Pos.at<double>(0, 3), Pos.at<double>(1, 3), Pos.at<double>(2, 3)));

        Eigen::Matrix<float, 3, 3> eigen_mat;
        cv::Mat Rot = Pos.rowRange(0,3).colRange(0,3).clone();
        cv::cv2eigen(Rot, eigen_mat);
        Eigen::Vector3f ea = eigen_mat.eulerAngles(2, 1, 0);

        frame_rotation.setRPY(ea.x(), ea.y(), ea.z()); 
        change_frame.setRotation(frame_rotation);
        broadcaster.sendTransform(tf::StampedTransform(change_frame, ros::Time::now(), "ladybug_slam/camera", "ladybug_slam/odom"));
        ROS_WARN("Setting new pos of the camera");
        */

        octmap_publisher->SetCurrentCameraPose(Pos);
    }
  }
};

void octomap_thread(ROSPublisher* octmap_publisher)
{
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ROS_WARN("Octomap thread start");
  octmap_publisher->Run();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ladybug_slam");
  ros::NodeHandle n;

  myLadybugGeometry LBG;
  LBG.InputImgSize(ImgWidth,ImgHeight);

  cout<<"Reading the Inner Para and the Extern Para..."<<endl;
  LBG.InputSphereRadius(20);
  if(!LBG.ReadInnerParaFromFile(dataset_path+strInnFilePath) || !LBG.ReadInvInnerParaFromFile(dataset_path+strInvInnFilePath) || !LBG.ReadUnitCameraExParaFromFile(dataset_path+strExFilePath)){
      ROS_ERROR("Error Reading parameters from Images Folder");
  }

  cv::Rect rect(0,0,1232,1200);
  LBG.SetRect(rect);

  Ladybug_SLAM::System SLAM(orb_voc_path,yaml_path, LBG, ORB_FEATURE_MATCHER, false);

  SLAM.Start();

	tf::TransformBroadcaster broadcaster;

	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

	tf::Quaternion frame_rotation;
	frame_rotation.setRPY(0, 0, 0); 
	change_frame.setRotation(frame_rotation);
  broadcaster.sendTransform(tf::StampedTransform(change_frame, ros::Time::now(), "/ladybug_slam/world_frame", "/ladybug_slam/camera"));

  ROSPublisher octmap_publisher(SLAM.GetMap(), &SLAM, frequency_octmap,  n);

  SLAM_Monitor slam_monitor(&SLAM, &octmap_publisher);

  // spawn another thread
  boost::thread thread_b(octomap_thread, &octmap_publisher);
  
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); // spin() will not return until the node has been shutdown

  cout << endl << endl << "Finished" << endl;
  SLAM.Shutdown(true);

  // wait the second thread to finish
  thread_b.join();

  return 0;
}

