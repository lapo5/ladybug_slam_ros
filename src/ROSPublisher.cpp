//
// Created by sebastiano on 8/18/16.
//

#include <include/ROSPublisher.h>
#include "FrameDrawer.h"
#include "Tracking.h"
#include "LoopClosing.h"
#include "System.h"

#include <thread>
#include <sstream>
#include <cassert>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>

#include <ladybug_msgs/ORBState.h>
#include <cv_bridge/cv_bridge.h>

#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <chrono>
#include <stdint.h>

using namespace Ladybug_SLAM;

ROSPublisher::ROSPublisher(Map *map, System* system, double frequency, ros::NodeHandle nh) :
    IMapPublisher(map),
    drawer_(system->GetFrameDrawer()),
    nh_(std::move(nh)),
    pub_rate_(frequency),
    lastBigMapChange_(-1),
    octomap_tf_based_(false),
    octomap_(PublisherUtils::getROSParam<float>(nh, "/ladybug_slam/octomap/resolution", 0.1)),
    pointcloud_chunks_stashed_(0),
    clear_octomap_(false),
    localize_only(false),
    map_scale_(1.50),
    perform_scale_correction_(true),
    scaling_distance_(1.00),
    camera_height_(0.205),
    camera_height_mult_(1.0),
    camera_height_corrected_(camera_height_*camera_height_mult_),
    publish_octomap_(false)
{

    initializeParameters(nh);
    orb_state_.state = ladybug_msgs::ORBState::UNKNOWN;

    SetSystem(system);

    // initialize publishers
    map_pub_            = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_map", 5);
    image_pub_          = nh_.advertise<sensor_msgs::Image>("frame", 5);
    state_pub_          = nh_.advertise<ladybug_msgs::ORBState>("info/state", 10);
    state_desc_pub_     = nh_.advertise<std_msgs::String>("info/state_description", 10);
    kp_pub_             = nh_.advertise<std_msgs::UInt32>("info/frame_keypoints", 1);
    kf_pub_             = nh_.advertise<std_msgs::UInt32>("info/map_keyframes", 1);
    mp_pub_             = nh_.advertise<std_msgs::UInt32>("info/matched_points", 1);
    loop_close_pub_     = nh_.advertise<std_msgs::Bool>("info/loop_closed", 2);
    trajectory_pub_     = nh_.advertise<nav_msgs::Path>("cam_path", 2);

    // initialize subscribers
    clear_path_sub_ = nh_.subscribe("clear_cam_path", 1, &ROSPublisher::clearCamTrajectoryCallback, this);

    if (octomap_enabled_)
    {
      if ( publish_octomap_ ) {
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 3);
      }
    }

    // used because of scale correction coefficient non-ideal estimation
    camera_height_corrected_ = camera_height_ * camera_height_mult_;
    static nav_msgs::Path msg;
    msg.poses.clear();

}

void ROSPublisher::initializeParameters(ros::NodeHandle &nh) {

  // freq and image_topic are defined in ros_mono.cc
  nh.param<float>("/ladybug_slam/topic/orb_state_republish_rate", orb_state_republish_rate_, 1);

  // odom topic defined in ScaleCorrector.cpp
  nh.param<bool>("/ladybug_slam/map_scale/perform_correction",        perform_scale_correction_,  false);
  nh.param<float>("/ladybug_slam/map_scale/scaling_distance",         scaling_distance_,          1.000);
  nh.param<float>("/ladybug_slam/map_scale/set_manually",             map_scale_,                 1.000);
  nh.param<float>("/ladybug_slam/map_scale/camera_height",            camera_height_,             0.205);
  nh.param<float>("/ladybug_slam/map_scale/camera_height_multiplier", camera_height_mult_,        1.000);

  nh.param<std::string>("/ladybug_slam/frame/map_frame",          map_frame_,          ROSPublisher::DEFAULT_MAP_FRAME);
  nh.param<std::string>("/ladybug_slam/frame/camera_frame",       camera_frame_,       ROSPublisher::DEFAULT_CAMERA_FRAME);
  nh.param<std::string>("/ladybug_slam/frame/base_frame",         base_frame_,         "/ladybug_slam/base_link");
  nh.param<std::string>("/ladybug_slam/frame/world_frame",        world_frame_,        "/ladybug_slam/world_frame");

  nh.param<bool>("/ladybug_slam/octomap/enabled",                octomap_enabled_,        false);
  nh.param<bool>("/ladybug_slam/octomap/publish_octomap",        publish_octomap_,        false);

  nh.param<bool>("use_semi_dense_reconstruction",  use_semi_dense_reconstruction_, true);

  nh.param<bool>("/ladybug_slam/octomap/rebuild",  octomap_rebuild_, false);
  nh.param<float>("/ladybug_slam/octomap/rate",    octomap_rate_,    1.0);
  // resolution is set default in constructor

  nh.param<double>("/ladybug_slam/occupancy/projected_map/min_height", projection_min_height_,  -10.0);
  nh.param<double>("/ladybug_slam/occupancy/projected_map/max_height", projection_max_height_,  +10.0);

  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/erode_se_size",  erode_se_size_,  3);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/erode_nb",       erode_nb_,       1);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/open_se_size",   open_se_size_,   3);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/open_nb",        open_nb_,        1);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/close_se_size",  close_se_size_,  3);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/close_nb",       close_nb_,       1);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/erode2_se_size", erode2_se_size_, 3);
  nh.param<int>   ("/ladybug_slam/occupancy/projected_map/morpho_oprations/erode2_nb",      erode2_nb_,      1);

  std::cout << endl;
  std::cout << "ROS Publisher parameters" << endl;
  std::cout << "TOPIC" << endl;
  std::cout << "- orb_state_republish_rate:  " << orb_state_republish_rate_ << std::endl;
  std::cout << "MAP SCALE" << endl;
  std::cout << "- perform_correction:  " << perform_scale_correction_ << std::endl;
  std::cout << "- set_manually:  " << map_scale_ << std::endl;
  std::cout << "- camera_height:  " << camera_height_ << std::endl;
  std::cout << "- camera_height_multiplier:  " << camera_height_mult_ << std::endl;
  std::cout << "FRAME" << endl;
  std::cout << "- map_frame:  " << map_frame_ << std::endl;
  std::cout << "- camera_frame:  " << camera_frame_ << std::endl;
  std::cout << "- base_frame:  " << base_frame_ << std::endl;
  std::cout << "- world_frame:  " << world_frame_ << std::endl;
  std::cout << "SEMI-DENSE RECONSTRUCTION" << endl;
  std::cout << "- use_semi_dense_reconstruction:  " << use_semi_dense_reconstruction_ << std::endl;
  std::cout << "OCTOMAP" << endl;
  std::cout << "- octomap/enabled:  " << octomap_enabled_ << std::endl;
  std::cout << "- octomap/publish_octomap:  " << publish_octomap_ << std::endl;
  std::cout << "- octomap/rebuild:  " << octomap_rebuild_ << std::endl;
  std::cout << "- octomap/rate:  " << octomap_rate_ << std::endl;
  std::cout << "OCCUPANCY/PROJECTED_MAP" << endl;
  std::cout << "- projected_map/min_height:  " << projection_min_height_ << std::endl;
  std::cout << "- projected_map/max_height:  " << projection_max_height_ << std::endl;
  std::cout << "OCCUPANCY/PROJECTED_MAP/MORPHO" << endl;
  std::cout << "- open_se_size:  " << open_se_size_ << std::endl;
  std::cout << "- open_nb:  " << open_nb_ << std::endl;
  std::cout << "- close_se_size:  " << close_se_size_ << std::endl;
  std::cout << "- close_nb:  " << close_nb_ << std::endl;
  std::cout << "- erode_se_size:  " << erode_se_size_ << std::endl;
  std::cout << "- erode_nb:  " << erode_nb_ << std::endl;
  std::cout << endl;

  // DEPRECATED
  // nh.param<bool>("/ladybug_slam/octomap/tf_based", octomap_tf_based_, false);
  // nh.param<bool>("/ladybug_slam/frame/align_map_to_cam_frame",   align_map_to_cam_frame_, true);
  // nh.param<bool>("/ladybug_slam/frame/adjust_map_frame",          adjust_map_frame_,      false);
  // nh.param<float>("/ladybug_slam/topic/loop_close_republish_rate_", loop_close_republish_rate_, ROSPublisher::LOOP_CLOSE_REPUBLISH_RATE);

}

/*
 * Either appends all GetReferenceMapPoints to the pointcloud stash or clears the stash and re-fills it
 * with GetAllMapPoints, in case there is a big map change in Ladybug_SLAM or all_map_points is set to true.
 */
void ROSPublisher::stashMapPoints(bool all_map_points)
{
    std::vector<MapPoint*> map_points;

    pointcloud_map_points_mutex_.lock();

    if (all_map_points || GetMap()->GetLastBigChangeIdx() > lastBigMapChange_)
    {
        map_points = GetMap()->GetAllMapPoints();
        lastBigMapChange_ = GetMap()->GetLastBigChangeIdx();
        clear_octomap_ = true;
        pointcloud_map_points_.clear();
        pointcloud_chunks_stashed_ = 1;

    } else {

        map_points = GetMap()->GetReferenceMapPoints();
        pointcloud_chunks_stashed_++;
    }

    for (MapPoint *map_point : map_points) {
        if (map_point->isBad()) {
            continue;
        }
        cv::Mat pos = map_point->GetWorldPos();
        PublisherUtils::transformPoint(pos, map_scale_, true, 1, camera_height_corrected_);
        pointcloud_map_points_.push_back(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }

    pointcloud_map_points_mutex_.unlock();
}

/*
 * Octomap worker thread function, which has exclusive access to the octomap. Updates and publishes it.
 */

void ROSPublisher::octomapWorker()
{

    static std::chrono::system_clock::time_point this_cycle_time;

    octomap::pose6d frame;
    octomap::point3d origin = { 0.0, 0.0, 0.0 };
    bool got_tf = false;

    // wait until Ladybug_SLAM is up and running
    ROS_INFO("octomapWorker thread: waiting for ORBState OK");

    while (GetSystem() != nullptr && !GetSystem()->isFinished() && orb_state_.state != ladybug_msgs::ORBState::OK)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    ROS_INFO("octomapWorker thread: starting to work (ORBState is OK)");

    // main thread loop
    while (GetSystem() != nullptr && !GetSystem()->isFinished() && !isStopped())
    {

        this_cycle_time = std::chrono::system_clock::now();

        if ( !got_tf ) {

          try {

            tf::StampedTransform transform_in_target_frame;
            tf_listener_.waitForTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            tf_listener_.lookupTransform( base_frame_, camera_frame_, ros::Time(0), transform_in_target_frame);
            static const tf::Transform octomap = PublisherUtils::createTF(tf::Vector3(tfScalar(0.0),
                                                                                      tfScalar(0.0),
                                                                                      tfScalar(camera_height_corrected_)),
                                                                          transform_in_target_frame.getRotation() );
            frame = octomap::poseTfToOctomap(octomap);
            got_tf = true;

          } catch (tf::TransformException &ex) {

            frame = octomap::pose6d(0, 0, 0, 0, 0, 0);
            got_tf = false;

          }

        }

        if (got_tf || octomap_rebuild_ )
        {
          // clear whenever TF mode changes
          clear_octomap_ |= (got_tf != octomap_tf_based_);

          if (clear_octomap_)
          {
            // WARNING: causes ugly segfaults in octomap 1.8.0
            octomap_.clear();
            ROS_INFO("octomapWorker: octomap cleared, rebuilding...");

            stashMapPoints(true);     // stash whole map
            clear_octomap_ = false;   // TODO: mutex?
          }

          pointcloud_map_points_mutex_.lock();
          octomap_.insertPointCloud(pointcloud_map_points_, origin, frame);

          pointcloud_map_points_.clear();
          int pointcloud_chunks_stashed = pointcloud_chunks_stashed_;
          pointcloud_chunks_stashed_ = 0;
          pointcloud_map_points_mutex_.unlock();

          octomap_tf_based_ = got_tf;

          if ( publish_octomap_ ) {
            ROS_INFO("Publishing Octomap...");
            publishOctomap();
            //ROS_INFO("Octomap published");
          }

          ROS_INFO("octomapWorker: finished cycle integrating %i pointcloud chunks.", pointcloud_chunks_stashed);
        }
        else
        {

          ROS_INFO("octomapWorker thread: missing camera TF, losing %i pointcloud chunks.", pointcloud_chunks_stashed_);
          pointcloud_map_points_mutex_.lock();
          pointcloud_map_points_.clear();
          pointcloud_chunks_stashed_ = 0;
          pointcloud_map_points_mutex_.unlock();

        }

        std::this_thread::sleep_until(this_cycle_time + std::chrono::milliseconds((int) (1000. / octomap_rate_)));

    }

    ROS_WARN("octomapWorker thread: stopped");
}

/*
 * Publishes Ladybug_SLAM GetAllMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMap()
{
    sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(GetMap()->GetAllMapPoints());
    msg.header.frame_id = map_frame_;
    map_pub_.publish(msg);
}

/*
 * Publishes Ladybug_SLAM GetReferenceMapPoints() as a PointCloud2.
 */
void ROSPublisher::publishMapUpdates()
{
    sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(GetMap()->GetReferenceMapPoints());
    msg.header.frame_id = map_frame_;
    map_pub_.publish(msg);
}

/*
 * Publishes Ladybug_SLAM GetCameraPose() as a TF.
 */
void ROSPublisher::publishCameraPose()
{

    // number of subscribers is unknown to a TransformBroadcaster
    cv::Mat xf = PublisherUtils::computeCameraTransform(GetCameraPose(), map_scale_);

    if (!xf.empty()) {
      try {

          camera_position_ = {  xf.at<float>(0, 3),
                                xf.at<float>(1, 3),
                                xf.at<float>(2, 3) };

          tf::Quaternion orientation = PublisherUtils::convertToQuaternion<tf::Quaternion>(xf);

          /* ------------------------------------
           * Camera's pose in map coordinate system
           * divided into translation and rotation
           */
          tf::Transform Tmc = PublisherUtils::createTF(camera_position_,
                                                       orientation);

          tf::StampedTransform transform(Tmc, ros::Time::now(), map_frame_,  world_frame_);
          camera_tf_pub_.sendTransform(transform);

          // camera trajectory extraction
          cam_pose_ = PublisherUtils::getPoseStamped(&Tmc, &camera_frame_);

          ROS_INFO("publishCameraPose()");


      } catch (tf::TransformException &ex) {
          ROS_ERROR("publishCameraPose error: %s",ex.what());
          ros::Duration(3.0).sleep();
      }
    }
}

/*
 * Publishes the previously built Octomap. (called from the octomap worker thread)
 */
void ROSPublisher::publishOctomap()
{
    auto t0 = std::chrono::system_clock::now();
    octomap_msgs::Octomap msgOctomap;
    msgOctomap.header.frame_id = map_frame_;

    msgOctomap.header.stamp = ros::Time::now();
    if (octomap_msgs::binaryMapToMsg(octomap_, msgOctomap))   // TODO: full/binary...?
    {
        auto tn = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
        //std::cout << "msg generation time: " << dt.count() << " ms" << std::endl;
        t0 = std::chrono::system_clock::now();
        octomap_pub_.publish(msgOctomap);
        tn = std::chrono::system_clock::now();
        dt = std::chrono::duration_cast<std::chrono::milliseconds>(tn - t0);
        //std::cout << "msg publish time: " << dt.count() << " ms" << std::endl;
    }
}

/*
 * Publishes the Ladybug_SLAM tracking state as ORBState int and/or as a description string.
 */
void ROSPublisher::publishState(Tracking *tracking)
{

    if (tracking != NULL) {
        // save state from tracking, even if there are no subscribers
        orb_state_ = PublisherUtils::toORBStateMessage(tracking->mState);
    
        if(tracking->mState == Ladybug_SLAM::Tracking::LOST)
        {
            clear_path_ = true;
        }
        // publish state as ORBState int
        orb_state_.header.stamp = ros::Time::now();
        state_pub_.publish(orb_state_);

        // publish state as string
        std_msgs::String state_desc_msg;
        // const char* test = PublisherUtils::stateDescription(orb_state_);
        state_desc_msg.data = PublisherUtils::stateDescription(orb_state_); // stateDescription(orb_state_);
        state_desc_pub_.publish(state_desc_msg);

        // last_state_publish_time_ = ros::Time::now();
    }
}

void ROSPublisher::publishCamTrajectory() {

  static nav_msgs::Path msg;

  if ( clear_path_ ) {
    msg.poses.clear();
    clear_path_ = false;
  }

  msg.header.frame_id = map_frame_;
  msg.header.stamp = ros::Time::now();
  msg.poses.push_back(cam_pose_);
  trajectory_pub_.publish(msg);

}

void ROSPublisher::publishLoopState(const bool &state) {
  std_msgs::Bool msg;
  msg.data = state;
  loop_close_pub_.publish(msg);
}

void ROSPublisher::publishUInt32Msg(const ros::Publisher &pub, const unsigned long &data) {
  std_msgs::UInt32 msg;
  msg.data = data;
  pub.publish(msg);
}

/*
 * Publishes the current Ladybug_SLAM status image.
 */
void ROSPublisher::publishImage()
{
    std_msgs::Header hdr;

    sensor_msgs::ImagePtr cv_ptr0 = cv_bridge::CvImage(hdr, "rgb8", drawer_->DrawFrame(true)).toImageMsg();

    image_pub_.publish(*cv_ptr0);
    
}

void ROSPublisher::SemiDenseUpdater() {

  // these operations are moved from Run() to separate thread, it was crucial in my application
  // to get camera pose updates as frequently as possible

  std::chrono::system_clock::time_point a = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point b = std::chrono::system_clock::now();

  Map* map = GetMap();
  ROS_WARN("SemiDenseUpdater started");
  while (GetSystem() != nullptr && !GetSystem()->isFinished()) {

            // Maintain designated frequency of 1 Hz (1000 ms per frame)
        a = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> work_time = a - b;

        if (work_time.count() < 1000.0)
        {
            std::chrono::duration<double, std::milli> delta_ms(1000.0 - work_time.count());
            auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
        }

        b = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> sleep_time = b - a;

      if(map == nullptr)
          break;


      vector<Ladybug_SLAM::MapPoint*> points = map->GetAllMapSemiDensePoints();

      if(points.size() > 0){
          ROS_WARN("SemiDenseUpdater update");
          sensor_msgs::PointCloud2 msg = PublisherUtils::convertToPCL2(points);
          msg.header.frame_id = map_frame_;
          map_pub_.publish(msg);
      }
    
  }
  ROS_WARN("SemiDenseUpdater finished");
  SetFinish(true);


}

void ROSPublisher::Run()
{
    using namespace std::this_thread;
    using namespace std::chrono;

    ROS_INFO("ROS publisher started");

    if (octomap_enabled_) {
      ROS_WARN("octomap_worker_thread_ started...");
      octomap_worker_thread_ = std::thread( [this] { octomapWorker(); } );
    }

    if(use_semi_dense_reconstruction_){
      ROS_WARN("semi_dense_reconstruction_thread started...");
      semi_dense_thread_ = std::thread( [this] { SemiDenseUpdater(); } );
    }

    SetFinish(false);
    
    while (WaitCycleStart()) {

        // only publish map, map updates and camera pose, if camera pose was updated
        if (isCamUpdated()) {

            static ros::Time last_camera_update;
            last_camera_update = ros::Time::now();

            publishCameraPose();

            publishCamTrajectory();
            
            if ( ros::Time::now() >= (last_state_publish_time_ +
                ros::Duration(1. / orb_state_republish_rate_)) )
            {
              // it's time to re-publish info
              publishUInt32Msg(kf_pub_, drawer_->GetKeyFramesNb());
              publishUInt32Msg(kp_pub_, drawer_->GetKeypointsNb());
              publishUInt32Msg(mp_pub_, drawer_->GetMatchedPointsNb());
              publishLoopState(GetSystem()->GetLoopClosing()->isRunningGBA()); // GBA is quite time-consuming task so it will probably be detected here
              last_state_publish_time_ = ros::Time::now();
            }

            publishMap();
            publishMapUpdates();
            
            if (octomap_enabled_)
            {
              // stashMapPoints(); // store current reference map points for the octomap worker
              stashMapPoints(false);
            }
            ResetCamFlag();
        }
    }

    ROS_INFO("ROS publisher finished");
    SetFinish(true);

    if(use_semi_dense_reconstruction_){
      semi_dense_thread_.join();
    }

    if (octomap_enabled_) {
      octomap_worker_thread_.join();
    }

    
}

bool ROSPublisher::WaitCycleStart()
{
    if (GetSystem() == nullptr || GetSystem()->isFinished())
        return false;
    pub_rate_.sleep();
    return true;
}

void ROSPublisher::Update(Tracking *tracking)
{
    if (tracking == nullptr)
        return;
    publishState(tracking);

    publishImage();
}

void ROSPublisher::clearCamTrajectoryCallback(const std_msgs::Bool::ConstPtr& msg) {
  if ( msg->data == true ) {
    clear_path_ = true;
  } else if ( msg->data == false ) {
    clear_path_ = false;
  }
}
