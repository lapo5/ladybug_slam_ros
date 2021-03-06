//
// Created by sebastiano on 8/18/16.
//

#ifndef LADYBUG_SLAM_ROSPUBLISHER_H
#define LADYBUG_SLAM_ROSPUBLISHER_H

/*
#include "IFrameSubscriber.h"
*/
#include "IPublisherThread.h"
#include "IMapPublisher.h"
#include "FrameDrawer.h"
#include "System.h"
#include "Map.h"
#include "LoopClosing.h"
#include "PublisherUtils.h"
#include "ScaleCorrector.h"

#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <octomap/OcTree.h>

#include <ladybug_msgs/ORBState.h>

namespace Ladybug_SLAM
{
    class System;
    class Map;
    class Tracking;
    class LoopClosing;
}

//public Ladybug_SLAM::IFrameSubscriber
class ROSPublisher : 
    public Ladybug_SLAM::IPublisherThread,
    public Ladybug_SLAM::IMapPublisher
{
public:

    static constexpr const char *DEFAULT_MAP_FRAME = "/ladybug_slam/map";
    static constexpr const char *DEFAULT_CAMERA_FRAME = "/ladybug_slam/camera";
    static constexpr const char *DEFAULT_IMAGE_TOPIC = "/ladybug_slam/image";

    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(
                Ladybug_SLAM::Map *map,
                Ladybug_SLAM::System *system,
                double frequency,
                ros::NodeHandle nh = ros::NodeHandle());

    virtual void Run(); //override;
    virtual void Update(Ladybug_SLAM::Tracking*);

protected:

    bool WaitCycleStart();

private:

    void initializeParameters(ros::NodeHandle &nh);
    void stashMapPoints(bool all_map_points = false);
    void octomapWorker();
    void SemiDenseUpdater();

    void updateOctoMap();
    void integrateMapPoints(const std::vector<Ladybug_SLAM::MapPoint*> &, const octomap::point3d &, const octomap::pose6d &, octomap::OcTree &);

    void octomapCutToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& map_erode, const double minZ_, const double maxZ_ );
    void octomapGradientToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, float max_height, int nb_erosions, float low_slope, float high_slope); // void octomapGradientToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, float max_height = GRADIENT_MAX_HEIGHT, int nb_erosions = GRADIENT_NB_EROSIONS, float low_slope = GRADIENT_LOW_SLOPE, float high_slope = GRADIENT_HIGH_SLOPE);

    void publishMap();
    void publishMapUpdates();
    void publishCameraPose();
    void publishOctomap();
    void publishState(Ladybug_SLAM::Tracking *tracking);
    void publishImage();
    void publishProjectedMap();
    void publishGradientMap();
    void publishCamTrajectory();

    Ladybug_SLAM::FrameDrawer* drawer_;

    ros::NodeHandle   nh_;
    ros::Publisher    map_pub_, map_updates_pub_, map_semidense_pub_, image_pub_, odom_pub_,
                      state_pub_, state_desc_pub_, octomap_pub_,
                      projected_map_pub_, projected_morpho_map_pub_, gradient_map_pub_,
                      kf_pub_, kp_pub_, mp_pub_,
                      trajectory_pub_,
                      loop_close_pub_;
    ros::Rate         pub_rate_;
    std::thread       semi_dense_thread_;

    geometry_msgs::PoseStamped cam_pose_;
    ros::Subscriber   clear_path_sub_;
    bool              clear_path_;
    void              clearCamTrajectoryCallback(const std_msgs::Bool::ConstPtr& msg);

    // -------- Mode
    ros::Subscriber   mode_sub_;
    void              localizationModeCallback(const std_msgs::Bool::ConstPtr& msg);
    bool              localize_only;
    void              publishLoopState(const bool &state);

    // -------- Feature Points
    void              publishKeypointsNb(const int &nb);
    void              publishUInt32Msg(const ros::Publisher &pub, const unsigned long &data);

    // -------- TF
    tf::TransformListener     tf_listener_;
    tf::TransformBroadcaster  camera_tf_pub_;
    tf::Vector3               camera_position_;
    float                     camera_height_;
    float                     camera_height_mult_;
    float                     camera_height_corrected_; // separate variable because there would be many * operations with transformPoint function

    int   lastBigMapChange_;
    bool  adjust_map_frame_;
    bool  align_map_to_cam_frame_;
    bool  perform_scale_correction_;
    float scaling_distance_;

    // ------ octomap
    octomap::OcTree octomap_;
    std::thread     octomap_worker_thread_;
    bool            octomap_tf_based_;
    bool            octomap_enabled_;
    bool            octomap_rebuild_;
    bool            clear_octomap_;
    float           octomap_rate_;
    bool            publish_octomap_;
    bool            publish_projected_map_;
    bool            publish_gradient_map_;

    // ------ Semi Dense Reconstruction
    bool use_semi_dense_reconstruction_;

    // ------ PCL
    octomap::Pointcloud pointcloud_map_points_;
    std::mutex          pointcloud_map_points_mutex_;
    int                 pointcloud_chunks_stashed_;
    float               map_scale_;

    // params for z-plane-based occupancy grid approach
    double  projection_min_height_;
    double  projection_max_height_;

    // params for morphological operations
    int  erode_se_size_;
    int  erode_nb_;
    int  open_se_size_;
    int  open_nb_;
    int  close_se_size_;
    int  close_nb_;
    int  erode2_nb_;
    int  erode2_se_size_;

    // params for gradient-based approach
    float   gradient_max_height_;
    float   gradient_low_slope_;
    float   gradient_high_slope_;
    int     gradient_nb_erosions_;

    // params for frames
    std::string map_frame_;
    std::string camera_frame_;
    std::string map_frame_adjusted_;
    std::string base_frame_;
    std::string world_frame_;

    // state republish rate
    ladybug_msgs::ORBState  orb_state_;
    ros::Time               last_state_publish_time_;
    float                   orb_state_republish_rate_;

    // loop closing republish
    std_msgs::Bool          loop_close_state_;
    ros::Time               last_loop_close_publish_time_;
    float                   loop_close_republish_rate_;

};

#endif //LADYBUG_SLAM_ROSPUBLISHER_H
