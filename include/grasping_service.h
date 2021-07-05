#include "ros/ros.h"
#include "grasping_tp/detect_grasps.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>

#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <gpd/candidate/hand.h>

#include "geometry_msgs/Pose.h"

#include <eigen_conversions/eigen_msg.h>

class GraspService{
    public:
        struct GraspPose{
            geometry_msgs::Pose pre;
            geometry_msgs::Pose actual;
            geometry_msgs::Pose after;
            float distance;
        };

        GraspService(ros::NodeHandle& node);
        bool grasp(grasping_tp::detect_grasps::Request  &req, grasping_tp::detect_grasps::Response &res);
        grasping_tp::GraspConfigList createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header);
        grasping_tp::GraspConfig convertToGraspMsg(const gpd::candidate::Hand& hand);
        GraspPose createPickingEEFPose(grasping_tp::GraspConfig grasp_msg);

    private:
        gpd::GraspDetector* grasp_detector;
        gpd::util::Cloud* cloud_camera;
        tf::TransformListener listener; 
};
