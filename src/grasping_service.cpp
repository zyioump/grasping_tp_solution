#include <grasping_service.h>

GraspService::GraspService(ros::NodeHandle& node){
  std::string cfg_file;
  node.param("config_file", cfg_file, std::string("/workspace/src/grasping_tp/cfg/gpd_config.cfg"));
  grasp_detector = new gpd::GraspDetector(cfg_file);
  tf::TransformListener listener; 
}

bool GraspService::grasp(grasping_tp::detect_grasps::Request  &req,
         grasping_tp::detect_grasps::Response &res) {
  ROS_INFO("Received service request ...");

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg (req.request.global_cloud, *cloud);
  std::cout << "PointCloud has: " << cloud->size()  << " data points." << std::endl;

  int minX = req.request.bounding_box.x_min.data;
  int minY = req.request.bounding_box.y_min.data;
  int maxX = req.request.bounding_box.x_max.data;
  int maxY = req.request.bounding_box.y_max.data;
  ROS_INFO_STREAM("Received bounding box with min_x: " << minX << " min_y: " << minY << " max_x: " << maxX << " max_y: " << maxY);

  std::vector<int> indices;
  for(int i = minX; i < maxX; i++){
    for(int j = minY; j < maxY; j++){
      indices.push_back(i + (j*640));
    }
  }
  
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (index_ptr);
  extract.setNegative (false); // If set to true, you can extract point clouds outside the specified index

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZRGBA>);
  extract.filter (*cloud_cropped);

  std::cout << "PointCloud after cropping has: " << cloud_cropped->size()  << " data points." << std::endl; 
  
  std::vector<int> ind;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_without_nan (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::removeNaNFromPointCloud(*cloud_cropped, *cloud_without_nan, ind);

  std::cout << "PointCloud after removing NaN has: " << cloud_without_nan->size()  << " data points." << std::endl; 

  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGBA>);

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);
  
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_without_nan);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGBA> extraction;
  extraction.setInputCloud (cloud_without_nan);
  extraction.setIndices (inliers);

  // Remove the planar inliers, extract the rest
  extraction.setNegative (true);
  extraction.filter (*cloud_segmented);
  
  std::cout << "PointCloud after segmentation has : " << cloud_segmented->points.size() << " data points." << std::endl;

  if(cloud_segmented->points.size() == 0){
    ROS_INFO_STREAM("No objects detected, exiting.");
    return false;
  }

  Eigen::Matrix3Xd view_points(3,1);
  cloud_camera = new gpd::util::Cloud(cloud_segmented, 0, view_points);

  // preprocess the point cloud
  try{
    grasp_detector->preprocessPointCloud(*cloud_camera);
  }
  catch(...) {
    ROS_INFO_STREAM("Pre-Processing of the point cloud failed.");
    return false;
  }

  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;
  // detect grasps in the point cloud
  try{
    grasps = grasp_detector->detectGrasps(*cloud_camera);
  }
  catch(...) {
    ROS_INFO_STREAM("No grasp position detected, exiting.");
    return false;
  }

  grasping_tp::GraspConfigList selected_grasps_msg = GraspService::createGraspListMsg(grasps, req.request.global_cloud.header);
  res.grasp_configs = selected_grasps_msg;
  return true;
}

grasping_tp::GraspConfigList GraspService::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header) {
  grasping_tp::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

grasping_tp::GraspConfig GraspService::convertToGraspMsg(const gpd::candidate::Hand& hand) {
  grasping_tp::GraspConfig msg;

  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);

  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  GraspPose grasp_pose;
  grasp_pose = createPickingEEFPose(msg);

  msg.pre_pose = grasp_pose.pre;
  msg.actual_pose = grasp_pose.actual;
  msg.after_pose = grasp_pose.after;

  return msg;
}

GraspService::GraspPose GraspService::createPickingEEFPose(grasping_tp::GraspConfig grasp_msg) {
  GraspPose grasp_pose;
  tf::StampedTransform tf_base_odom;

  // TODO: try transformPose : http://docs.ros.org/indigo/api/tf/html/c++/classtf_1_1Transformer.html#a0a7b72eb4cc62194164d030afbefda9a
  tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

  tf::Vector3 tr_grasp_base(grasp_msg.position.x, grasp_msg.position.y, grasp_msg.position.z);
  tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);

  try {
      listener.waitForTransform("odom", "head_rgbd_sensor_rgb_frame", ros::Time(0), ros::Duration(3.0) );
      listener.lookupTransform("odom", "head_rgbd_sensor_rgb_frame", ros::Time(0), tf_base_odom);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
    }

    // Find grasp pose
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base;
    tf::poseTFToMsg(tf_grasp_odom, grasp_pose.actual);

    // Find pre-grasp pose
    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.1));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, grasp_pose.pre);

    grasp_pose.after = grasp_pose.actual;

    grasp_pose.after.position.z = grasp_pose.after.position.z + 0.15;
    tf::StampedTransform tf_hand_odom;
    try {
        listener.waitForTransform("odom", "hand_palm_link", ros::Time(0), ros::Duration(3.0) );
        listener.lookupTransform("odom", "hand_palm_link", ros::Time(0), tf_hand_odom);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
      }

    return grasp_pose;

} 


int main(int argc, char **argv) {
  ros::init(argc, argv, "grasping_service");
  ros::NodeHandle n;

  GraspService graspService(n);

  ros::ServiceServer service = n.advertiseService("grasping", &GraspService::grasp, &graspService);
  ROS_INFO("Waiting");
  ros::spin();

  return 0;
}