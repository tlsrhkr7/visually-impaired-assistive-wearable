#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rmw/qos_profiles.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav2_map_server/map_io.hpp"

#include <chrono>
#include <filesystem>
#include <sstream>

#include <cv_bridge/cv_bridge.h>

// this is orb_slam3
#include "System.h"

// Override Sophus abort() with a catchable exception.
// Activated by SOPHUS_ENABLE_ENSURE_HANDLER in CMakeLists.
namespace Sophus {
void ensureFailed(char const* function, char const* /*file*/, int /*line*/,
                  char const* description) {
  throw std::runtime_error(std::string("Sophus ensure failed in ") +
                           function + ": " + description);
}
}  // namespace Sophus

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class ImuMonoRealSense : public rclcpp::Node {
public:
  ImuMonoRealSense()
    : Node("imu_mono_realsense"),
      vocabulary_file_path(std::string(PROJECT_PATH) +
                           "/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
      inertial_ba1_(false), inertial_ba2_(false)
  {

    // declare parameters
    declare_parameter("sensor_type", "imu-monocular");
    declare_parameter("use_pangolin", true);
    declare_parameter("localization_mode", false);

    // get parameters
    sensor_type_param = get_parameter("sensor_type").as_string();
    use_pangolin = get_parameter("use_pangolin").as_bool();
    bool localization_mode = get_parameter("localization_mode").as_bool();

    // define callback groups
    image_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slam_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;

    // set the sensor type based on parameter
    ORB_SLAM3::System::eSensor sensor_type;
    if (sensor_type_param == "monocular") {
      sensor_type = ORB_SLAM3::System::MONOCULAR;
      settings_file_path =
        std::string(PROJECT_PATH) + "/config/Monocular/RealSense_D435i.yaml";
    } else if (sensor_type_param == "imu-monocular") {
      sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
      settings_file_path = std::string(PROJECT_PATH) +
                           "/config/Monocular-Inertial/RealSense_D435i.yaml";
    } else if (sensor_type_param == "rgbd-inertial") {
      sensor_type = ORB_SLAM3::System::IMU_RGBD;
      if (localization_mode) {
        settings_file_path = std::string(PROJECT_PATH) +
                             "/config/RGBD-Inertial/RealSense_D435i_localization.yaml";
      } else {
        settings_file_path = std::string(PROJECT_PATH) +
                             "/config/RGBD-Inertial/RealSense_D435i.yaml";
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type not recognized");
      rclcpp::shutdown();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "vocabulary_file_path: " << vocabulary_file_path);

    // setup orb slam object
    orb_slam3_system_ = std::make_shared<ORB_SLAM3::System>(
      vocabulary_file_path, settings_file_path, sensor_type, use_pangolin, 0);

    if (localization_mode) {
      orb_slam3_system_->ActivateLocalizationMode();
      RCLCPP_INFO(get_logger(), "Localization mode activated - map loaded, not adding new keyframes");
    }

    // create publishers
    live_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("live_point_cloud", 10);
    pose_array_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
    live_occupancy_grid_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("live_occupancy_grid", 10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("orb_odom", 10);
    orb_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("/orb_camera/image", 10);
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>("/orb_camera/imu", 10);

    // create subscriptions
    rclcpp::QoS image_qos(rclcpp::KeepLast(10));
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(10));
    imu_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    imu_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    if (sensor_type_param == "rgbd-inertial") {
      rgb_filter_sub_.subscribe(this, "camera/camera/color/image_raw",
                                image_qos.get_rmw_qos_profile());
      depth_filter_sub_.subscribe(this, "camera/camera/depth/image_rect_raw",
                                  image_qos.get_rmw_qos_profile());
      rgbd_sync_ = std::make_shared<RGBDSync>(
        RGBDPolicy(10), rgb_filter_sub_, depth_filter_sub_);
      rgbd_sync_->registerCallback(
        std::bind(&ImuMonoRealSense::rgbd_callback, this, _1, _2));
    } else {
      image_sub = create_subscription<sensor_msgs::msg::Image>(
        "camera/camera/color/image_raw", image_qos,
        std::bind(&ImuMonoRealSense::image_callback, this, _1), image_options);
    }

    imu_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/imu", imu_qos,
      std::bind(&ImuMonoRealSense::imu_callback, this, _1), imu_options);

    // tf broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create timer
    timer = create_wall_timer(
      100ms, std::bind(&ImuMonoRealSense::timer_callback, this),
      timer_callback_group_);

    timestamp_ = generate_timestamp_string();

    std::string path = std::string(PROJECT_PATH) + "/output/" + timestamp_;
    if (!std::filesystem::create_directory(path)) {
      std::cout << "Failed to create output directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/cloud")) {
      std::cout << "Failed to create cloud directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/grid")) {
      std::cout << "Failed to create grid directory" << std::endl;
      return;
    }
    if (!std::filesystem::create_directory(path + "/video")) {
      std::cout << "Failed to create images directory" << std::endl;
      return;
    }

    rclcpp::on_shutdown([this]() {
      video_writer_.release();
      pcl::io::savePCDFileBinary(std::string(PROJECT_PATH) + "/output/" +
                                   timestamp_ + "/cloud/" + timestamp_ + ".pcd",
                                 live_pcl_cloud_);
      nav2_map_server::SaveParameters save_params;
      save_params.map_file_name = std::string(PROJECT_PATH) + "/output/" +
                                  timestamp_ + "/grid/" + timestamp_;
      save_params.image_format = "pgm";
      save_params.free_thresh = 0.196;
      save_params.occupied_thresh = 0.65;
      nav2_map_server::saveMapToFile(*live_occupancy_grid_, save_params);
    });

    initialize_variables();

    std::string orb_slam_video_path = std::string(PROJECT_PATH) + "/output/" +
                                      timestamp_ + "/video/" + timestamp_ +
                                      ".mp4";
    RCLCPP_INFO_STREAM(get_logger(), "Video path: " << orb_slam_video_path);
    video_writer_.open(orb_slam_video_path,
                       cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30,
                       cv::Size(640, 500));

    if (!video_writer_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Error opening video writer");
      rclcpp::shutdown();
    }
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  filter_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*sor_cloud);

    // radius outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
    radius_outlier.setInputCloud(sor_cloud);
    radius_outlier.setRadiusSearch(0.3);
    radius_outlier.setMinNeighborsInRadius(2);
    radius_outlier.filter(*radius_cloud);

    return radius_cloud;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr
  point_cloud_to_occupancy_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    // calculate the centroid
    Eigen::Matrix<float, 4, 1> centroid;
    pcl::ConstCloudIterator<pcl::PointXYZ> cloud_iterator(*cloud);
    pcl::compute3DCentroid(cloud_iterator, centroid);

    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();

    for (const auto &point : cloud->points) {
      if (point.x > max_x) {
        max_x = point.x;
      }
      if (point.y > max_y) {
        max_y = point.y;
      }
      if (point.x < min_x) {
        min_x = point.x;
      }
      if (point.y < min_y) {
        min_y = point.y;
      }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid =
      std::make_shared<nav_msgs::msg::OccupancyGrid>();
    cloud->width = cloud->points.size();
    occupancy_grid->header.frame_id = "live_map";
    occupancy_grid->header.stamp = get_clock()->now();
    occupancy_grid->info.resolution = 0.05;
    occupancy_grid->info.width =
      std::abs(max_x - min_x) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.height =
      std::abs(max_y - min_y) / occupancy_grid->info.resolution + 1;
    occupancy_grid->info.origin.position.x = min_x;
    occupancy_grid->info.origin.position.y = min_y;
    occupancy_grid->info.origin.position.z = 0;
    occupancy_grid->info.origin.orientation.x = 0;
    occupancy_grid->info.origin.orientation.y = 0;
    occupancy_grid->info.origin.orientation.z = 0;
    occupancy_grid->info.origin.orientation.w = 1;
    occupancy_grid->data.resize(
      occupancy_grid->info.width * occupancy_grid->info.height, 0);
    for (const auto &point : cloud->points) {
      int x = (point.x - min_x) / occupancy_grid->info.resolution;
      int y = (point.y - min_y) / occupancy_grid->info.resolution;
      int index = y * occupancy_grid->info.width + x;
      occupancy_grid->data.at(index) = 100;
    }
    return occupancy_grid;
  }

  void initialize_variables()
  {
    pose_array_ = geometry_msgs::msg::PoseArray();
    pose_array_.header.frame_id = "live_map";

    live_pcl_cloud_msg_ = sensor_msgs::msg::PointCloud2();
    live_pcl_cloud_msg_.header.frame_id = "live_map";

    live_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  }

  std::string generate_timestamp_string()
  {
    std::time_t now = std::time(nullptr);
    std::tm *ptm = std::localtime(&now);

    std::ostringstream oss;

    oss << std::put_time(ptm, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    } else {
      std::cerr << "Error image type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    rclcpp::Time time_now = get_clock()->now();

    sensor_msgs::msg::Image::SharedPtr msg_out =
      std::make_shared<sensor_msgs::msg::Image>(*msg);
    msg_out->header.stamp = time_now;
    msg_out->header.frame_id = "base_link";
    orb_image_publisher_->publish(*msg_out);

    img_buf_.push(msg);

    // begin to empty the img_buf_ queue, which is full of other queues
    while (!img_buf_.empty()) {
      // grab the oldest image
      auto imgPtr = img_buf_.front();
      img_buf_.pop();

      cv::Mat imageFrame = get_image(imgPtr);
      double tImage =
        imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;

      // package all the imu data for this image for orbslam3 to process
      buf_mutex_imu_.lock();
      while (!imu_buf_.empty()) {
        auto imuPtr = imu_buf_.front();
        imu_buf_.pop();
        double tIMU =
          imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;

        cv::Point3f acc(imuPtr->linear_acceleration.x,
                        imuPtr->linear_acceleration.y,
                        imuPtr->linear_acceleration.z);
        cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                        imuPtr->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
      }

      buf_mutex_imu_.unlock();

      if (vImuMeas.empty() && sensor_type_param == "imu-monocular") {
        // RCLCPP_WARN(get_logger(),
        //             "No valid IMU data available for the current frame "
        //             "at time %.6f.",
        //             tImage);
        return;
      }

      try {
        if (sensor_type_param == "monocular") {
          auto Tcw = orb_slam3_system_->TrackMonocular(imageFrame, tImage);
          Tcw_.translation() = Tcw.translation();
          Tcw_.setQuaternion(Tcw.unit_quaternion());
        } else {
          if (vImuMeas.size() > 1) {
            auto Tcw =
              orb_slam3_system_->TrackMonocular(imageFrame, tImage, vImuMeas);
            Tcw_.translation() = Tcw.translation();
            Tcw_.setQuaternion(Tcw.unit_quaternion());
          }
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
      }

      cv::Mat pretty_frame = orb_slam3_system_->getPrettyFrame();
      video_writer_.write(pretty_frame);
    }
  }

  void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
  {
    cv_bridge::CvImageConstPtr rgb_ptr, depth_ptr;
    try {
      rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::MONO8);
      depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    double tImage = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    buf_mutex_imu_.lock();
    while (!imu_buf_.empty()) {
      auto imuPtr = imu_buf_.front();
      imu_buf_.pop();
      double tIMU = imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9;
      cv::Point3f acc(imuPtr->linear_acceleration.x,
                      imuPtr->linear_acceleration.y,
                      imuPtr->linear_acceleration.z);
      cv::Point3f gyr(imuPtr->angular_velocity.x, imuPtr->angular_velocity.y,
                      imuPtr->angular_velocity.z);
      vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tIMU));
    }
    buf_mutex_imu_.unlock();

    if (vImuMeas.empty()) {
      return;
    }

    try {
      if (vImuMeas.size() > 1) {
        auto Tcw = orb_slam3_system_->TrackRGBD(
          rgb_ptr->image, depth_ptr->image, tImage, vImuMeas);

        int state = orb_slam3_system_->GetTrackingState();
        if (state == 2) {  // OK
          tracking_lost_count_ = 0;
          if (!std::isnan(Tcw.translation().x()) &&
              !std::isnan(Tcw.unit_quaternion().coeffs().sum())) {
            Tcw_.translation() = Tcw.translation();
            Tcw_.setQuaternion(Tcw.unit_quaternion());
          }
        } else {
          tracking_lost_count_++;
          if (tracking_lost_count_ > 60) {
            RCLCPP_WARN(get_logger(),
                        "Tracking lost for %d frames, resetting system",
                        tracking_lost_count_);
            orb_slam3_system_->Reset();
            tracking_lost_count_ = 0;
          }
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "SLAM processing exception: %s", e.what());
    }

    cv::Mat pretty_frame = orb_slam3_system_->getPrettyFrame();
    video_writer_.write(pretty_frame);
  }

  void imu_callback(const sensor_msgs::msg::Imu &msg)
  {
    buf_mutex_imu_.lock();
    sensor_msgs::msg::Imu msg_out = msg;
    msg_out.header.stamp = get_clock()->now();
    msg_out.header.frame_id = "base_link";
    // imu_publisher_->publish(msg_out);
    if (!std::isnan(msg.linear_acceleration.x) &&
        !std::isnan(msg.linear_acceleration.y) &&
        !std::isnan(msg.linear_acceleration.z) &&
        !std::isnan(msg.angular_velocity.x) &&
        !std::isnan(msg.angular_velocity.y) &&
        !std::isnan(msg.angular_velocity.z)) {
      const sensor_msgs::msg::Imu::SharedPtr msg_ptr =
        std::make_shared<sensor_msgs::msg::Imu>(msg);
      imu_buf_.push(msg_ptr);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid IMU data - nan");
    }
    buf_mutex_imu_.unlock();
  }

  void timer_callback()
  {
    unique_lock<mutex> lock(orbslam3_mutex_);

    geometry_msgs::msg::Pose pose;
    if (orb_slam3_system_->isImuInitialized()) {
      rclcpp::Time time_now = get_clock()->now();
      auto Twc = Tcw_.inverse();

      tf2::Quaternion q_orig(
        Twc.unit_quaternion().x(), Twc.unit_quaternion().y(),
        Twc.unit_quaternion().z(), Twc.unit_quaternion().w());

      tf2::Matrix3x3 m(q_orig);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0, 0, yaw);

      tf2::Quaternion q_rot_x, q_rot_z;
      // q_rot_x.setRPY(M_PI / 2.0, 0, 0);
      q_rot_z.setRPY(0, 0, M_PI / 2.0);

      // change camera coordinate to map coordinates
      tf2::Quaternion q_combined = q_rot_z * q_yaw;
      q_combined.normalize();

      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = time_now;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = Twc.translation().x();
      odom_tf.transform.translation.y = Twc.translation().y();
      odom_tf.transform.translation.z = Twc.translation().z();
      odom_tf.transform.rotation.x = Twc.unit_quaternion().x();
      odom_tf.transform.rotation.y = Twc.unit_quaternion().y();
      odom_tf.transform.rotation.z = Twc.unit_quaternion().z();
      odom_tf.transform.rotation.w = Twc.unit_quaternion().w();
      tf_broadcaster->sendTransform(odom_tf);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time_now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = Twc.translation().x();
      odom.pose.pose.position.y = Twc.translation().y();
      odom.pose.pose.position.z = Twc.translation().z();
      odom.pose.pose.orientation.x = Twc.unit_quaternion().x();
      odom.pose.pose.orientation.y = Twc.unit_quaternion().y();
      odom.pose.pose.orientation.z = Twc.unit_quaternion().z();
      odom.pose.pose.orientation.w = Twc.unit_quaternion().w();
      odom_publisher_->publish(odom);

      geometry_msgs::msg::Pose pose;
      pose.position.x = Twc.translation().x();
      pose.position.y = Twc.translation().y();
      // pose.position.z = Twc.translation().z();
      pose.orientation.x = q_combined.x();
      pose.orientation.y = q_combined.y();
      pose.orientation.z = q_combined.z();
      pose.orientation.w = q_combined.w();
      pose_array_.header.stamp = time_now;
      pose_array_.poses.push_back(pose);
      pose_array_publisher_->publish(pose_array_);

      // geometry_msgs::msg::TransformStamped base_link_tf;
      // base_link_tf.header.stamp = time_now;
      // base_link_tf.header.frame_id = "live_map";
      // base_link_tf.child_frame_id = "base_link";
      // base_link_tf.transform.translation.x = Twc.translation().x();
      // base_link_tf.transform.translation.y = Twc.translation().y();
      // base_link_tf.transform.rotation.x = q_combined.x();
      // base_link_tf.transform.rotation.y = q_combined.y();
      // base_link_tf.transform.rotation.z = q_combined.z();
      // base_link_tf.transform.rotation.w = q_combined.w();
      // tf_broadcaster->sendTransform(base_link_tf);

      geometry_msgs::msg::TransformStamped point_cloud_tf;
      point_cloud_tf.header.stamp = time_now;
      point_cloud_tf.header.frame_id = "map";
      point_cloud_tf.child_frame_id = "point_cloud";
      tf_broadcaster->sendTransform(point_cloud_tf);

      geometry_msgs::msg::TransformStamped live_map_tf;
      live_map_tf.header.stamp = time_now;
      live_map_tf.header.frame_id = "map";
      live_map_tf.child_frame_id = "live_map";
      tf_broadcaster->sendTransform(live_map_tf);

      live_pcl_cloud_ = orb_slam3_system_->GetMapPCL();

      pcl::PointCloud<pcl::PointXYZ>::Ptr live_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(live_pcl_cloud_);

      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      filtered_cloud_ptr = filter_point_cloud(live_ptr);

      filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();
      pcl::toROSMsg(*filtered_cloud_ptr, live_pcl_cloud_msg_);

      // this can go
      // live_occupancy_grid_ = point_cloud_to_occupancy_grid(filtered_cloud_ptr);
      // live_occupancy_grid_->header.stamp = time_now;
      // live_occupancy_grid_->header.frame_id = "live_map";
      // live_occupancy_grid_publisher_->publish(*live_occupancy_grid_);

      live_pcl_cloud_msg_.header.stamp = time_now;
      live_pcl_cloud_msg_.header.frame_id = "live_map";
      live_point_cloud_publisher_->publish(live_pcl_cloud_msg_);
    } else {
      // RCLCPP_INFO_STREAM(get_logger(), "IMU not initialized");
      initialize_variables();
    }
    if (!inertial_ba1_ && orb_slam3_system_->GetInertialBA1()) {
      inertial_ba1_ = true;
      initialize_variables();
      RCLCPP_INFO(get_logger(), "Inertial BA1 complete");
    }

    if (!inertial_ba2_ && orb_slam3_system_->GetInertialBA2()) {
      inertial_ba2_ = true;
      initialize_variables();
      RCLCPP_INFO(get_logger(), "Inertial BA2 complete");
    }
  }

  using RGBDPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using RGBDSync = message_filters::Synchronizer<RGBDPolicy>;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_filter_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_filter_sub_;
  std::shared_ptr<RGBDSync> rgbd_sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    live_point_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
    pose_array_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    live_occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr orb_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr slam_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  sensor_msgs::msg::Imu imu_msg;
  geometry_msgs::msg::PoseArray pose_array_;

  std::string sensor_type_param;
  bool use_pangolin;

  std::vector<geometry_msgs::msg::Vector3> vGyro;
  std::vector<double> vGyro_times;
  std::vector<geometry_msgs::msg::Vector3> vAccel;
  std::vector<double> vAccel_times;

  queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf_;
  queue<sensor_msgs::msg::Image::SharedPtr> img_buf_;
  std::mutex buf_mutex_imu_, buf_mutex_img_, orbslam3_mutex_,
    live_pcl_cloud_mutex_;

  std::shared_ptr<ORB_SLAM3::System> orb_slam3_system_;
  std::string vocabulary_file_path;
  std::string settings_file_path;

  sensor_msgs::msg::PointCloud2 live_pcl_cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ> live_pcl_cloud_;
  nav_msgs::msg::OccupancyGrid::SharedPtr live_occupancy_grid_;

  bool inertial_ba1_;
  bool inertial_ba2_;
  int tracking_lost_count_ = 0;
  Sophus::SE3f Tcw_;

  cv::VideoWriter video_writer_;
  std::string timestamp_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuMonoRealSense>());
  rclcpp::shutdown();
  return 0;
}
