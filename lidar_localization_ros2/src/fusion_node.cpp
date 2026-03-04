#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <nav_msgs/msg/path.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>

using namespace std::chrono_literals;
using gtsam::BetweenFactor;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Symbol;

class LioNdtFusionNode : public rclcpp::Node
{
private:
  // GTSAM related
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps_;

  // State variables
  int current_index_;
  int last_lio_index_;
  Pose3 current_pose_;
  Pose3 smoothed_translation;
  rclcpp::Time last_odom_time_;

  // IMPORTANT: Store the initial pose in map frame and first LIO pose
  Pose3 initial_pose_map_;    // The robot's starting pose in map frame (e.g., 20,20,0)
  Pose3 last_lio_pose_map_;   // Last LIO pose in map frame (for relative motion)
  Pose3 first_lio_pose_odom_; // First LIO pose in odom frame (typically 0,0,0)
  Pose3 last_ndt_pose_;       // Last NDT pose in (for distance-based filtering)
  bool initial_pose_received_;
  bool first_lio_processed_;

  // Parameters
  struct Parameters
  {
    // Topics
    std::string lio_topic;
    std::string ndt_topic;
    std::string initial_pose_topic;
    std::string fused_odom_topic;
    std::string fused_pose_topic;
    std::string fused_path_topic;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    // Noise model parameters
    std::vector<double> prior_noise_sigmas;
    std::vector<double> lio_between_noise_sigmas;
    std::vector<double> ndt_prior_noise_sigmas;

    // Smoothing parameters
    double smoother_lag_duration;
    double isam2_relinearize_threshold;
    int isam2_relinearize_skip;

    // NDT filtering
    int ndt_decimation_factor;
    double ndt_mahalanobis_threshold;

    // Pose smoothing
    double pose_smoothing_alpha;

    // Logging
    bool enable_logging;
    std::string log_directory;
  } params_;

  // Timers and publishers/subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fused_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Path visualization
  nav_msgs::msg::Path path_;

  // Logging
  std::chrono::system_clock::time_point exe_start_time_;
  std::string log_file_name_;
  int ndt_counter_;

public:
  LioNdtFusionNode() : Node("lio_ndt_fusion_node"),
                       current_index_(0),
                       last_lio_index_(-1),
                       initial_pose_received_(false),
                       first_lio_processed_(false),
                       ndt_counter_(0)
  {
    // Declare and load parameters
    declareParameters();
    loadParameters();

    // Initialize smoother with parameters
    initializeSmoother();

    // Initialize logging if enabled
    if (params_.enable_logging)
    {
      log_file_name_ = generateFilename();
    }

    // Setup subscribers
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.lio_topic, 10,
        std::bind(&LioNdtFusionNode::lioCallback, this, std::placeholders::_1));

    ndt_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        params_.ndt_topic, 10,
        std::bind(&LioNdtFusionNode::ndtCallback, this, std::placeholders::_1));

    // initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     params_.initial_pose_topic, 1,
    //     std::bind(&LioNdtFusionNode::initialPoseCallback, this, std::placeholders::_1));

    // Setup publishers
    fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(params_.fused_odom_topic, 10);
    fused_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(params_.fused_pose_topic, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(params_.fused_path_topic, 10);

    // Setup TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "LIO-NDT Fusion Node initialized");
    RCLCPP_INFO(get_logger(), "Waiting for initial pose on topic: %s", params_.initial_pose_topic.c_str());
  }

private:
  void declareParameters()
  {
    // Topic parameters
    declare_parameter<std::string>("topics.lio", "/Odometry");
    declare_parameter<std::string>("topics.ndt", "/pcl_pose");
    declare_parameter<std::string>("topics.initial_pose", "/initialpose");
    declare_parameter<std::string>("topics.fused_odom", "/fused_odom");
    declare_parameter<std::string>("topics.fused_pose", "/fused_pose");
    declare_parameter<std::string>("topics.fused_path", "/fused_path");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.odom", "odom");
    declare_parameter<std::string>("frames.base", "base_link");

    // Noise model parameters
    declare_parameter<std::vector<double>>("noise.prior.sigmas",
                                           {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001});
    declare_parameter<std::vector<double>>("noise.lio_between.sigmas",
                                           {1.5, 1.5, 5.0, 0.08, 0.08, 0.08});
    declare_parameter<std::vector<double>>("noise.ndt_prior.sigmas",
                                           {1.0, 1.0, 0.8, 0.4, 0.4, 0.4});

    // Smoother parameters
    declare_parameter<double>("smoother.lag_duration", 50.0);
    declare_parameter<double>("smoother.relinearize_threshold", 0.12);
    declare_parameter<int>("smoother.relinearize_skip", 1);

    // NDT filtering
    declare_parameter<int>("ndt.decimation_factor", 5);
    declare_parameter<double>("ndt.mahalanobis_threshold", 7.0);

    // Pose smoothing
    declare_parameter<double>("pose.smoothing_alpha", 0.5);

    // Logging
    declare_parameter<bool>("logging.enable", false);
    declare_parameter<std::string>("logging.directory", "data/optimization_log/");
  }

  void loadParameters()
  {
    // Load topics
    params_.lio_topic = get_parameter("topics.lio").as_string();
    params_.ndt_topic = get_parameter("topics.ndt").as_string();
    params_.initial_pose_topic = get_parameter("topics.initial_pose").as_string();
    params_.fused_odom_topic = get_parameter("topics.fused_odom").as_string();
    params_.fused_pose_topic = get_parameter("topics.fused_pose").as_string();
    params_.fused_path_topic = get_parameter("topics.fused_path").as_string();
    params_.map_frame = get_parameter("frames.map").as_string();
    params_.odom_frame = get_parameter("frames.odom").as_string();
    params_.base_frame = get_parameter("frames.base").as_string();

    // Load noise parameters
    params_.prior_noise_sigmas = get_parameter("noise.prior.sigmas").as_double_array();
    params_.lio_between_noise_sigmas = get_parameter("noise.lio_between.sigmas").as_double_array();
    params_.ndt_prior_noise_sigmas = get_parameter("noise.ndt_prior.sigmas").as_double_array();

    // Validate vector sizes
    if (params_.prior_noise_sigmas.size() != 6)
    {
      RCLCPP_WARN(get_logger(), "prior_noise_sigmas size != 6, using defaults");
      params_.prior_noise_sigmas = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};
    }

    if (params_.lio_between_noise_sigmas.size() != 6)
    {
      RCLCPP_WARN(get_logger(), "lio_between_noise_sigmas size != 6, using defaults");
      params_.lio_between_noise_sigmas = {1.5, 1.5, 5.0, 0.08, 0.08, 0.08};
    }

    if (params_.ndt_prior_noise_sigmas.size() != 6)
    {
      RCLCPP_WARN(get_logger(), "ndt_prior_noise_sigmas size != 6, using defaults");
      params_.ndt_prior_noise_sigmas = {1.0, 1.0, 0.8, 0.4, 0.4, 0.4};
    }

    // Load smoother parameters
    params_.smoother_lag_duration = get_parameter("smoother.lag_duration").as_double();
    params_.isam2_relinearize_threshold = get_parameter("smoother.relinearize_threshold").as_double();
    params_.isam2_relinearize_skip = get_parameter("smoother.relinearize_skip").as_int();

    // Load NDT filtering
    params_.ndt_decimation_factor = get_parameter("ndt.decimation_factor").as_int();
    params_.ndt_mahalanobis_threshold = get_parameter("ndt.mahalanobis_threshold").as_double();

    // Load pose smoothing
    params_.pose_smoothing_alpha = get_parameter("pose.smoothing_alpha").as_double();

    // Load logging
    params_.enable_logging = get_parameter("logging.enable").as_bool();
    params_.log_directory = get_parameter("logging.directory").as_string();
  }

  void initializeSmoother()
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = params_.isam2_relinearize_threshold;
    parameters.relinearizeSkip = params_.isam2_relinearize_skip;
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(params_.smoother_lag_duration, parameters);
  }

  std::string generateFilename()
  {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << params_.log_directory;
    ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    return ss.str();
  }

  double mahalanobisDistance6D(
      const gtsam::Pose3 &pose1,
      const gtsam::Pose3 &pose2,
      const gtsam::SharedNoiseModel &noise)
  {
    auto diag = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(noise);
    if (!diag)
      return std::numeric_limits<double>::max();

    gtsam::Vector6 delta = pose1.localCoordinates(pose2);
    gtsam::Vector6 sigmas = diag->sigmas();
    gtsam::Vector6 norm = delta.cwiseQuotient(sigmas);
    return std::sqrt(norm.dot(norm));
  }

  Pose3 poseMsgToGtsam(const geometry_msgs::msg::Pose &pose_msg)
  {
    return Pose3(
        gtsam::Rot3::Quaternion(pose_msg.orientation.w, pose_msg.orientation.x,
                                pose_msg.orientation.y, pose_msg.orientation.z),
        gtsam::Point3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
  }

  geometry_msgs::msg::Pose gtsamToPoseMsg(const Pose3 &pose)
  {
    geometry_msgs::msg::Pose msg;
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    msg.position.x = pose.x();
    msg.position.y = pose.y();
    msg.position.z = pose.z();
    return msg;
  }

  gtsam::SharedNoiseModel createNoiseModelFromVector(const std::vector<double> &sigmas)
  {
    gtsam::Vector6 noise_vec = gtsam::Vector6::Zero();
    for (int i = 0; i < 6; ++i)
      noise_vec(i) = sigmas[i];

    return gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
  }

  /**
   * @brief Converts a pose from odometry frame to map frame using the initial pose relationship
   *
   * Since the first LIO pose is at (0,0,0) in odom frame and the robot's initial pose
   * in map frame is known, we can compute any LIO pose in map frame as:
   *
   * pose_map = initial_pose_map * (first_lio_pose_odom.inverse() * current_lio_pose_odom)
   *
   * But since first_lio_pose_odom is usually identity (0,0,0), this simplifies to:
   * pose_map = initial_pose_map * current_lio_pose_odom
   */
  Pose3 odomPoseToMapPose(const Pose3 &lio_pose_odom)
  {
    // The relative motion from first LIO pose to current LIO pose
    // Since first LIO pose is typically identity, this is just lio_pose_odom
    Pose3 relative_motion = first_lio_pose_odom_.between(lio_pose_odom);
    // std::cout << "Relative motion from first LIO pose to current LIO pose: ["
    //           << relative_motion.x() << ", " << relative_motion.y() << ", " << relative_motion.z() << "]" << std::endl;
    // RCLCPP_DEBUG(get_logger(), "Relative motion from first LIO pose to current LIO pose: ["
    //                                << relative_motion.x() << ", " << relative_motion.y() << ", " << relative_motion.z() << "]");
    // // Apply this relative motion to the initial map pose
    return initial_pose_map_.compose(relative_motion);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (initial_pose_received_)
    {
      RCLCPP_WARN(get_logger(), "Initial pose already received, ignoring new one");
      return;
    }

    RCLCPP_INFO(get_logger(), "Received initial pose in map frame");

    // Store the initial pose in map frame
    initial_pose_map_ = poseMsgToGtsam(msg->pose.pose);

    // Add prior factor for the initial pose (this will be key 0)
    auto prior_noise = createNoiseModelFromVector(params_.prior_noise_sigmas);

    const gtsam::Symbol current_key('x', current_index_);
    new_factors_.addPrior(current_key, initial_pose_map_, prior_noise);
    new_values_.insert(current_key, initial_pose_map_);
    new_timestamps_[current_key] = rclcpp::Time(msg->header.stamp).seconds();

    // Update smoother
    smoother_->update(new_factors_, new_values_, new_timestamps_);

    // Clear for next iteration
    new_factors_.resize(0);
    new_values_.clear();
    new_timestamps_.clear();

    last_lio_index_ = current_index_;
    current_index_++;
    last_odom_time_ = msg->header.stamp;
    initial_pose_received_ = true;
    // check time of initial pose reception

    RCLCPP_INFO(get_logger(), "Initial pose received at time: %.3f seconds", rclcpp::Time(msg->header.stamp).seconds());
    RCLCPP_INFO(get_logger(), "Initial pose set in map frame at: [%.3f, %.3f, %.3f]",
                initial_pose_map_.x(), initial_pose_map_.y(), initial_pose_map_.z());

    publishEstimate();
  }

  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!initial_pose_received_)
    {
      RCLCPP_DEBUG(get_logger(), "Waiting for initial pose before processing LIO");
      return;
    }

    exe_start_time_ = std::chrono::system_clock::now();
    const double current_time = rclcpp::Time(msg->header.stamp).seconds();

    // Get LIO pose in odometry frame (LIO's own frame, starting at origin)
    Pose3 lio_pose_odom = poseMsgToGtsam(msg->pose.pose);

    if (!first_lio_processed_)
    {
      // This is the first LIO message
      first_lio_pose_odom_ = lio_pose_odom;

      // RCLCPP_INFO(get_logger(), "First LIO pose in odom frame: [%.3f, %.3f, %.3f]",
      //             lio_pose_odom.x(), lio_pose_odom.y(), lio_pose_odom.z());

      // The first LIO pose in map frame should equal the initial pose
      // Since first_lio_pose_odom is typically identity, this works automatically
      Pose3 lio_pose_map = odomPoseToMapPose(lio_pose_odom);
      last_lio_pose_map_ = lio_pose_map;

      // // Add as a prior (should match the initial pose closely)
      // auto prior_noise = createNoiseModelFromVector(params_.prior_noise_sigmas);
      // const gtsam::Symbol current_key('x', current_index_);
      // new_factors_.addPrior(current_key, lio_pose_map, prior_noise);
      // new_values_.insert(current_key, lio_pose_map);
      // new_timestamps_[current_key] = current_time;

      first_lio_processed_ = true;

      RCLCPP_INFO(get_logger(), "First LIO pose in map frame: [%.3f, %.3f, %.3f]",
                  lio_pose_map.x(), lio_pose_map.y(), lio_pose_map.z());
    }
    else
    {
      // For subsequent LIO messages, create between factor
      const gtsam::Symbol prev_key('x', last_lio_index_);
      const gtsam::Symbol current_key('x', current_index_);
      // current key and last key log

      RCLCPP_INFO(get_logger(), "Processing LIO pose for key %d at time %.3f", current_index_, current_time);
      // Get previous pose in map frame from smoother
      // gtsam::Values estimates = smoother_->calculateEstimate();
      // Pose3 prev_pose_map = estimates.at<Pose3>(prev_key);

      // Convert current LIO pose to map frame
      Pose3 current_pose_map = odomPoseToMapPose(lio_pose_odom);

      // Calculate relative motion between consecutive poses in map frame
      // This should be the same as the relative motion in odom frame
      Pose3 relative_motion = last_lio_pose_map_.between(current_pose_map);
      last_lio_pose_map_ = current_pose_map;
      auto between_noise = createNoiseModelFromVector(params_.lio_between_noise_sigmas);
      new_factors_.add(BetweenFactor<Pose3>(prev_key, current_key, relative_motion, between_noise));
      new_values_.insert(current_key, current_pose_map);
      new_timestamps_[current_key] = current_time;
      last_lio_index_ = current_index_;
      current_index_++;
    }
    RCLCPP_INFO(get_logger(), "Added LIO factor between ");

    last_odom_time_ = msg->header.stamp;
    RCLCPP_INFO(get_logger(), "Added LIO factor for key %d at time %.3f", last_lio_index_, current_time);
    // Update smoother
    smoother_->update(new_factors_, new_values_, new_timestamps_);

    // Clear for next iteration
    new_factors_.resize(0);
    new_values_.clear();
    new_timestamps_.clear();
    RCLCPP_INFO(get_logger(), "Smoother updated with LIO factor for key %d", last_lio_index_);
    publishEstimate();
    logExecutionTime();
  }

  void ndtCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {

    // NDT pose is already in map frame
    Pose3 ndt_pose_map = poseMsgToGtsam(msg->pose.pose);

    if (!initial_pose_received_ || last_lio_index_ < 0 || !first_lio_processed_)
    {
      RCLCPP_DEBUG(get_logger(), "Waiting for initial pose and first LIO pose before processing NDT");
      initialPoseCallback(msg);
      last_ndt_pose_ = ndt_pose_map; // Store the first NDT pose for distance calculations
      return;
    }
    //  calaculate the pose distance between previous pose and current ndt pose
    double distance = (ndt_pose_map.translation() - last_ndt_pose_.translation()).norm();
    RCLCPP_INFO(get_logger(), "NDT pose distance from previous LIO pose: %.3f", distance);

    // Decimate NDT messages
    if (++ndt_counter_ % params_.ndt_decimation_factor != 0 || distance < 0.5) // Also check if NDT pose is too close to last one
    {
      return;
    }

    exe_start_time_ = std::chrono::system_clock::now();

    // Check Mahalanobis distance
    auto ndt_noise = createNoiseModelFromVector(params_.ndt_prior_noise_sigmas);
    // double mah_distance = mahalanobisDistance6D(current_pose_, ndt_pose_map, ndt_noise);

    // if (mah_distance > params_.ndt_mahalanobis_threshold)
    // {
    //     RCLCPP_DEBUG(get_logger(), "Rejecting NDT pose with Mahalanobis distance: %.2f", mah_distance);
    //     return;
    // }

    // Add NDT as prior factor on the current state
    const gtsam::Symbol current_key('x', last_lio_index_);
    new_factors_.addPrior(current_key, ndt_pose_map, ndt_noise);

    // Update smoother
    smoother_->update(new_factors_, gtsam::Values(), gtsam::FixedLagSmoother::KeyTimestampMap());

    // Clear factors
    new_factors_.resize(0);

    RCLCPP_INFO(get_logger(), "Added NDT prior factor for key %d", last_lio_index_);
    publishEstimate();
    logExecutionTime();
  }

  void publishEstimate()
  {
    if (last_lio_index_ < 0)
      return;

    try
    {
      // Get latest estimate from smoother
      gtsam::Values estimates = smoother_->calculateEstimate();
      RCLCPP_DEBUG(get_logger(), "Calculated smoother estimate for key %d", last_lio_index_);
      const gtsam::Symbol current_key('x', last_lio_index_);

      if (estimates.exists(current_key))
      {
        current_pose_ = estimates.at<Pose3>(current_key);

        // Create and publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_odom_time_;
        odom_msg.header.frame_id = params_.map_frame;
        odom_msg.child_frame_id = params_.base_frame;

        odom_msg.pose.pose = gtsamToPoseMsg(current_pose_);

        // Set covariance (simplified)
        for (int i = 0; i < 36; i++)
          odom_msg.pose.covariance[i] = 0.01;

        fused_odom_pub_->publish(odom_msg);

        // Publish pose with covariance
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
        pose_cov.header = odom_msg.header;
        pose_cov.pose = odom_msg.pose;
        fused_pose_pub_->publish(pose_cov);

        // Publish TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = odom_msg.header;
        tf_msg.child_frame_id = params_.base_frame;
        tf_msg.transform.translation.x = current_pose_.x();
        tf_msg.transform.translation.y = current_pose_.y();
        tf_msg.transform.translation.z = current_pose_.z();

        gtsam::Quaternion q = current_pose_.rotation().toQuaternion();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);

        // Update and publish path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        path_.poses.push_back(pose_stamped);

        nav_msgs::msg::Path path;
        path.header = odom_msg.header;
        path.poses = path_.poses;
        path_pub_->publish(path);
      }
    }
    catch (const gtsam::ValuesKeyDoesNotExist &e)
    {
      RCLCPP_ERROR(get_logger(), "Error in publishEstimate: %s", e.what());
    }
  }

  void logExecutionTime()
  {
    if (!params_.enable_logging)
      return;

    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
        std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time_).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;

    std::ofstream optimization_log;
    optimization_log << std::fixed << std::setprecision(9);
    optimization_log.open(log_file_name_, std::ios::app);

    optimization_log << rclcpp::Time(last_odom_time_).seconds() << " "
                     << exe_time << " "
                     << params_.smoother_lag_duration << " "
                     << last_lio_index_ << std::endl;

    optimization_log.close();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LioNdtFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}