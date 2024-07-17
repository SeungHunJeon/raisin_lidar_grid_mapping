// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by jsh on 24. 5. 2.
//

#ifndef RAISIN_PLUGIN_LIDAR_GRID_MAPPING_HPP
#define RAISIN_PLUGIN_LIDAR_GRID_MAPPING_HPP

#include "raisin_plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "grid_mapping/grid_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "omp.h"
#include "tbb/tbb.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

namespace raisin
{
namespace plugin
{

struct Pose
{
  double timeStamp;
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
};

class PoseBuffer
{
public:
  PoseBuffer(size_t maxSize = 100)
  : maxSize_(maxSize) {}

  // Move constructor
  PoseBuffer(PoseBuffer && other) noexcept
  : maxSize_(other.maxSize_),
    buffer(std::move(other.buffer)) {}

  // Move assignment operator
  PoseBuffer & operator=(PoseBuffer && other) noexcept
  {
    if (this != &other) {
      std::lock_guard<std::mutex> guard(mtx_);
      maxSize_ = other.maxSize_;
      buffer = std::move(other.buffer);
    }
    return *this;
  }

  void addPose(double timeStamp, Eigen::Vector3d pos, Eigen::Matrix3d ori)
  {

    std::lock_guard<std::mutex> guard(mtx_);
    if (buffer.size() >= maxSize_) {
      buffer.pop_front();        // Remove the oldest pose if buffer is full
    }
    buffer.push_back({timeStamp, pos, ori});
  }

  Pose getClosestPose(double timeStamp)
  {
    Pose closestPose;
    double minDiff = std::numeric_limits<double>::max();

    std::lock_guard<std::mutex> guard(mtx_);
    for (auto rit = buffer.rbegin(); rit != buffer.rend(); ++rit) {
      double diff = std::abs(rit->timeStamp - timeStamp);
      if (diff < minDiff) {
        minDiff = diff;
        closestPose = *rit;
      } else {
        break;
      }
    }
    return closestPose;
  }

private:
  size_t maxSize_;
  std::deque<Pose> buffer;
  std::mutex mtx_;
};
typedef struct {
  float x;            /**< X axis, Unit:m */
  float y;            /**< Y axis, Unit:m */
  float z;            /**< Z axis, Unit:m */
  float reflectivity; /**< Reflectivity   */
  uint8_t tag;        /**< Livox point tag   */
  uint8_t line;       /**< Laser line id     */
  double timestamp;   /**< Timestamp of point*/
} LivoxPointXyzrtlt;

// Hash function for Eigen::Vector3i
struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i& v) const {
    return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
  }

  bool equal(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
    return a == b;
  }

  static size_t hash(const Eigen::Vector3i& v) {
    return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
  }
};

class LidarGridMapping : public Plugin, public rclcpp::Node
{
public:
  LidarGridMapping(
    raisim::World & world, raisim::RaisimServer & server,
    raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);

  ~LidarGridMapping();

  bool advance() final;

  bool init() final;

  void mapReset();

  void initMap();

  void initSensor();

  void mapUpdate(const std::string & linkName);

  void lidarMapUpdate(const std::string & lidarName);

  inline void rayCasting(const Eigen::Vector3d & start, const Eigen::Vector3d & end);

  void convertToHeightVec(const std::string & layerName = "mean"); // for grid map to raisim height map

  inline bool downsamplingSelection(
    const int & camWidth, const int & rayIdx, const int & iterIdx,
    const int & kernelSize);

  void isNearRobot(
    std::vector<raisim::Vec<3>> & pointCloud, std::vector<std::string> legsToCheck,
    double timeStamp, std::vector<bool> & result);
  std::map<std::string, std::vector<bool>> isNearRobot_;

  std::vector<raisim::Vec<3>> voxelGridFilter(const std::vector<raisim::Vec<3>>& pointCloud, double voxelSize);

private:
  void readOctree(const std::string & lidarName);

  void convertToOctree(const std::string & lidarName);

  std::mutex mtx_;
  parameter::ParameterContainer & param_;
  grid_map::GridMap localMap_;
  raisim::HeightMap * raisimHeightMap_;
  std::vector<double> heightVec_;

  std::string robotModel_;

  Eigen::Vector3d basePosition_;
  std::map<std::string, raisim::DepthCamera *> depthCameras_;
  std::map<std::string, raisim::SpinningLidar *> lidars_;
  std::map<std::string, std::vector<raisim::Vec<3>>> pointClouds_;
  std::map<std::string, double> updateTimeStamps_;
  std::map<std::string, int> camWidth_, camHeight_;
  std::map<std::string, PoseBuffer> poseBuffers_;
  std::map<std::string, std::unique_ptr<std::mutex>> mutexPerCamera_;
  std::map<std::string, std::unique_ptr<std::mutex>> mutexPerLidar_;

  // grid map config
  double mapResol_;
  double lmSize_;
  double lmSizeHalf_;
  int lmGridSize_;
  double maxCount_;

  // trust region
  double velTrustThreshold_;
  double distThreshold_;
  double heightThresholdMinimum_;
  double heightThresholdProportional_;
  Eigen::MatrixXf heightThresholdMap_;

  // ray casting
  bool isRc_;
  int rcIterIdx_;
  int rcDownsamplingKernel_;

  // grid mapping ray downsampling
  int mappingIterIdx_;
  int mappingDownsamplingKernel_;

  // lidar param
  std::vector<raisim::Vec<3>> lidar_map_pc;
  std::vector<raisim::Vec<3>> octomap_pc;
  raisim::InstancedVisuals* scans_;
  double lidarTimeStamp_ = 0;
  double imuTimeStamp_ = 0;

  // lidar mutex
  std::mutex lidar_mutex_;
  std::mutex odom_mutex_;

  // Grid mapping
  Eigen::Matrix3d baseOrientation_;
  grid_map::GridMap tempHeightMap;

  // global map
  std::unique_ptr<std::mutex> map_mutex_;

  // Odometry
  raisim::Mat<3,3> rotBaseToLidar_;
  raisim::Vec<3> posBaseToLidar_;

  // Octomap
  std::mutex octree_mutex_;
  std::unordered_map<std::string, std::unique_ptr<octomap::OcTree>> ocTrees_;

  // raisim Sensors
  raisim::InertialMeasurementUnit* lidar_imu_;

  // logging
  double lidarMapUpdateElapsedTime_ = 0;
  double cameraMapUpdateElapsedTime_ = 0;
  double advanceElapsedTime_ = 0;
};

//class LidarGridMappingReal: public LidarGridMapping
//{
// public:
//  explicit LidarGridMappingReal(raisim::World & world, raisim::RaisimServer & server,
//                     raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);
//
//  bool advance() override;
//
//  void createSubscriber();
//
//  void lidarCallbackSub(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
//
// private:
//  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidarSubscription_;
//
//
//  livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg_livox;
//};

//class LidarGridMappingSim: public LidarGridMapping
//{
// public:
//  explicit LidarGridMappingSim(raisim::World & world, raisim::RaisimServer & server,
//                    raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);
//
//  bool advance() override;
//
// private:
//
////  raisim::SpinningLidar * lidarSim_;
////  raisim::InertialMeasurementUnit * lidarImuSim_;
////  raisim::InertialMeasurementUnit * imuSim_;
//
//};


}     // plugin
} // raisin

#endif //RAISIN_PLUGIN_LIDAR_GRID_MAPPING_HPP
