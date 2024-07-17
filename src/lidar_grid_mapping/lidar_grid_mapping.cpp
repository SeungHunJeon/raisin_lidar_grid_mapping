// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

//
// Created by jsh on 24. 5. 2.
//

#include "lidar_grid_mapping/lidar_grid_mapping.hpp"

namespace raisin
{
namespace plugin
{

LidarGridMapping::LidarGridMapping(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
: rclcpp::Node("LidarGridMapping"), Plugin(world, server, worldSim, serverSim, globalResource), param_(parameter::ParameterContainer::getRoot()["raisin_lidar_grid_mapping_plugin"])
{
  pluginType_ = PluginType::MAPPING;

  param_.loadFromPackageParameterFile("raisin_lidar_grid_mapping_plugin");

  velTrustThreshold_ = param_("velocity_trust_threshold");
  distThreshold_ = param_("distance_threshold");
  heightThresholdMinimum_ = param_("height_threshold_minimum");
  heightThresholdProportional_ = param_("height_threshold_proportional");
  isRc_ = param_["ray_casting"]("is_ray_casting");
  rcIterIdx_ = 0;
  rcDownsamplingKernel_ = param_["ray_casting"]("downsampling_kernel");
  mappingIterIdx_ = 0;
  mappingDownsamplingKernel_ = param_("grid_mapping_downsampling_kernel");

  logIdx_ = dataLogger_.initializeAnotherDataGroup(
      "LidarGridMapping",
      "cameraMapUpdateElapsedTime", cameraMapUpdateElapsedTime_,
      "lidarMapUpdateElapsedTime", lidarMapUpdateElapsedTime_,
      "advanceElapsedTime", advanceElapsedTime_ 
  );

  // for (int i = 0; i < 10000; i++) {
  //   server.addVisualBox("sphere" + std::to_string(i), 0.025, 0.025, 0.025, 1, 0, 0, 1);
  // }
}

LidarGridMapping::~LidarGridMapping()
{
  for (const auto & pair: depthCameras_) {
    auto depthCamera = pair.first;
    std::lock_guard<std::mutex> guard(*mutexPerCamera_[depthCamera]);
  } // wait for detached mapUpdate to end

  for (const auto & pair: lidars_) {
    auto lidarName = pair.first;
    std::lock_guard<std::mutex> guard(*mutexPerLidar_[lidarName]);
  }

  worldHub_.removeObject(raisimHeightMap_);
  auto ground = worldHub_.addGround();
  ground->setName("terrain");
}

bool LidarGridMapping::init()
{
  robotModel_ = std::string((paramRoot_)["Raibo"]("robot_model"));

  initMap();
  initSensor();

  return true;
}

void LidarGridMapping::initMap()
{
  mapResol_ = param_("map_resolution");
  lmSize_ = param_("local_map_size");
  maxCount_ = param_("max_count");
  lmGridSize_ = (int) (lmSize_ / mapResol_);
  lmSize_ = lmGridSize_ * mapResol_;
  lmSizeHalf_ = lmSize_ / 2.;

  mapReset();

  convertToHeightVec();
  robotHub_->lockMutex();
  basePosition_ = robotHub_->getBasePosition().e();
  robotHub_->unlockMutex();

  worldHub_.removeObject(worldHub_.getObject("terrain"));

  serverHub_.lockVisualizationServerMutex();
  raisimHeightMap_ = worldHub_.addHeightMap(
    lmGridSize_, lmGridSize_, lmSize_, lmSize_,
    basePosition_(0), basePosition_(1), heightVec_);
  raisimHeightMap_->setName("terrain");

  serverHub_.unlockVisualizationServerMutex();

  grid_map::Index centerIdx;
  localMap_.getIndex(localMap_.getPosition(), centerIdx);
  heightThresholdMap_.setZero(lmGridSize_, lmGridSize_);
  for (int row = 0; row < lmGridSize_; ++row) {
    for (int col = 0; col < lmGridSize_; ++col) {
      grid_map::Index currentIdx(row, col);
      int dx = currentIdx.x() - centerIdx.x();
      int dy = currentIdx.y() - centerIdx.y();
      float indexDist = std::sqrt(dx * dx + dy * dy);

      heightThresholdMap_(
        row,
        col) = heightThresholdMinimum_ + indexDist * mapResol_ * heightThresholdProportional_;
    }
  }
}

void LidarGridMapping::initSensor()
{
  bool align = (paramRoot_)["Raibo"]["sensors"]["d435i_front"]("align_depth_to_rgb");
  std::vector<std::string> d435Modules, d430Modules, lidarModules;

  if (robotModel_ == "raibo1") {  // Raibo1
    d435Modules = {"d435i_R", "d435i_L"};
    d430Modules = {};
  } else if (robotModel_ == "raibo2") { // Raibo2
    d435Modules = {};
    d430Modules = {"d430_front", "d430_rear"};
    lidarModules = {"lidar_link"};
  } else {
    RSWARN("Undefined robot model: " << robotModel_)
  }


  for (const auto & d435Module: d435Modules) {
    auto sensorSet = robotHub_->getSensorSet(d435Module);
    if (align) {
      depthCameras_[d435Module] =
          sensorSet->getSensor<raisim::DepthCamera>("depth_aligned");
    } else {
      depthCameras_[d435Module] = sensorSet->getSensor<raisim::DepthCamera>("depth");
    }
    camWidth_[d435Module] = depthCameras_[d435Module]->getProperties().width;
    camHeight_[d435Module] = depthCameras_[d435Module]->getProperties().height;

    updateTimeStamps_[d435Module] = -1.0;
    pointClouds_[d435Module] = std::vector<raisim::Vec<3>>{};
  }

  for (const auto & d430Module: d430Modules) {
    auto sensorSet = robotHub_->getSensorSet(d430Module);
    depthCameras_[d430Module] = sensorSet->getSensor<raisim::DepthCamera>("depth");
    camWidth_[d430Module] = depthCameras_[d430Module]->getProperties().width;
    camHeight_[d430Module] = depthCameras_[d430Module]->getProperties().height;

    updateTimeStamps_[d430Module] = -1.0;
    pointClouds_[d430Module] = std::vector<raisim::Vec<3>>{};
  }

  for (const auto & pair: depthCameras_) {
    auto depthCamera = pair.first;
    isNearRobot_[depthCamera] = std::vector<bool>(camWidth_[depthCamera] * camHeight_[depthCamera]);
    poseBuffers_.emplace(depthCamera, PoseBuffer(100));
    mutexPerCamera_[depthCamera] = std::make_unique<std::mutex>();
  }
  for (const std::string & leg :{"LF", "RF", "LH", "RH"}) {
    for (const std::string & part :{"THIGH", "SHANK"}) {
      poseBuffers_.emplace(leg + "_" + part, PoseBuffer(100));
    }
  }

  for (const auto & lidarModule: lidarModules) {
    auto sensorSet = robotHub_->getSensorSet(lidarModule);
    lidars_[lidarModule] = sensorSet->getSensor<raisim::SpinningLidar>("lidar");
    poseBuffers_.emplace(lidarModule, PoseBuffer(100));
    updateTimeStamps_[lidarModule] = -1.0;
    pointClouds_[lidarModule] = std::vector<raisim::Vec<3>>{};
    mutexPerLidar_[lidarModule] = std::make_unique<std::mutex>();

    // Octree generation
    double leafSize = param_("octree_leaf_size");
    std::unique_ptr<octomap::OcTree> tree = std::make_unique<octomap::OcTree>(leafSize);
    ocTrees_[lidarModule] = std::move(tree);
  }
}

void LidarGridMapping::mapReset()
{
  // std::lock_guard<std::mutex> lock(map_mutex_);

  robotHub_->lockMutex();
  basePosition_ = robotHub_->getBasePosition().e();
  robotHub_->unlockMutex();

  localMap_ = grid_map::GridMap({"sum", "mean", "count", "max"});
  localMap_.setFrameId("robot");
  localMap_.setGeometry(
    grid_map::Length(lmSize_, lmSize_), mapResol_,
    grid_map::Position(basePosition_[0], basePosition_[1]));
  localMap_["sum"].setConstant(0);
  localMap_["max"].setConstant(0);
  localMap_["count"].setConstant(0);
}

void LidarGridMapping::convertToHeightVec(const std::string & layerName)
{
  grid_map::Size size = localMap_.getSize();
  grid_map::Matrix mat = localMap_[layerName];

  heightVec_.resize(mat.size());
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
    heightVec_.data(), size[0], size[1]) = mat.cast<double>();
}

bool LidarGridMapping::advance()
{
  std::chrono::time_point<std::chrono::high_resolution_clock> advanceStart_ =
    std::chrono::high_resolution_clock::now();
  // add pose to the buffer
  double timeStamp = worldHub_.getWorldTime();
  raisim::Vec<3> position;
  raisim::Mat<3, 3> rot;

  for (auto & lidarPair : lidars_) {
    std::string lidarName = lidarPair.first;
    raisim::SpinningLidar * lidar = lidarPair.second;

    lidar->lockMutex();
    lidar->updatePose();
    const raisim::Vec<3> & lidarPos = lidar->getPosition();
    const raisim::Mat<3, 3> & lidarOri = lidar->getOrientation();
    poseBuffers_[lidarName].addPose(timeStamp, lidarPos.e(), lidarOri.e());
    lidar->unlockMutex();
  }

  for (auto & lidarPair: lidars_) {
    std::string lidarName = lidarPair.first;
    raisim::SpinningLidar* lidar = lidarPair.second;

    lidar->lockMutex();
    double updateTimeStamp = lidar->getUpdateTimeStamp();
    lidar->unlockMutex();

    if (updateTimeStamp - updateTimeStamps_[lidarName] > 1e-6) {
      updateTimeStamps_[lidarName] = updateTimeStamp;
      
      // read state
      robotHub_->lockMutex();
      Eigen::VectorXd gv = robotHub_->getGeneralizedVelocity().e();
      robotHub_->unlockMutex();

      bool noDrift = (gv.head(3).norm() < velTrustThreshold_);

      if (noDrift) {
        std::thread updateThread(&LidarGridMapping::lidarMapUpdate, this, lidarName);
        updateThread.detach();
      } else {
        // state drift case --> reset mapping
        mapReset();
        convertToHeightVec();
        RSINFO("Height map reset due to state estimation drift")
      }

      // update raisim heightmap object
      raisimHeightMap_->lockMutex();
      raisimHeightMap_->update(
        localMap_.getPosition()[0],
        localMap_.getPosition()[1], lmSize_, lmSize_, heightVec_);
      raisimHeightMap_->unlockMutex();
    }
  }

  for (const std::string & linkName :{"LF", "RF", "LH", "RH"}) {
    robotHub_->getPosition(
      robotHub_->getBodyIdx(linkName + "_THIGH"), raisim::Vec<3>({-0.04575, 0, -0.17283}),
      position);
    robotHub_->getOrientation(robotHub_->getBodyIdx(linkName + "_THIGH"), rot);
    poseBuffers_[linkName + "_THIGH"].addPose(timeStamp, position.e(), rot.e());

    robotHub_->getPosition(
      robotHub_->getBodyIdx(linkName + "_SHANK"), raisim::Vec<3>({0.00606, 0, -0.13162}),
      position);
    robotHub_->getOrientation(robotHub_->getBodyIdx(linkName + "_SHANK"), rot);
    poseBuffers_[linkName + "_SHANK"].addPose(timeStamp, position.e(), rot.e());
  }

  for (auto & camPair : depthCameras_) {
    std::string linkName = camPair.first;
    raisim::DepthCamera * depthCamera = camPair.second;

    depthCamera->lockMutex();
    depthCamera->updatePose();
    const raisim::Vec<3> & camPos = depthCamera->getPosition();
    const raisim::Mat<3, 3> & camOri = depthCamera->getOrientation();
    poseBuffers_[linkName].addPose(timeStamp, camPos.e(), camOri.e());
    depthCamera->unlockMutex();
  }

  for (auto & camPair : depthCameras_) {
    std::string linkName = camPair.first;
    raisim::DepthCamera * depthCamera = camPair.second;

    depthCamera->lockMutex();
    double updateTimeStamp = depthCamera->getUpdateTimeStamp();
    depthCamera->unlockMutex();

    if (updateTimeStamp - updateTimeStamps_[linkName] > 1e-6) {
      updateTimeStamps_[linkName] = updateTimeStamp;

      // read state
      robotHub_->lockMutex();
      Eigen::VectorXd gv = robotHub_->getGeneralizedVelocity().e();
      robotHub_->unlockMutex();

      bool noDrift = (gv.head(3).norm() < velTrustThreshold_);

      if (noDrift) {
        std::thread updateThread(&LidarGridMapping::mapUpdate, this, linkName);
        updateThread.detach();
      } else {
        // state drift case --> reset mapping
        mapReset();
        convertToHeightVec();
        RSINFO("Height map reset due to state estimation drift")
      }

      // update raisim heightmap object
      raisimHeightMap_->lockMutex();
      raisimHeightMap_->update(
        localMap_.getPosition()[0],
        localMap_.getPosition()[1], lmSize_, lmSize_, heightVec_);
      raisimHeightMap_->unlockMutex();
    }
  }

  std::lock_guard<std::mutex> guard(mtx_);
  // thresholding at the maximum number of count
  localMap_["sum"] = (localMap_["count"].array() > maxCount_).select(
    localMap_["sum"].array() * maxCount_ / localMap_["count"].array(), localMap_["sum"].array());
  localMap_["count"] = (localMap_["count"].array() >= maxCount_).select(
    maxCount_, localMap_["count"].array());

  // if (isRc_) {
  //   rcIterIdx_ = (rcIterIdx_ + 1) % (rcDownsamplingKernel_ * rcDownsamplingKernel_);
  // }

  mappingIterIdx_ = (mappingIterIdx_ + 1) %
    (mappingDownsamplingKernel_ * mappingDownsamplingKernel_);

   std::chrono::time_point<std::chrono::high_resolution_clock> advanceEnd_ =
   std::chrono::high_resolution_clock::now();

  advanceElapsedTime_ = std::chrono::duration_cast<std::chrono::microseconds>(
   advanceEnd_ - advanceStart_).count() / 1.e6;

  // RSINFO(elapsedTime)

  dataLogger_.append(
    logIdx_,
    cameraMapUpdateElapsedTime_,
    lidarMapUpdateElapsedTime_,
    advanceElapsedTime_);

  return true;
}

void LidarGridMapping::convertToOctree(const std::string & lidarName)
{
  static int idx = 0;

  if(idx == 5) {
    ocTrees_[lidarName]->clear();
    idx = 0;
  }
  
  for (const auto& point : pointClouds_[lidarName]) {
    ocTrees_[lidarName]->updateNode(octomap::point3d(point[0], point[1], point[2]), true);
  }

  ocTrees_[lidarName]->updateInnerOccupancy();

  std::vector<raisim::Vec<3>> temp_octomap_pc;
  int num_point = 0;
  raisim::Vec<3> pos;  
  for (octomap::OcTree::tree_iterator it = ocTrees_[lidarName]->begin_tree(),
                                      end = ocTrees_[lidarName]->end_tree();
      it != end; ++it)
  {
    if (it.isLeaf() && ocTrees_[lidarName]->isNodeOccupied(*it))
    {
        octomap::point3d coord = it.getCoordinate();
        double occupancy = it->getOccupancy();
        double size = it.getSize();
        // RSINFO("Occupied node at" << coord.x() << ", " << coord.y() << ", " << coord.z() << ", occupancy: " << occupancy << ", size : " << size)
        pos.e() << coord.x(),coord.y(),coord.z();
        temp_octomap_pc.push_back(pos);
        //  if(idx < 10000) {
        //    serverSim_.getVisualObject("sphere" + std::to_string(idx))->setPosition(pos.e());
        //  }
        num_point++;
    }
  }
  // Lock the mutex only when updating the shared resource
  pointClouds_[lidarName].swap(temp_octomap_pc);

  RSINFO("num point : " << num_point)

  idx++;
}

void LidarGridMapping::readOctree(const std::string & lidarName)
{
}

void LidarGridMapping::lidarMapUpdate(const std::string & lidarName) {
  std::chrono::time_point<std::chrono::high_resolution_clock> advanceStart_ =
    std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> guardPerLidar(*mutexPerLidar_[lidarName]);

  raisim::SpinningLidar * lidar = lidars_[lidarName];
  double timeStamp = lidar->getUpdateTimeStamp();
  
  auto pose = poseBuffers_[lidarName].getClosestPose(timeStamp);
  const Eigen::Vector3d & lidarPos = pose.position;
  const Eigen::Matrix3d & lidarOri = pose.orientation;

  std::vector<raisim::Vec<3>> tempPointCloud = lidar->getScan();
  pointClouds_[lidarName].swap(tempPointCloud);;
  
  // Process the point cloud
  
  for (auto & point : pointClouds_[lidarName]) {
    point.e() = lidarOri * point.e() + lidarPos;
  }
  
  robotHub_->lockMutex();
  basePosition_ = robotHub_->getBasePosition().e();
  robotHub_->unlockMutex();

  // this->convertToOctree(lidarName);

  pointClouds_[lidarName] = voxelGridFilter(pointClouds_[lidarName], param_("octree_leaf_size"));

  // update map
  grid_map::Position position(basePosition_[0], basePosition_[1]);

  grid_map::GridMap tempHeightMap;
  tempHeightMap = grid_map::GridMap({"sum", "count", "mean", "max"});
  tempHeightMap.setGeometry(grid_map::Length(lmSize_, lmSize_), mapResol_, position);
  tempHeightMap["sum"].setConstant(0);
  tempHeightMap["max"].setConstant(0);
  tempHeightMap["count"].setConstant(0);

  double h_thres = 0.7;
  int idx = 0;
  // Eigen::Vector3d transformed_point;
  for (auto & transformed_point : pointClouds_[lidarName]) {
    grid_map::Index index;

    if (!tempHeightMap.getIndex(transformed_point.e().head(2), index)) {
      continue;
    }    
    if(transformed_point[2] < basePosition_[2] + h_thres) {
    // if(idx < 10000) {
    //   serverHub_.getVisualObject("sphere" + std::to_string(idx))->setPosition(point.e());
    //   idx ++;
    // } 
      tempHeightMap["sum"](index(0), index(1)) += transformed_point(2);
      if (tempHeightMap["count"](index(0), index(1)) < 0.5) {
        tempHeightMap["max"](index(0), index(1)) = static_cast<float>(transformed_point(2));
      } else {
        tempHeightMap["max"](index(0), index(1)) = std::max(
          tempHeightMap["max"](index(0), index(
            1)), static_cast<float>(transformed_point(2)));
      }
      tempHeightMap["count"](index(0), index(1)) = 1;  
    }
  }


  tempHeightMap["mean"] = tempHeightMap["sum"].cwiseQuotient(tempHeightMap["count"]);

  std::lock_guard<std::mutex> guard(mtx_);
  localMap_.move(position);

  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> mismatchMask =
    (tempHeightMap["max"] - localMap_["mean"]).array().abs() > heightThresholdMap_.array();
  mismatchMask = mismatchMask && !(tempHeightMap["count"].array() < 0.5) &&
    !(localMap_["mean"].array().isNaN());
  localMap_["count"] = mismatchMask.select((localMap_["count"] / 2.), localMap_["count"]);

  // mismatch update & ray casting update
  localMap_["sum"] = localMap_["mean"].cwiseProduct(localMap_["count"]);
  localMap_["sum"] = localMap_["mean"].array().isNaN().select(0., localMap_["sum"]);

  // input update
  localMap_["count"] += tempHeightMap["count"];
  localMap_["sum"] += tempHeightMap["max"];

  // compute the mean height map
  localMap_["mean"] = localMap_["sum"].cwiseQuotient(localMap_["count"]);
  localMap_["max"] = localMap_["max"].cwiseMax(tempHeightMap["max"]);
  
  // RSINFO("Map updating done! Conver to HeightVec")
  this->convertToHeightVec();

  std::chrono::time_point<std::chrono::high_resolution_clock> advanceEnd_ =
   std::chrono::high_resolution_clock::now();

  lidarMapUpdateElapsedTime_ = std::chrono::duration_cast<std::chrono::microseconds>(
   advanceEnd_ - advanceStart_).count() / 1.e6;
}

// Define the voxel grid filter function
std::vector<raisim::Vec<3>> LidarGridMapping::voxelGridFilter(const std::vector<raisim::Vec<3>>& pointCloud, double voxelSize) {
  // Map to store points by voxel index
  tbb::concurrent_hash_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, Vector3iHash> voxelMap;

  // Insert points into voxel grid in parallel
  tbb::parallel_for(size_t(0), pointCloud.size(), [&](size_t i) {
    const auto& point = pointCloud[i];
    Eigen::Vector3d p(point[0], point[1], point[2]);
    Eigen::Vector3i voxelIdx = (p / voxelSize).cast<int>();

    tbb::concurrent_hash_map<Eigen::Vector3i, std::vector<Eigen::Vector3d>, Vector3iHash>::accessor accessor;
    voxelMap.insert(accessor, voxelIdx);
    accessor->second.push_back(p);
  });

  // Create filtered point cloud
  std::vector<raisim::Vec<3>> filteredPointCloud;
  filteredPointCloud.reserve(voxelMap.size());

  for (const auto& voxel : voxelMap) {
    const auto& points = voxel.second;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
      centroid += p;
    }
    centroid /= points.size();

    raisim::Vec<3> filteredPoint;
    filteredPoint[0] = centroid.x();
    filteredPoint[1] = centroid.y();
    filteredPoint[2] = centroid.z();
    filteredPointCloud.push_back(filteredPoint);
  }

  return filteredPointCloud;
}

void LidarGridMapping::mapUpdate(const std::string & linkName) {
  std::chrono::time_point<std::chrono::high_resolution_clock> advanceStart_ =
    std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> guardPerCamera(*mutexPerCamera_[linkName]);

  raisim::DepthCamera * depthCamera = depthCameras_[linkName];
  double timeStamp = depthCamera->getUpdateTimeStamp();
  const std::vector<float> depthArray = depthCamera->getDepthArray();
  const int & camWidth = camWidth_[linkName];
  const int & camHeight = camHeight_[linkName];

  auto pose = poseBuffers_[linkName].getClosestPose(timeStamp);
  const Eigen::Vector3d & camPos = pose.position;
  const Eigen::Matrix3d & camOri = pose.orientation;

  depthCamera->depthToPointCloud(depthArray, pointClouds_[linkName], true);
  std::vector<raisim::Vec<3>> & pointCloud = pointClouds_[linkName];
  for (auto & point : pointCloud) {
    point.e() = camOri * point.e() + camPos;
  }

  robotHub_->lockMutex();
  basePosition_ = robotHub_->getBasePosition().e();
  robotHub_->unlockMutex();

  // update map
  grid_map::Position position(basePosition_[0], basePosition_[1]);

  grid_map::GridMap tempHeightMap;
  tempHeightMap = grid_map::GridMap({"sum", "count", "mean", "max"});
  tempHeightMap.setGeometry(grid_map::Length(lmSize_, lmSize_), mapResol_, position);
  tempHeightMap["sum"].setConstant(0);
  tempHeightMap["max"].setConstant(0);
  tempHeightMap["count"].setConstant(0);

  std::vector<std::string> legsToCheck; // legs to check if a depth is projected to.

  // TODO: check camera link name
  if (linkName == "d430_front") {
    legsToCheck = {"LF", "RF"};
  } else if (linkName == "d430_bottom") {
    legsToCheck = {"LF", "RF", "LH", "RH"};
  } else if (linkName == "d430_rear") {
    legsToCheck = {"LH", "RH"};
  }

  isNearRobot(pointCloud, legsToCheck, timeStamp, isNearRobot_[linkName]);

  int idx = 0;
  for (const float & depth : depthArray) {
    if (!downsamplingSelection(camWidth, idx, mappingIterIdx_, mappingDownsamplingKernel_)) {
      ++idx;
      continue;
    }

    const Eigen::Vector3d & point = pointCloud[idx].e();

    if (isNearRobot_[linkName][idx]) {
      ++idx;
      continue;
    }

    // if (isRc_ && downsamplingSelection(camWidth, idx, rcIterIdx_, rcDownsamplingKernel_)) {
    //   rayCasting(camPos, point);
    // }

    if (depth > distThreshold_ && depth < lmSizeHalf_) {
      grid_map::Index index;
      if (!tempHeightMap.getIndex(point.head(2), index)) {
        ++idx;
        continue;  // Skip this point if it does not lie within the elevation map.
      }

      tempHeightMap["sum"](index(0), index(1)) += point(2);
      if (tempHeightMap["count"](index(0), index(1)) < 0.5) {
        tempHeightMap["max"](index(0), index(1)) = static_cast<float>(point(2));
      } else {
        tempHeightMap["max"](index(0), index(1)) = std::max(
          tempHeightMap["max"](index(0), index(
            1)), static_cast<float>(point(2)));
      }
      tempHeightMap["count"](index(0), index(1)) = 1;
    }
    ++idx;
  }
  tempHeightMap["mean"] = tempHeightMap["sum"].cwiseQuotient(tempHeightMap["count"]);

  std::lock_guard<std::mutex> guard(mtx_);
  localMap_.move(position);
  // compensate mismatch of current input and localMap_
  Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> mismatchMask =
    (tempHeightMap["max"] - localMap_["mean"]).array().abs() > heightThresholdMap_.array();
  mismatchMask = mismatchMask && !(tempHeightMap["count"].array() < 0.5) &&
    !(localMap_["mean"].array().isNaN());
  localMap_["count"] = mismatchMask.select((localMap_["count"] / 2.), localMap_["count"]);

  // mismatch update & ray casting update
  localMap_["sum"] = localMap_["mean"].cwiseProduct(localMap_["count"]);
  localMap_["sum"] = localMap_["mean"].array().isNaN().select(0., localMap_["sum"]);

  // input update
  localMap_["count"] += tempHeightMap["count"];
  localMap_["sum"] += tempHeightMap["max"];

  // compute the mean height map
  localMap_["mean"] = localMap_["sum"].cwiseQuotient(localMap_["count"]);
  localMap_["max"] = localMap_["max"].cwiseMax(tempHeightMap["max"]);

  this->convertToHeightVec();

  std::chrono::time_point<std::chrono::high_resolution_clock> advanceEnd_ =
   std::chrono::high_resolution_clock::now();

  cameraMapUpdateElapsedTime_ = std::chrono::duration_cast<std::chrono::microseconds>(
   advanceEnd_ - advanceStart_).count() / 1.e6;
}

inline bool LidarGridMapping::downsamplingSelection(
  const int & camWidth, const int & rayIdx,
  const int & iterIdx, const int & kernelSize)
{
  if ((rayIdx % camWidth + rayIdx / camWidth) % (kernelSize * kernelSize) == iterIdx) {
    return true;
  } else {return false;}
}

void LidarGridMapping::isNearRobot(
  std::vector<raisim::Vec<3>> & pointCloud, std::vector<std::string> legsToCheck,
  double timeStamp, std::vector<bool> & result)
{
  Eigen::Vector3d position_thigh, position_shank;
  Eigen::Matrix3d rot_thigh, rot_shank;
  Eigen::Vector3d point_thigh, point_shank;

  for (int pointIdx = 0; pointIdx < pointCloud.size(); pointIdx++) {
    result[pointIdx] = false;
  }

  double halfLength_thigh = 0.15;
  double halfLength_shank = 0.2;
  double radius_thigh_squared = 0.1 * 0.1;
  double radius_shank_squared = 0.1 * 0.1;
  for (int idx = 0; idx < legsToCheck.size(); idx++) {
    std::string linkName = legsToCheck[idx];
    auto poseThigh = poseBuffers_[linkName + "_THIGH"].getClosestPose(timeStamp);
    auto poseShank = poseBuffers_[linkName + "_SHANK"].getClosestPose(timeStamp);
    position_thigh = poseThigh.position;
    rot_thigh = poseThigh.orientation;
    position_shank = poseShank.position;
    rot_shank = poseShank.orientation;

    for (int pointIdx = 0; pointIdx < pointCloud.size(); pointIdx++) {
      Eigen::Vector3d point = pointCloud[pointIdx].e();
      point_thigh = rot_thigh.transpose() * (point - position_thigh);
      point_shank = rot_shank.transpose() * (point - position_shank);
      if (((point_thigh[2] < halfLength_thigh && point_thigh[2] > -halfLength_thigh) &&
        (point_thigh[0] * point_thigh[0] + point_thigh[1] * point_thigh[1] <
        radius_thigh_squared)) ||
        ((point_shank[2] < halfLength_shank && point_shank[2] > -halfLength_shank) &&
        (point_shank[0] * point_shank[0] + point_shank[1] * point_shank[1] < radius_shank_squared)))
      {
        result[pointIdx] = true;
      }
    }
  }
}

extern "C" Plugin * create(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
{
  return new LidarGridMapping(world, server, worldSim, serverSim, globalResource);
}

extern "C" void destroy(Plugin * p)
{
  delete p;
}
}     // plugin
} // raisin
