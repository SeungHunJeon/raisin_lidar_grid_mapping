/*
 * grid_map.cpp (modified from GridMap.cpp)
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *	Modified on: March 11, 2024
 *	    Author: Juhyeok Mun
 *   Institute: RAILAB, KAIST
 */


#include "grid_mapping/grid_map.hpp"

#include <cmath>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

using std::cout;
using std::endl;
using std::isfinite;

namespace grid_map
{

GridMap::GridMap(const std::vector<std::string> & layers)
{
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  size_.setZero();
  timestamp_ = 0;
  layers_ = layers;

  for (auto & layer : layers_) {
    data_.insert(std::pair<std::string, Matrix>(layer, Matrix()));
  }
}

GridMap::GridMap()
: GridMap(std::vector<std::string>()) {}

void GridMap::setGeometry(const Length & length, const double resolution, const Position & position)
{
  assert(length(0) > 0.0);
  assert(length(1) > 0.0);
  assert(resolution > 0.0);

  Size size;
  size(0) = static_cast<int>(round(length(0) / resolution));      // There is no round() function in Eigen.
  size(1) = static_cast<int>(round(length(1) / resolution));
  resize(size);
  clearAll();

  resolution_ = resolution;
  length_ = (size_.cast<double>() * resolution_).matrix();
  position_ = position;
}

void GridMap::setBasicLayers(const std::vector<std::string> & basicLayers)
{
  basicLayers_ = basicLayers;
}

const std::vector<std::string> & GridMap::getBasicLayers() const
{
  return basicLayers_;
}

bool GridMap::hasBasicLayers() const
{
  return !basicLayers_.empty();
}

bool GridMap::hasSameLayers(const GridMap & other) const
{
  return std::all_of(
    layers_.begin(), layers_.end(),
    [&](const std::string & layer) {return other.exists(layer);});
}

void GridMap::add(const std::string & layer, const double value)
{
  add(layer, Matrix::Constant(size_(0), size_(1), value));
}

void GridMap::add(const std::string & layer, const Matrix & data)
{
  assert(size_(0) == data.rows());
  assert(size_(1) == data.cols());

  if (exists(layer)) {
    // Type exists already, overwrite its data.
    data_.at(layer) = data;
  } else {
    // Type does not exist yet, add type and data.
    data_.insert(std::pair<std::string, Matrix>(layer, data));
    layers_.push_back(layer);
  }
}

bool GridMap::exists(const std::string & layer) const
{
  return !(data_.find(layer) == data_.end());
}

const Matrix & GridMap::get(const std::string & layer) const
{
  try {
    return data_.at(layer);
  } catch (const std::out_of_range & exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer '" + layer + "' available.");
  }
}

Matrix & GridMap::get(const std::string & layer)
{
  try {
    return data_.at(layer);
  } catch (const std::out_of_range & exception) {
    throw std::out_of_range("GridMap::get(...) : No map layer of type '" + layer + "' available.");
  }
}

const Matrix & GridMap::operator[](const std::string & layer) const
{
  return get(layer);
}

Matrix & GridMap::operator[](const std::string & layer)
{
  return get(layer);
}

bool GridMap::erase(const std::string & layer)
{
  const auto dataIterator = data_.find(layer);
  if (dataIterator == data_.end()) {
    return false;
  }
  data_.erase(dataIterator);

  const auto layerIterator = std::find(layers_.begin(), layers_.end(), layer);
  if (layerIterator == layers_.end()) {
    return false;
  }
  layers_.erase(layerIterator);

  const auto basicLayerIterator = std::find(basicLayers_.begin(), basicLayers_.end(), layer);
  if (basicLayerIterator != basicLayers_.end()) {
    basicLayers_.erase(basicLayerIterator);
  }

  return true;
}

const std::vector<std::string> & GridMap::getLayers() const
{
  return layers_;
}

float & GridMap::atPosition(const std::string & layer, const Position & position)
{
  Index index;
  if (getIndex(position, index)) {
    return at(layer, index);
  }
  throw std::out_of_range("GridMap::atPosition(...) : Position is out of range.");
}

float & GridMap::at(const std::string & layer, const Index & index)
{
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range & exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer + "' available.");
  }
}

float GridMap::at(const std::string & layer, const Index & index) const
{
  try {
    return data_.at(layer)(index(0), index(1));
  } catch (const std::out_of_range & exception) {
    throw std::out_of_range("GridMap::at(...) : No map layer '" + layer + "' available.");
  }
}

bool GridMap::getIndex(const Position & position, Index & index) const
{
  Vector offset = (0.5 * length_).matrix();

  Vector indexVector = ((position + offset - position_).array() / resolution_).matrix();
  index = Index{indexVector[0], indexVector[1]};

  return isInside(position);
}

bool GridMap::getPosition(const Index & index, Position & position) const
{
  Vector offset = (0.5 * length_).matrix();
  position = position_ - offset + resolution_ * Vector{index[0], index[1]};

  return index[0] >= 0 && index[1] >= 0 && index[0] < size_[0] && index[1] < size_[1];
}

bool GridMap::isInside(const Position & position) const
{
  Vector offset = (0.5 * length_).matrix();
  Position positionTransformed = position - position_ + offset;

  return positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0 &&
         positionTransformed.x() < length_(0) && positionTransformed.y() < length_(1);
}

bool GridMap::isValid(DataType value) const
{
  return isfinite(value);
}

bool GridMap::isValid(const Index & index) const
{
  return isValid(index, basicLayers_);
}

bool GridMap::isValid(const Index & index, const std::string & layer) const
{
  return isValid(at(layer, index));
}

bool GridMap::isValid(const Index & index, const std::vector<std::string> & layers) const
{
  if (layers.empty()) {
    return false;
  }
  return std::all_of(
    layers.begin(), layers.end(),
    [&](const std::string & layer) {return isValid(index, layer);});
}

bool GridMap::getPosition3(
  const std::string & layer, const Index & index,
  Position3 & position) const
{
  const auto value = at(layer, index);
  if (!isValid(value)) {
    return false;
  }
  Position position2d;
  getPosition(index, position2d);
  position.head(2) = position2d;
  position.z() = value;
  return true;
}

bool GridMap::getVector(
  const std::string & layerPrefix, const Index & index,
  Eigen::Vector3d & vector) const
{
  Eigen::Vector3d temp{at(layerPrefix + "x", index), at(layerPrefix + "y", index), at(
      layerPrefix + "z", index)};
  if (!isValid(temp[0]) || !isValid(temp[1]) || !isValid(temp[2])) {
    return false;
  } else {
    vector = temp;
    return true;
  }
}

void GridMap::setPosition(const Position & position)
{
  position_ = position;
}

bool GridMap::move(const Position & position)
{
  Position positionShift = position - position_;
  Vector indexShiftVectorTemp = (positionShift.array() / resolution_).matrix();
  Eigen::Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++) {
    indexShiftVector[i] =
      static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  }
  Index indexShift = Eigen::Array2i{indexShiftVector[0], indexShiftVector[1]};
  Position alignedPositionShift = Vector{indexShift[0], indexShift[1]} *resolution_;

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++) {
    if (indexShift(i) != 0) {
      if (abs(indexShift(i)) >= getSize()(i)) {
        // Entire map is dropped.
        for (auto & layer : layers_) {
          data_.at(layer).setConstant(0);
        }
      } else {
        int sign = (indexShift(i) > 0 ? 1 : -1);
        int nCells = abs(indexShift(i));
        if (i == 0) {
          int rest = size_(0) - nCells;
          if (sign > 0) {
            for (auto & layer : layers_) {
              Matrix temp = data_.at(layer).bottomRows(rest);
              data_.at(layer).topRows(rest) = temp;
              data_.at(layer).bottomRows(nCells).setConstant(0);
            }
          } else {
            for (auto & layer : layers_) {
              Matrix temp = data_.at(layer).topRows(rest);
              data_.at(layer).bottomRows(rest) = temp;
              data_.at(layer).topRows(nCells).setConstant(0);
            }
          }
        } else if (i == 1) {
          int rest = size_(1) - nCells;
          if (sign > 0) {
            for (auto & layer : layers_) {
              Matrix temp = data_.at(layer).rightCols(rest);
              data_.at(layer).leftCols(rest) = temp;
              data_.at(layer).rightCols(nCells).setConstant(0);
            }
          } else {
            for (auto & layer : layers_) {
              Matrix temp = data_.at(layer).leftCols(rest);
              data_.at(layer).rightCols(rest) = temp;
              data_.at(layer).leftCols(nCells).setConstant(0);
            }
          }
        }
      }
    }
  }

  // Update information.
  position_ += alignedPositionShift;

  // Check if map has been moved at all.
  return indexShift.any();
}

void GridMap::setTimestamp(const Time timestamp)
{
  timestamp_ = timestamp;
}

Time GridMap::getTimestamp() const
{
  return timestamp_;
}

void GridMap::resetTimestamp()
{
  timestamp_ = 0.0;
}

void GridMap::setFrameId(const std::string & frameId)
{
  frameId_ = frameId;
}

const std::string & GridMap::getFrameId() const
{
  return frameId_;
}

const Length & GridMap::getLength() const
{
  return length_;
}

const Position & GridMap::getPosition() const
{
  return position_;
}

double GridMap::getResolution() const
{
  return resolution_;
}

const Size & GridMap::getSize() const
{
  return size_;
}

void GridMap::clear(const std::string & layer)
{
  try {
    data_.at(layer).setConstant(0);
  } catch (const std::out_of_range & exception) {
    throw std::out_of_range("GridMap::clear(...) : No map layer '" + layer + "' available.");
  }
}

void GridMap::clearBasic()
{
  for (auto & layer : basicLayers_) {
    clear(layer);
  }
}

void GridMap::clearAll()
{
  for (auto & data : data_) {
    data.second.setConstant(0);
  }
}

void GridMap::clearRows(unsigned int index, unsigned int nRows)
{
  for (auto & layer : layers_) {
    data_.at(layer).block(index, 0, nRows, getSize()(1)).setConstant(0);
  }
}

void GridMap::clearCols(unsigned int index, unsigned int nCols)
{
  for (auto & layer : layers_) {
    data_.at(layer).block(0, index, getSize()(0), nCols).setConstant(0);
  }
}

void GridMap::resize(const Index & size)
{
  size_ = size;
  for (auto & data : data_) {
    data.second.resize(size_(0), size_(1));
  }
}


}  // namespace grid_map
