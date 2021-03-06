#ifndef SRC_WIDGET_KITTIREADER_H_
#define SRC_WIDGET_KITTIREADER_H_

#include <stdint.h>
#include <QtCore/QDir>
#include <QtCore/QString>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include "common.h"
#include "data/kitti_utils.h"

// PCL includes
#include "pcd_custom_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/impl/io.hpp> // SHOULD NOT BE INCLUDED AS THE FIRST LINE!! Leads to compilation error: ‘int pcl::getFieldIndex(const pcl::PointCloud<PointT>&, const string&, std::vector<pcl::PCLPointField>&)’ should have been declared inside ‘pcl’.


/** \brief tile-based KITTI reader.
 *
 *  Given a size of a tile, the reader reads all scans that potentially overlap with the tile.
 *  If a scan overlaps with the tile is determined by a circle to square overlap test.
 *
 *  Thus, it might happen that the circle of radius max_distance overlaps with the square defined by the tile,
 *  but there are actually no points inside the tile. However, we cannot check this at this point in time,
 *  since we would have to open all point clouds and this would take forever.
 *
 *  \see settings.cfg
 *
 *  \author behley
 */

class KittiReader {
 public:

  bool b_bin_cloud = 0; // 0 = PCD XYZI or XYZIR file, 1 = binary XYZI file
  bool b_bin_label = 0; // 0 = ASCII labels file, 1 = binary labels file
  bool b_no_instances_support = 1; // Skip code related to instances. Use default instances.txt file with "0:0" in it.

  struct Tile {
    int32_t i, j;                   // tile coordinates
    std::vector<uint32_t> indexes;  // scan indexes
    float x, y, size;               // actual world coordinates.
  };

  /** \brief get poses and filenames of velodyne, labels, etc.
   *  If the labels directory does not exist, the directory is created.
   **/
  void initialize(const QString& directory);

  /** \brief number of scans. **/
  uint32_t count() const { return velodyne_filenames_.size(); }

  void setMaximumDistance(float distance) { maxDistance_ = distance; }

  /** \brief get points, labels, and images for given world coordinates. **/
  void retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels, std::vector<std::string>& images);

  /** \brief get points, labels, and images for given indexes. **/
  void retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                std::vector<LabelsPtr>& labels, std::vector<std::string>& images);

  /** \brief update labels for given scan indexes. **/
  void update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels);

  void setTileSize(float size);

  const std::vector<Tile>& getTiles() const { return tiles_; }

  const Tile& getTile(const Eigen::Vector3f& pos) const;
  const Tile& getTile(uint32_t i, uint32_t j) const;

  const Eigen::Vector2i& numTiles() const { return numTiles_; }

  const std::vector<Eigen::Vector2f>& getTileTrajectory() const { return trajectory_; }

  std::map<uint32_t, uint32_t> getMaxInstanceIds() const { return maxInstanceIds_; }

  void updateMetaInformation(const std::map<uint32_t, uint32_t>& maxInstanceIds);

 protected:
  void readPoints(const std::string& filename, Laserscan& scan);
  // void readLabels(const std::string& filename, std::vector<uint32_t>& labels);
  void readLabels(const std::string& filename, std::vector<uint32_t>& labels, uint32_t cloud_num_points = 0);
  void readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses);

  QDir base_dir_;
  KITTICalibration calib_;
  std::vector<Eigen::Matrix4f> poses_;
  std::vector<std::string> velodyne_filenames_;
  std::vector<std::string> label_filenames_;
  std::vector<std::string> image_filenames_;

  using PT_XYZ = pcl::PointXYZ;
  using PT_XYZI = pcl::PointXYZI;
  using PT_XYZIR = velodyne_pointcloud::PointXYZIR;
  using PT_TYPE = PT_XYZIR;

  // cache reads from before.
  std::map<uint32_t, PointcloudPtr> pointsCache_;
  std::map<uint32_t, LabelsPtr> labelCache_;

  float maxDistance_{15.0f};

  inline uint32_t tileIdxToOffset(uint32_t i, uint32_t j) const { return i + j * numTiles_.x(); }

  float tileSize_{50};
  std::vector<Tile> tiles_;
  Eigen::Vector2f offset_;
  Eigen::Vector2i numTiles_;

  std::vector<Eigen::Vector2f> trajectory_;

  std::map<uint32_t, uint32_t> maxInstanceIds_;
};

#endif /* SRC_WIDGET_KITTIREADER_H_ */
