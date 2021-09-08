#include <stdint.h>
#include <widget/KittiReader.h>
#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include "rv/string_utils.h"

#include <boost/lexical_cast.hpp>

// Rakshith: Initialize the point labeller tool
void KittiReader::initialize(const QString& directory) {
  std::cout << ">>> KittiReader Init." << std::endl;
  velodyne_filenames_.clear();
  label_filenames_.clear();
  image_filenames_.clear();
  poses_.clear();

  pointsCache_.clear();
  labelCache_.clear();
  tiles_.clear();

  base_dir_ = QDir(directory);
  QDir velodyne_dir;
  QDir velodyne_dir_1(base_dir_.filePath("velodyne"));
  QDir velodyne_dir_2(base_dir_.filePath("velodyne_pcd"));
  if(b_bin_cloud) velodyne_dir = velodyne_dir_1;
  else velodyne_dir = velodyne_dir_2;
  QStringList entries = velodyne_dir.entryList(QDir::Files, QDir::Name);
  for (int32_t i = 0; i < entries.size(); ++i) {
    velodyne_filenames_.push_back(velodyne_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir_.exists("calib.txt"))
    throw std::runtime_error("Missing calibration file: " + base_dir_.filePath("calib.txt").toStdString());

  calib_.initialize(base_dir_.filePath("calib.txt").toStdString());

  readPoses(base_dir_.filePath("poses.txt").toStdString(), poses_);

  std::cout << ">>> Create label files for each lidar scan and set 0 as class id." << std::endl;
  // create label dir, etc.
  QDir labels_dir;
  QDir labels_dir_1(base_dir_.filePath("labels"));
  QDir labels_dir_2(base_dir_.filePath("labels_ascii"));
  if(b_bin_label) labels_dir = labels_dir_1;
  else labels_dir = labels_dir_2;
  // find corresponding label files.
  if (!labels_dir.exists()) {
    if(b_bin_label) base_dir_.mkdir("labels");
    else base_dir_.mkdir("labels_ascii");
  }

  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename;
    if(b_bin_label) filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label";
    else filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".label_ascii";

    // std::cout << "filename_velo: " << velodyne_filenames_[i].c_str() << std::endl;
    std::cout << "filename_label: " << labels_dir.filePath(filename).toStdString().c_str() << std::endl;

    if (!labels_dir.exists(filename)) {

      if(b_bin_label) {
        std::ifstream in(velodyne_filenames_[i].c_str());
        in.seekg(0, std::ios::end);
        uint32_t num_points = in.tellg() / (4 * sizeof(float));
        in.close();

        std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());

        // Rakshith: Initialize the labels file for each scan that is loaded
        // Each labels file has the same name as the scan name but ends with .labels
        // The labels file has scan.size() number of integers in the file, 
        // each representing the class that a particular point from that scan belongs to
        // HOW ARE MULTIPLE LABLES ENCODED?? --- disabled by default since "overwrite labels" is checked in the toolbar on top. Anyway, not needed for my application
        std::vector<uint32_t> labels(num_points, 0);
        out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));

        out.close();
      }
      else {
        std::string path = velodyne_filenames_[i].c_str();
        pcl::PointCloud<PT_TYPE>::Ptr tmpCloud (new pcl::PointCloud<PT_TYPE>);        
        if (pcl::io::loadPCDFile<PT_TYPE> (path, *tmpCloud) == -1) {
            PCL_ERROR ("Couldn't read PCD file. \n");
        }
        int num_points = tmpCloud->size();

       
        std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());
        std::string str;
        str += "FORMAT: POINT_IDX_IN_CLOUD, CLAstr_LABEL\n";
        for(int i = 0; i < num_points; ++i) {
          str += std::to_string(i) + " " + std::to_string((int)0); 
        }
        out << str;
        out.close();
      }
    }

    label_filenames_.push_back(labels_dir.filePath(filename).toStdString().c_str());
  }

  std::string missing_img = QDir::currentPath().toStdString() + "/../assets/missing.png";
  QDir image_dir(base_dir_.filePath("image_2"));
  for (uint32_t i = 0; i < velodyne_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(velodyne_filenames_[i])).baseName() + ".png";
    if (image_dir.exists(filename)) {
      image_filenames_.push_back(image_dir.filePath(filename).toStdString());
    } else {
      image_filenames_.push_back(missing_img);
    }
  }

  // assumes that (0,0,0) is always the start.
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();

  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector4f t = poses_[i].col(3);

    min.x() = std::min(t.x() - maxDistance_, min.x());
    min.y() = std::min(t.y() - maxDistance_, min.y());
    max.x() = std::max(t.x() + maxDistance_, max.x());
    max.y() = std::max(t.y() + maxDistance_, max.y());
  }

  //  std::cout << "tileSize = " << tileSize_ << std::endl;
  //  std::cout << "min = " << min << ", max = " << max << std::endl;

  offset_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;
  offset_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;

  //  std::cout << "offset = " << offset_ << std::endl;

  numTiles_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.x() - 0.5 * tileSize_) / tileSize_) + 1;
  numTiles_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.y() - 0.5 * tileSize_) / tileSize_) + 1;

  //  std::cout << "numTiles = " << numTiles_ << std::endl;

  tiles_.clear();
  tiles_.resize(numTiles_.x() * numTiles_.y());

  Eigen::Vector2f idxRadius(maxDistance_ / tileSize_, maxDistance_ / tileSize_);

  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      tile.i = i;
      tile.j = j;
      tile.x = i * tileSize_ - offset_.x() + 0.5 * tileSize_;
      tile.y = j * tileSize_ - offset_.y() + 0.5 * tileSize_;
      tile.size = tileSize_;
    }
  }

  trajectory_.clear();

  Eigen::Vector2f e(0.5 * tileSize_, 0.5 * tileSize_);
  for (uint32_t i = 0; i < poses_.size(); ++i) {
    Eigen::Vector2f t = poses_[i].col(3).head(2);
    Eigen::Vector2f idx((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_);

    trajectory_.push_back(Eigen::Vector2f((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_));

    //    tiles_[tileIdxToOffset(uint32_t(idx.x()), uint32_t(idx.y()))].indexes.push_back(i);
    //    uint32_t u_min = std::max(int32_t(idx.x() - idxRadius.x()), 0);
    //    uint32_t u_max = std::min(int32_t(std::ceil(idx.x() + idxRadius.x())), numTiles_.x());
    //    uint32_t v_min = std::max(int32_t(idx.y() - idxRadius.y()), 0);
    //    uint32_t v_max = std::min(int32_t(std::ceil(idx.y() + idxRadius.y())), numTiles_.y());

    // FIXME: workaround check all tiles.
    for (uint32_t u = 0; u < uint32_t(numTiles_.x()); ++u) {
      for (uint32_t v = 0; v < uint32_t(numTiles_.y()); ++v) {
        auto& tile = tiles_[tileIdxToOffset(u, v)];
        Eigen::Vector2f q = t - Eigen::Vector2f(tile.x, tile.y);
        q[0] = std::abs(q[0]);
        q[1] = std::abs(q[1]);

        // check for exact overlap (see Behley et al., ICRA, 2015)
        if (std::max(q[0], q[1]) > e[0] + maxDistance_) continue;  // definitely outside.
        if (std::min(q[0], q[1]) < e[0] || (q - e).norm() < maxDistance_) {
          tile.indexes.push_back(i);
        }
      }
    }
  }

  // sanity check:

  for (auto& t : tiles_) {
    std::sort(t.indexes.begin(), t.indexes.end());
    //    std::cout << "Tile has " << t.indexes.size() << " tiles associated." << std::endl;
    for (uint32_t i = 1; i < t.indexes.size(); ++i) {
      if (t.indexes[i - 1] == t.indexes[i]) {
        std::cout << "found duplicate!" << std::endl;
      }
    }
  }

  uint32_t tileCount = 0;
  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      std::sort(tile.indexes.begin(), tile.indexes.end());
      if (tile.indexes.size() > 0) tileCount += 1;
    }
  }

  std::cout << "#tiles  = " << tileCount << std::endl;

  // meta information for faster loading.
  if (base_dir_.exists("instances.txt")) {
    std::cout << "Reading instances.txt..." << std::flush;

    std::ifstream in(base_dir_.filePath("instances.txt").toStdString());

    while (in.good()) {
      std::string line;
      std::getline(in, line);
      if (line.size() == 0) break;

      std::vector<std::string> tokens = rv::split(line, ":");
      if (tokens.size() != 2) {
        throw std::runtime_error("Invalid instance meta information found!");
      }

      uint32_t label = boost::lexical_cast<uint32_t>(tokens[0]);
      uint32_t maxInstanceId = boost::lexical_cast<uint32_t>(tokens[1]);
      maxInstanceIds_[label] = maxInstanceId;

      in.peek();
    }

    in.close();
    std::cout << "finished." << std::endl;
    std::cout << "-----------" << std::endl;
  } else {
    std::cout << "Generating intances.txt" << std::flush;
    // get the counts from the label files.
    for (const std::string& filename : label_filenames_) {
      std::vector<uint32_t> labels;
      readLabels(filename, labels);

      for (uint32_t instance_label : labels) {
        uint32_t instanceId = (instance_label >> 16) & uint32_t(0xFFFF);
        uint32_t label = instance_label & uint32_t(0xFFFF);
        if (maxInstanceIds_.find(label) == maxInstanceIds_.end())
          maxInstanceIds_[label] = instanceId;
        else
          maxInstanceIds_[label] = std::max(instanceId, maxInstanceIds_[label]);
      }
    }

    // directly update meta information:
    updateMetaInformation(maxInstanceIds_);
  }
}

void KittiReader::updateMetaInformation(const std::map<uint32_t, uint32_t>& maxInstanceIds) {
  std::ofstream out(base_dir_.filePath("instances.txt").toStdString().c_str());
  for (auto it = maxInstanceIds.begin(); it != maxInstanceIds.end(); ++it) {
    out << it->first << ":" << it->second << std::endl;
  }
  out.close();
}

void KittiReader::retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes,
                           std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels,
                           std::vector<std::string>& images) {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);

  retrieve(idx.x(), idx.y(), indexes, points, labels, images);
}

// Rakshith: Read scans on opening a folder.
void KittiReader::retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels, std::vector<std::string>& images) {
  
  std::cout << ">>> Reading cloud and label files." << std::endl;

  indexes.clear();
  points.clear();
  labels.clear();
  images.clear();

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  indexes = tiles_[tileIdxToOffset(i, j)].indexes;
  std::sort(indexes.begin(), indexes.end());
  // loop through all scan
  for (uint32_t t : indexes) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      // Rakshith: Function that reads lidar scans from binary file
      std::cout << ">>> Reading cloud file: " << velodyne_filenames_[t] << std::endl;
      readPoints(velodyne_filenames_[t], *points.back());
      pointsCache_[t] = points.back();
      points.back()->pose = poses_[t];

      labels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      // Rakshith: Function that reads lidar scans from binary file
      std::cout << ">>> Reading label file: " << label_filenames_[t] << std::endl;
      uint32_t num_points = points.back()->size(); // To prevent resizing of labels vector should pass cloud size
      readLabels(label_filenames_[t], *labels.back(), num_points);
      labelCache_[t] = labels.back();

      if (points.back()->size() != labels.back()->size()) {
        std::cout << "Filename: " << velodyne_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << points.back()->size() << " vs. num. labels = " << labels.back()->size()
                  << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

    } else {
      points.push_back(pointsCache_[t]);
      labels.push_back(labelCache_[t]);
    }

    images.push_back(image_filenames_[t]);
  }

  std::cout << ">>> " << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }
}

const KittiReader::Tile& KittiReader::getTile(const Eigen::Vector3f& position) const {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);
  return tiles_[tileIdxToOffset(idx.x(), idx.y())];
}
const KittiReader::Tile& KittiReader::getTile(uint32_t i, uint32_t j) const { return tiles_[tileIdxToOffset(i, j)]; }

void KittiReader::setTileSize(float size) { tileSize_ = size; }

// Rakshith: Update label file based on current labels marked in tool. 
// Update is done across all scans in memory (not just those visible/toggled in the viewer)
void KittiReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {
  std::cout << ">>> Updating labels as per changes made in viewer." << std::endl;
  for (uint32_t i = 0; i < indexes.size(); ++i) {
    if (labels[i]->size() == 0) {
      std::cout << "Warning: 0 labels?" << std::endl;
      continue;
    }

    if (pointsCache_.find(indexes[i]) == pointsCache_.end()) {
      std::cout << "Warning: labels of non cached points?" << std::endl;
      continue;
    }

    if (labels[i]->size() != pointsCache_[indexes[i]]->size()) {
      std::cout << "Warning: inconsistent numbers of labels for given point cloud!" << std::endl;

      continue;
    }

    if (label_filenames_.size() < indexes[i]) {
      std::cout << "Warning: wrong index?" << std::endl;

      continue;
    }

    if(b_bin_label) {
      // Rakshith: Update label file. By default triggered on pressing save button.
      std::ofstream out(label_filenames_[indexes[i]].c_str());
      // labels = vector<LabelsPtr>;
      // From commons.h: typedef std::shared_ptr<std::vector<uint32_t>> LabelsPtr;
      // So, labels[i] => LabelsPtr => std::vector<uint32_t>
      // (const char*)&(*labels[i])[0] ==> labels[i] = ptr to label data-struct, * dereferences it, 
      // Of that, reference/memo-addr of 0th idx is typecasted to (const char*)
      // And a chunk of labels[i]->size() * sizeof(unint32_t) is written to disk as binary. 
      out.write((const char*)&(*labels[i])[0], labels[i]->size() * sizeof(uint32_t));
      out.close();
    } 
    else {
      std::cout << ">>> Updating label file: " << label_filenames_[indexes[i]].c_str() << std::endl;
      std::ofstream out(label_filenames_[indexes[i]].c_str());

      std::string str;
      str += "FORMAT: POINT_IDX_IN_CLOUD, CLAstr_LABEL\n";
      
      LabelsPtr labels_for_this_scan = labels[i]; // labels for this scan
      int num_points = labels_for_this_scan->size(); // Assuming one label per points, number of labels = number of points in this scan
      for(int i = 0; i < num_points; ++i) {

        int class_label_for_point = (*labels_for_this_scan)[i];
        str += std::to_string(i) + " " + std::to_string( class_label_for_point ) + "\n";
      }
      out << str;
      
      out.close();
    }
  }
}

void KittiReader::readPoints(const std::string& filename, Laserscan& scan) {
  
  if(b_bin_cloud) {
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (!in.is_open()) return;

    scan.clear();

    in.seekg(0, std::ios::end);
    uint32_t num_points = in.tellg() / (4 * sizeof(float));
    in.seekg(0, std::ios::beg);

    std::vector<float> values(4 * num_points);
    in.read((char*)&values[0], 4 * num_points * sizeof(float));

    //@Rakshith
    // std::cout << "num_points: " << num_points << std::endl;
    // std::cout << "sizeof(float): " << sizeof(float) << std::endl;
    // std::cout << "4*sizeof(float): " << 4*sizeof(float) << std::endl;
    // std::cout << "4*num_points*sizeof(float): " << 4*num_points*sizeof(float) << std::endl;

    in.close();

    // Operations are on REFERENCE of "scan.points" and "scan.remissions". 
    // So effectively changing "points" and "remissions" variables is changing the "scan" variable!! 
    // By convention "scan" should have been accepted as a reference in the funciton call itself rather than here.
    std::vector<Point3f>& points = scan.points;
    std::vector<float>& remissions = scan.remissions;

    points.resize(num_points);
    remissions.resize(num_points);

    for (uint32_t i = 0; i < num_points; ++i) {
      points[i].x = values[4 * i];
      points[i].y = values[4 * i + 1];
      points[i].z = values[4 * i + 2];
      remissions[i] = values[4 * i + 3];
    }
  }
  else
  {
    scan.clear();

    // Operations are on REFERENCE of "scan.points" and "scan.remissions". 
    // So effectively changing "points" and "remissions" variables is changing the "scan" variable!! 
    // By convention "scan" should have been accepted as a reference in the funciton call itself rather than here.
    std::vector<Point3f>& points = scan.points;
    std::vector<float>& remissions = scan.remissions;

    pcl::PointCloud<PT_TYPE>::Ptr tmpCloud (new pcl::PointCloud<PT_TYPE>);
    if (pcl::io::loadPCDFile<PT_TYPE> (filename, *tmpCloud) == -1) {
        PCL_ERROR ("Couldn't read PCD file. \n");
    }
    int num_points = tmpCloud->size(); 
    std::cout << "size: " << num_points << std::endl; 

    points.resize(num_points);
    remissions.resize(num_points);

    PT_TYPE pt;
    for (int i = 0; i < num_points; ++i) {
      pt = tmpCloud->points[i];
      points[i].x = pt.x;
      points[i].y = pt.y;
      points[i].z = pt.z;
      remissions[i] = pt.intensity;
    }
  }
}

// void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels, uint32_t cloud_num_points) {

  if(b_bin_label) {
    std::ifstream in(filename.c_str(), std::ios::binary);
    if (!in.is_open()) {
      std::cerr << "Unable to open label file. " << std::endl;
      return;
    }

    labels.clear();

    in.seekg(0, std::ios::end);
    uint32_t num_points = in.tellg() / (sizeof(uint32_t));
    in.seekg(0, std::ios::beg);

    labels.resize(num_points);
    in.read((char*)&labels[0], num_points * sizeof(uint32_t));

    in.close();
  }
  else
  {
    labels.clear();
    labels.resize(cloud_num_points); // need to re-read PCD file to figure size of cloud so just passing as parameter (OR) can just let C++ resize automatically.

    std::ifstream in(filename.c_str(), std::ios_base::in);

    // Get rid of header lines
    std::string line;
    std::getline(in, line);
    // std::cout << "HEADER LINE 1: " << line << std::endl;

    // process other lines
    int ctr = 0;
    std::string point_index_str, class_label_str;
    uint32_t point_index, class_label;
    while (std::getline(in, line))
    {
        // Read labels
        std::stringstream ss(line);
        ss >> point_index_str >> class_label_str;

        point_index = std::stoi(point_index_str);
        class_label = std::stoi(class_label_str);

        // For now, we are sure that point_indices are continuous and 
        // no missing/skipped numbers exist so can use point_index stored in files
        labels[point_index] = class_label;
        // labels[ctr] = class_label; // otherwise just use a counter variable.

        ctr += 1;
    }
    std::cout << "size: " << ctr << std::endl;
  }
  
}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {

  std::cout << ">>> Reading pose file." << std::endl;

  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to velodyne coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }

}


// TODO Rakshith Tree Mapping:
/*

* Note process: load multiple view and label single, multiple in range, or all based on what is visible in an overlapping manner
save all that are in memory (not only visible ones) to file

* Label output in Ascii. 
* Label loading from Ascii.

* Depending on the labelling method and how points are being stored, may or may not need to read from pcd. 
* MIGHT MAKE THIS TASK EASY IF LABELS ARE ADDED POINTWISE.
* OR even if reading from PCD, just do it such that it is read from pcd and then changed to binary as required by rest of the tool.
* Reading pcd as pcd files than binary -- for easy no pcd to binary conversion, XYZIR info retained rather than just XYZI, individual manipulation, ability to read as ascii when opened and later rviz visualization, etc.

* change extension to .pcd and .asciilabel once changes have been made.




Useful features:

single scan
scan range

follow pose

brush vs polygon

overlap labels

automatically save

selecting tiles to control total RAM used

view: perspective vs orthographic


*/
