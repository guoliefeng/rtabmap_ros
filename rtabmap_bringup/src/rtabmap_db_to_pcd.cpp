/*
 * Export all optimized LiDAR map components stored in an RTAB-Map database.
 *
 * This intentionally reads the compressed LaserScan payloads directly instead
 * of regenerating occupancy grids.  Occupancy-grid regeneration can discard a
 * 3D scan when its segmentation parameters do not match the recorded sensor.
 */

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{

struct Options
{
  std::string database;
  std::string output;
  float voxelSize = 0.25f;
  float rangeMin = 2.0f;
  float rangeMax = 100.0f;
};

void usage(const char * executable)
{
  std::cerr
      << "Usage: " << executable
      << " --database FILE --output FILE [--voxel-size M]"
         " [--range-min M] [--range-max M]\n";
}

float parsePositiveFloat(const std::string & value, const std::string & name, bool allowZero)
{
  std::size_t parsed = 0;
  const float result = std::stof(value, &parsed);
  if(parsed != value.size() || !std::isfinite(result) ||
     (allowZero ? result < 0.0f : result <= 0.0f))
  {
    throw std::runtime_error("Invalid " + name + ": " + value);
  }
  return result;
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for(int i = 1; i < argc; ++i)
  {
    const std::string argument(argv[i]);
    if(argument == "-h" || argument == "--help")
    {
      usage(argv[0]);
      std::exit(0);
    }
    if(i + 1 >= argc)
    {
      throw std::runtime_error("Missing value for " + argument);
    }
    const std::string value(argv[++i]);
    if(argument == "--database")
    {
      options.database = value;
    }
    else if(argument == "--output")
    {
      options.output = value;
    }
    else if(argument == "--voxel-size")
    {
      options.voxelSize = parsePositiveFloat(value, argument, false);
    }
    else if(argument == "--range-min")
    {
      options.rangeMin = parsePositiveFloat(value, argument, true);
    }
    else if(argument == "--range-max")
    {
      options.rangeMax = parsePositiveFloat(value, argument, false);
    }
    else
    {
      throw std::runtime_error("Unknown argument: " + argument);
    }
  }
  if(options.database.empty() || options.output.empty())
  {
    throw std::runtime_error("Both --database and --output are required");
  }
  if(options.rangeMax <= options.rangeMin)
  {
    throw std::runtime_error("--range-max must be greater than --range-min");
  }
  return options;
}

template<typename PointT>
void voxelize(typename pcl::PointCloud<PointT>::Ptr & cloud, float leafSize)
{
  if(cloud->empty())
  {
    return;
  }
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(leafSize, leafSize, leafSize);
  filter.setInputCloud(cloud);
  typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  filter.filter(*filtered);
  cloud.swap(filtered);
}

bool fileExists(const std::string & path)
{
  std::ifstream stream(path.c_str(), std::ios::binary);
  return stream.good();
}

struct PoseSelection
{
  std::map<int, rtabmap::Transform> poses;
  std::size_t activeOdomPoses = 0;
  std::size_t savedOptimizedPoses = 0;
  std::size_t connectedComponents = 0;
  std::size_t savedComponents = 0;
  std::size_t reoptimizedComponents = 0;
  std::size_t odometryFallbackComponents = 0;
};

PoseSelection loadAllComponentPoses(rtabmap::DBDriver & driver)
{
  PoseSelection selection;
  std::map<int, rtabmap::Transform> allOdomPoses;
  std::map<int, rtabmap::Transform> odomPoses;
  driver.getAllOdomPoses(allOdomPoses, false, false);
  // DBDriver's ignoreIntermediateNodes flag is not consistent across RTAB-Map
  // database versions.  Read the weight explicitly: weight=-9 marks an
  // intermediate node, while retained mapping nodes have weight>-9.
  for(std::map<int, rtabmap::Transform>::const_iterator iter = allOdomPoses.begin();
      iter != allOdomPoses.end(); ++iter)
  {
    int weight = -9;
    driver.getWeight(iter->first, weight);
    if(iter->first > 0 && weight > -9 && !iter->second.isNull())
    {
      odomPoses.insert(*iter);
    }
  }
  selection.activeOdomPoses = odomPoses.size();
  if(odomPoses.empty())
  {
    throw std::runtime_error("Database contains no active odometry poses");
  }

  const std::map<int, rtabmap::Transform> savedPoses = driver.loadOptimizedPoses();
  for(std::map<int, rtabmap::Transform>::const_iterator iter = savedPoses.begin();
      iter != savedPoses.end(); ++iter)
  {
    if(odomPoses.find(iter->first) != odomPoses.end() && !iter->second.isNull())
    {
      ++selection.savedOptimizedPoses;
    }
  }

  std::multimap<int, rtabmap::Link> allLinks;
  driver.getAllLinks(allLinks, true, false);
  std::multimap<int, rtabmap::Link> activeLinks;
  for(std::multimap<int, rtabmap::Link>::const_iterator iter = allLinks.begin();
      iter != allLinks.end(); ++iter)
  {
    if(odomPoses.find(iter->second.from()) != odomPoses.end() &&
       odomPoses.find(iter->second.to()) != odomPoses.end())
    {
      activeLinks.insert(*iter);
    }
  }

  const rtabmap::ParametersMap parameters = driver.getLastParameters();
  std::unique_ptr<rtabmap::Optimizer> optimizer(rtabmap::Optimizer::create(parameters));
  if(!optimizer)
  {
    throw std::runtime_error("Cannot create the RTAB-Map graph optimizer");
  }

  std::map<int, rtabmap::Transform> remaining = odomPoses;
  while(!remaining.empty())
  {
    const int rootId = remaining.begin()->first;
    std::map<int, rtabmap::Transform> componentPoses;
    std::multimap<int, rtabmap::Link> componentLinks;
    optimizer->getConnectedGraph(
        rootId, odomPoses, activeLinks, componentPoses, componentLinks);
    if(componentPoses.empty())
    {
      componentPoses.insert(*remaining.begin());
    }
    ++selection.connectedComponents;

    bool savedComponentComplete = true;
    for(std::map<int, rtabmap::Transform>::const_iterator iter = componentPoses.begin();
        iter != componentPoses.end(); ++iter)
    {
      if(savedPoses.find(iter->first) == savedPoses.end() ||
         savedPoses.at(iter->first).isNull())
      {
        savedComponentComplete = false;
        break;
      }
    }

    if(savedComponentComplete)
    {
      for(std::map<int, rtabmap::Transform>::const_iterator iter = componentPoses.begin();
          iter != componentPoses.end(); ++iter)
      {
        selection.poses.insert(std::make_pair(iter->first, savedPoses.at(iter->first)));
      }
      ++selection.savedComponents;
    }
    else
    {
      std::map<int, rtabmap::Transform> optimized =
          optimizer->optimize(rootId, componentPoses, componentLinks);
      if(optimized.size() == componentPoses.size())
      {
        selection.poses.insert(optimized.begin(), optimized.end());
        ++selection.reoptimizedComponents;
      }
      else
      {
        // FAST-LIO odometry is already expressed in the shared GPS-aligned
        // ENU frame.  Keep the export complete if a disconnected component
        // cannot be optimized, and report the fallback explicitly.
        selection.poses.insert(componentPoses.begin(), componentPoses.end());
        ++selection.odometryFallbackComponents;
      }
    }

    for(std::map<int, rtabmap::Transform>::const_iterator iter = componentPoses.begin();
        iter != componentPoses.end(); ++iter)
    {
      remaining.erase(iter->first);
    }
  }

  if(selection.poses.size() != odomPoses.size())
  {
    throw std::runtime_error(
        "Failed to select a pose for every active mapping node");
  }
  return selection;
}

}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    const Options options = parseOptions(argc, argv);
    if(fileExists(options.output))
    {
      throw std::runtime_error("Refusing to overwrite existing output: " + options.output);
    }

    std::unique_ptr<rtabmap::DBDriver> driver(rtabmap::DBDriver::create());
    if(!driver || !driver->openConnection(options.database, false, true))
    {
      throw std::runtime_error("Cannot open RTAB-Map database read-only: " + options.database);
    }

    const PoseSelection poseSelection = loadAllComponentPoses(*driver);
    const std::map<int, rtabmap::Transform> & poses = poseSelection.poses;

    pcl::PointCloud<pcl::PointXYZI>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZI>);
    std::size_t inputPoints = 0;
    std::size_t finiteRangePoints = 0;
    std::size_t exportedNodes = 0;
    std::size_t emptyScans = 0;
    const float minRangeSquared = options.rangeMin * options.rangeMin;
    const float maxRangeSquared = options.rangeMax * options.rangeMax;

    std::cout << "pose_source=all_connected_components"
              << " poses=" << poses.size()
              << " active_odom_poses=" << poseSelection.activeOdomPoses
              << " saved_optimized_poses=" << poseSelection.savedOptimizedPoses
              << " components=" << poseSelection.connectedComponents
              << " saved_components=" << poseSelection.savedComponents
              << " reoptimized_components=" << poseSelection.reoptimizedComponents
              << " odometry_fallback_components="
              << poseSelection.odometryFallbackComponents << '\n';

    for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
        iter != poses.end(); ++iter)
    {
      if(iter->first <= 0 || iter->second.isNull())
      {
        continue;
      }

      rtabmap::SensorData data;
      driver->getNodeData(iter->first, data, false, true, false, false);
      rtabmap::LaserScan scan;
      data.uncompressDataConst(0, 0, &scan);
      if(scan.empty())
      {
        ++emptyScans;
        continue;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr local =
          rtabmap::util3d::laserScanToPointCloudI(scan, scan.localTransform());
      inputPoints += local->size();

      pcl::PointCloud<pcl::PointXYZI>::Ptr valid(new pcl::PointCloud<pcl::PointXYZI>);
      valid->reserve(local->size());
      for(pcl::PointCloud<pcl::PointXYZI>::const_iterator point = local->begin();
          point != local->end(); ++point)
      {
        if(!std::isfinite(point->x) || !std::isfinite(point->y) ||
           !std::isfinite(point->z) || !std::isfinite(point->intensity))
        {
          continue;
        }
        const float rangeSquared = point->x * point->x + point->y * point->y +
                                   point->z * point->z;
        if(rangeSquared >= minRangeSquared && rangeSquared <= maxRangeSquared)
        {
          valid->push_back(*point);
        }
      }
      finiteRangePoints += valid->size();
      voxelize<pcl::PointXYZI>(valid, options.voxelSize);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed =
          rtabmap::util3d::transformPointCloud(valid, iter->second);
      *assembled += *transformed;
      ++exportedNodes;

      // Bound peak memory without changing the final requested voxel size.
      if(exportedNodes % 100 == 0)
      {
        voxelize<pcl::PointXYZI>(assembled, options.voxelSize);
        std::cout << "processed_nodes=" << exportedNodes
                  << " assembled_points=" << assembled->size() << '\n';
      }
    }

    voxelize<pcl::PointXYZI>(assembled, options.voxelSize);
    assembled->width = static_cast<std::uint32_t>(assembled->size());
    assembled->height = 1;
    assembled->is_dense = true;
    if(assembled->empty())
    {
      throw std::runtime_error("All database scans were empty or filtered out");
    }
    if(pcl::io::savePCDFileBinary(options.output, *assembled) != 0)
    {
      throw std::runtime_error("Failed to write PCD: " + options.output);
    }

    driver->closeConnection(false);
    std::cout << std::fixed << std::setprecision(3)
              << "export_complete=true\n"
              << "exported_nodes=" << exportedNodes << '\n'
              << "connected_components=" << poseSelection.connectedComponents << '\n'
              << "saved_components=" << poseSelection.savedComponents << '\n'
              << "reoptimized_components=" << poseSelection.reoptimizedComponents << '\n'
              << "odometry_fallback_components="
              << poseSelection.odometryFallbackComponents << '\n'
              << "empty_scans=" << emptyScans << '\n'
              << "input_points=" << inputPoints << '\n'
              << "finite_range_points=" << finiteRangePoints << '\n'
              << "output_points=" << assembled->size() << '\n'
              << "voxel_size_m=" << options.voxelSize << '\n'
              << "range_min_m=" << options.rangeMin << '\n'
              << "range_max_m=" << options.rangeMax << '\n'
              << "output=" << options.output << '\n';
    return 0;
  }
  catch(const std::exception & exception)
  {
    std::cerr << "rtabmap_db_to_pcd: " << exception.what() << '\n';
    usage(argv[0]);
    return 2;
  }
}
