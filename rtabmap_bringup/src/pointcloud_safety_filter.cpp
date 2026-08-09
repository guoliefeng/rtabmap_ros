#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace
{

bool fieldOffset(const sensor_msgs::PointCloud2 & cloud, const std::string & name, uint32_t & offset)
{
	for(const sensor_msgs::PointField & field : cloud.fields)
	{
		if(field.name == name && field.datatype == sensor_msgs::PointField::FLOAT32)
		{
			offset = field.offset;
			return true;
		}
	}
	return false;
}

float readFloat32(const uint8_t * point, uint32_t offset)
{
	float value = std::numeric_limits<float>::quiet_NaN();
	std::memcpy(&value, point + offset, sizeof(value));
	return value;
}

class PointCloudSafetyFilter
{
public:
	PointCloudSafetyFilter() : privateNh_("~"), maxRange_(300.0), minPoints_(20), removedPoints_(0), droppedMessages_(0)
	{
		privateNh_.param("input_topic", inputTopic_, std::string("/fast_lio_ns/cloud_registered_body"));
		privateNh_.param("output_topic", outputTopic_, std::string("/fast_lio_ns/cloud_registered_body_safe"));
		privateNh_.param("max_range", maxRange_, maxRange_);
		privateNh_.param("min_points", minPoints_, minPoints_);
		if(maxRange_ <= 0.0)
		{
			ROS_WARN("max_range must be positive; using 300 m");
			maxRange_ = 300.0;
		}
		if(minPoints_ < 1)
		{
			minPoints_ = 1;
		}
		publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(outputTopic_, 2);
		subscriber_ = nh_.subscribe(inputTopic_, 2, &PointCloudSafetyFilter::callback, this);
		ROS_INFO("Filtering %s -> %s (finite XYZ, range <= %.1f m, min points=%d)",
				inputTopic_.c_str(), outputTopic_.c_str(), maxRange_, minPoints_);
	}

private:
	void callback(const sensor_msgs::PointCloud2ConstPtr & message)
	{
		uint32_t xOffset = 0;
		uint32_t yOffset = 0;
		uint32_t zOffset = 0;
		uint32_t intensityOffset = 0;
		const bool hasIntensity = fieldOffset(*message, "intensity", intensityOffset);
		if(!fieldOffset(*message, "x", xOffset) || !fieldOffset(*message, "y", yOffset) ||
				!fieldOffset(*message, "z", zOffset) || message->point_step == 0 ||
				message->data.size() < static_cast<size_t>(message->width) * message->height * message->point_step)
		{
			++droppedMessages_;
			ROS_WARN_THROTTLE(2.0, "Dropping malformed FAST-LIO PointCloud2 (dropped=%llu)",
					static_cast<unsigned long long>(droppedMessages_));
			return;
		}

		const uint64_t pointCount = static_cast<uint64_t>(message->width) * message->height;
		const double maxRangeSquared = maxRange_ * maxRange_;
		pcl::PointCloud<pcl::PointXYZI> filtered;
		filtered.points.reserve(static_cast<size_t>(pointCount));
		uint64_t removed = 0;
		for(uint64_t index = 0; index < pointCount; ++index)
		{
			const uint8_t * point = &message->data[static_cast<size_t>(index) * message->point_step];
			const float x = readFloat32(point, xOffset);
			const float y = readFloat32(point, yOffset);
			const float z = readFloat32(point, zOffset);
			const double rangeSquared = static_cast<double>(x)*x + static_cast<double>(y)*y + static_cast<double>(z)*z;
			if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
					!std::isfinite(rangeSquared) || rangeSquared > maxRangeSquared)
			{
				++removed;
				continue;
			}
			pcl::PointXYZI accepted;
			accepted.x = x;
			accepted.y = y;
			accepted.z = z;
			accepted.intensity = hasIntensity ? readFloat32(point, intensityOffset) : 0.0f;
			if(!std::isfinite(accepted.intensity))
			{
				accepted.intensity = 0.0f;
			}
			filtered.points.push_back(accepted);
		}

		removedPoints_ += removed;
		if(filtered.points.size() < static_cast<size_t>(minPoints_))
		{
			++droppedMessages_;
			ROS_WARN_THROTTLE(2.0, "Dropping FAST-LIO cloud with %zu valid points (removed=%llu, dropped=%llu)",
					filtered.points.size(), static_cast<unsigned long long>(removed),
					static_cast<unsigned long long>(droppedMessages_));
			return;
		}
		filtered.width = static_cast<uint32_t>(filtered.points.size());
		filtered.height = 1;
		filtered.is_dense = true;
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(filtered, output);
		output.header = message->header;
		publisher_.publish(output);
		if(removed > 0)
		{
			ROS_WARN_THROTTLE(1.0, "Removed %llu unsafe FAST-LIO points from one cloud (total=%llu)",
					static_cast<unsigned long long>(removed), static_cast<unsigned long long>(removedPoints_));
		}
	}

	ros::NodeHandle nh_;
	ros::NodeHandle privateNh_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	std::string inputTopic_;
	std::string outputTopic_;
	double maxRange_;
	int minPoints_;
	uint64_t removedPoints_;
	uint64_t droppedMessages_;
};

} // namespace

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "pointcloud_safety_filter");
	PointCloudSafetyFilter filter;
	ros::spin();
	return 0;
}
