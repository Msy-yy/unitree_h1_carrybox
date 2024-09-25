#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudConverterNode
{
public:
  PointCloudConverterNode()
  {
    // Initialize ROS node and subscribers/publishers
    ros::NodeHandle nh("~");
    std::string sub_lidar_topic, pub_lidar_topic;
    nh.param<std::string>("lidar_topic_sub_name", sub_lidar_topic, "/livox/lidar");
    nh.param<std::string>("lidar_topic_pub_name", pub_lidar_topic, "/livox/lidar_pc");
    nh.param<std::string>("frame_id", frame_id_, "livox_frame");
    pointcloud_subscriber_ = nh.subscribe(
        sub_lidar_topic, 1, &PointCloudConverterNode::pointCloudCallback, this);
    custom_msg_publisher_ = nh.advertise<livox_ros_driver::CustomMsg>(
        pub_lidar_topic, 1);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc2_msg);

private:
  ros::Subscriber pointcloud_subscriber_;
  ros::Publisher custom_msg_publisher_;
  std::string frame_id_; // Adjust frame_id if needed
};

void PointCloudConverterNode::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &pc2_msg)
{
  livox_ros_driver::CustomMsg custom_msg;

  // Initialize custom_msg header
  custom_msg.header.frame_id.assign(frame_id_);

  uint64_t timestamp = 0;
  if (!pc2_msg->data.empty())
  {
    timestamp = pc2_msg->header.stamp.toNSec();
  }
  custom_msg.timebase = timestamp;

  // Set the header timestamp
  custom_msg.header.stamp = pc2_msg->header.stamp;

  // Get field offsets
  int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1,
      tag_offset = -1, line_offset = -1, timestamp_offset = -1;
  for (size_t i = 0; i < pc2_msg->fields.size(); ++i)
  {
    if (pc2_msg->fields[i].name == "x")
    {
      x_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "y")
    {
      y_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "z")
    {
      z_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "intensity")
    {
      intensity_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "tag")
    {
      tag_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "line")
    {
      line_offset = pc2_msg->fields[i].offset;
    }
    else if (pc2_msg->fields[i].name == "timestamp")
    {
      timestamp_offset = pc2_msg->fields[i].offset;
    }
  }

  // Fill the points data
  const uint8_t *data_ptr = pc2_msg->data.data();
  size_t point_step = pc2_msg->point_step;
  size_t num_points = pc2_msg->width * pc2_msg->height;
  num_points = pc2_msg->data.size() / point_step;
  // #pragma omp parallel for
  for (size_t i = 0; i < num_points; ++i)
  {
    const uint8_t *data_ptr = pc2_msg->data.data() + i * point_step;
    livox_ros_driver::CustomPoint custom_point;

    if (x_offset != -1)
    {
      custom_point.x = *reinterpret_cast<const float *>(data_ptr + x_offset);
    }
    if (y_offset != -1)
    {
      custom_point.y = *reinterpret_cast<const float *>(data_ptr + y_offset);
    }
    if (z_offset != -1)
    {
      custom_point.z = *reinterpret_cast<const float *>(data_ptr + z_offset);
    }
    if (intensity_offset != -1)
    {
      custom_point.reflectivity =
          *reinterpret_cast<const float *>(data_ptr + intensity_offset);
    }
    if (tag_offset != -1)
    {
      custom_point.tag =
          *reinterpret_cast<const uint8_t *>(data_ptr + tag_offset);
    }
    if (line_offset != -1)
    {
      custom_point.line =
          *reinterpret_cast<const uint8_t *>(data_ptr + line_offset);
    }
    if (timestamp_offset != -1)
    {
      custom_point.offset_time =
          *reinterpret_cast<const double *>(data_ptr + timestamp_offset) - custom_msg.timebase;
    }
    if (abs(custom_point.x) < 1.4 &&
        abs(custom_point.y) < 1.4 &&
        abs(custom_point.z) < 1.4)
      continue;

    // #pragma omp critical
    custom_msg.points.push_back(custom_point);
  }
  custom_msg.point_num = custom_msg.points.size();

  // Publish the custom_msg
  custom_msg_publisher_.publish(custom_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_converter_node");
  PointCloudConverterNode converter_node;
  ros::spin();
  return 0;
}
