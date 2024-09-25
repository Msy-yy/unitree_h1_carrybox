#include <numeric>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // for doTransform function


class QRCodeDetectorNode
{
public:
  QRCodeDetectorNode():tf_listener(tf_buffer)
  {
    // tf2_ros::TransformListener tf_listener(tf_buffer);
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh_, "/camera/color/image_raw", 1);
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh_, "/camera/aligned_depth_to_color/image_raw", 1);
    camera_info_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/camera_info", 10, &QRCodeDetectorNode::cameraInfoCallback, this);
    qr_code_image_pub_ = nh_.advertise<sensor_msgs::Image>("/qr_code_image", 10);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(30), *image_sub_, *depth_sub_);
    sync_->registerCallback(boost::bind(&QRCodeDetectorNode::callback, this, _1, _2));
    qr_code_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/qr_code_pose", 10);
    qr_detector_ = cv::QRCodeDetector();
  }

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg)
  {
    fx_ = camera_info_msg->K[0]; // Focal length fx
    fy_ = camera_info_msg->K[4]; // Focal length fy
    cx_ = camera_info_msg->K[2]; // Principal point cx
    cy_ = camera_info_msg->K[5]; // Principal point cy
  }

  bool transformPose(const geometry_msgs::PoseStamped &input_pose, geometry_msgs::PoseStamped &output_pose, const std::string &target_frame)
  {
    try
    {
      // 获取从input_pose.frame_id到target_frame的转换
      geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(target_frame, input_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));

      // 应用转换
      tf2::doTransform(input_pose, output_pose, transform_stamped);
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Could not transform pose: %s", ex.what());
      return false;
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr &image_msg, const sensor_msgs::Image::ConstPtr &depth_msg)
  {
    cv::Mat color_image = convertImageToCvMat(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat depth_image = convertImageToCvMat(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

    if (color_image.empty() || depth_image.empty())
    {
      ROS_ERROR("Error converting images.");
      return;
    }

    std::vector<cv::Point> qr_points;
    std::string qr_data = detectQRCode(color_image, qr_points);
    if (qr_data.empty())
    {
      ROS_WARN("No QR code detected.");
      return;
    }

    ROS_INFO("QR code detected with %ld corners", qr_points.size());

    drawQRCode(color_image, qr_points);
    publishImage(color_image, image_msg->header);

    std::vector<cv::Point3f> qr_corners_3d = getQRCodeCorners3D(qr_points, depth_image);
    if (qr_corners_3d.size() < 4)
    {
      ROS_WARN("Not enough valid QR code corners with depth information.");
      return;
    }

    cv::Point3f qr_center = calculateQRCodeCenter(qr_corners_3d);
    ROS_INFO("QR code 3D position: [X: %f, Y: %f, Z: %f]", qr_center.x, qr_center.y, qr_center.z);

    geometry_msgs::PoseStamped qr_code_pose_msg;
    qr_code_pose_msg.header.frame_id = depth_msg->header.frame_id;
    qr_code_pose_msg.header.stamp = depth_msg->header.stamp;
    qr_code_pose_msg.pose.position.x = qr_center.x;
    qr_code_pose_msg.pose.position.y = qr_center.y;
    qr_code_pose_msg.pose.position.z = qr_center.z;
    geometry_msgs::PoseStamped qr_code_msg_in_map;
    transformPose(qr_code_pose_msg, qr_code_msg_in_map, "map");

    qr_code_pose_publisher_.publish(qr_code_pose_msg);
  }

  cv::Mat convertImageToCvMat(const sensor_msgs::Image::ConstPtr &image_msg, const std::string &encoding)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
    return cv_ptr->image;
  }

  std::string detectQRCode(cv::Mat &image, std::vector<cv::Point> &points)
  {
    return qr_detector_.detectAndDecode(image, points);
  }

  void drawQRCode(cv::Mat &image, const std::vector<cv::Point> &points)
  {
    for (const auto &point : points)
    {
      cv::circle(image, point, 5, cv::Scalar(0, 255, 0), -1); // Draw corners
    }
    if (points.size() == 4)
    {
      cv::line(image, points[0], points[1], cv::Scalar(0, 0, 255), 2);
      cv::line(image, points[1], points[2], cv::Scalar(0, 0, 255), 2);
      cv::line(image, points[2], points[3], cv::Scalar(0, 0, 255), 2);
      cv::line(image, points[3], points[0], cv::Scalar(0, 0, 255), 2);
    }
  }

  void publishImage(const cv::Mat &image, const std_msgs::Header &header)
  {
    auto image_out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    qr_code_image_pub_.publish(image_out_msg);
    cv::imshow("qr_code_view", image);
    cv::waitKey(10);
  }

  std::vector<cv::Point3f> getQRCodeCorners3D(const std::vector<cv::Point> &points, const cv::Mat &depth_image)
  {
    std::vector<cv::Point3f> qr_corners_3d;
    for (const auto &corner : points)
    {
      int u = corner.x;
      int v = corner.y;

      if (u < 0 || u >= depth_image.cols || v < 0 || v >= depth_image.rows)
      {
        ROS_WARN("QR corner pixel out of bounds: [X: %d, Y: %d]", u, v);
        continue;
      }

      uint16_t depth_value = getAverageDepth(depth_image, corner);
      float z = depth_value / 1000.0f; // Convert to meters
      float x = (u - cx_) * z / fx_;
      float y = (v - cy_) * z / fy_;

      qr_corners_3d.push_back(cv::Point3f(x, y, z));
    }
    return qr_corners_3d;
  }

  uint16_t getAverageDepth(const cv::Mat &depth_image, const cv::Point &corner)
  {
    int windowSize = 5;
    std::vector<uint16_t> depths;
    for (int dy = -windowSize; dy <= windowSize; ++dy)
    {
      for (int dx = -windowSize; dx <= windowSize; ++dx)
      {
        int ny = corner.y + dy;
        int nx = corner.x + dx;
        if (ny >= 0 && ny < depth_image.rows && nx >= 0 && nx < depth_image.cols)
        {
          uint16_t d = depth_image.at<uint16_t>(ny, nx);
          if (d > 0)
          {
            depths.push_back(d);
          }
        }
      }
    }
    if (depths.empty())
      return 0;

    float sum = std::accumulate(depths.begin(), depths.end(), 0.0f);
    return static_cast<uint16_t>(sum / depths.size());
  }

  cv::Point3f calculateQRCodeCenter(const std::vector<cv::Point3f> &corners)
  {
    cv::Point3f center(0, 0, 0);
    for (const auto &corner : corners)
    {
      center += corner;
    }
    return center / static_cast<float>(corners.size());
  }

  // Camera intrinsics
  float fx_, fy_, cx_, cy_;

  ros::NodeHandle nh_;
  cv::QRCodeDetector qr_detector_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher qr_code_image_pub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  ros::Publisher qr_code_pose_publisher_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_code_detector_node");
  QRCodeDetectorNode qr_code_detector_node;
  ros::spin();
  return 0;
}
