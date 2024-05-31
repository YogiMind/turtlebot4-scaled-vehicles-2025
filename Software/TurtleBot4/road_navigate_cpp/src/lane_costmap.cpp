#include "road_navigate_cpp/lane_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <cmath> 
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/types.hpp>
#include <deque>
#include <algorithm>
#include <numeric>
#include <opencv2/video/tracking.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// The goal of this plugin is to detect the right road lane and mark it as a low-cost area in the costmap.


namespace road_navigate_cpp
{

// Corrected constructor initialization order
LaneCostmap::LaneCostmap()
: logger_(rclcpp::get_logger("LaneCostmap")),
  robot_yaw_(0.0),
  last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()),
  need_recalculation_(false),
  low_cost_width_(10)
  // preferred_cost_(static_cast<unsigned char>(nav2_costmap_2d::FREE_SPACE)),  // Assuming this should also be initialized here
  // lane_data_(cv::Mat())  // Initialize an empty cv::Mat; proper sizing will be done later
{
}




void LaneCostmap::onInitialize()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock node_");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    // Setting up QoS profile for the publisher with Transient Local durability
    rclcpp::QoS qos_settings(10); // You can adjust the depth as needed
    qos_settings.transient_local(); // Set durability to Transient Local
    qos_settings.reliable(); // Set reliability to Reliable (optional, but often paired with Transient Local)

    lanesCV_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/lanesCV", qos_settings);

    occupancy_grid_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/myRoadPositionCentered", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            this->detectRoadLineCV(msg);
        });

    declareParameter("low_cost_width", rclcpp::ParameterValue(10));  // Default width
    declareParameter("preferred_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::FREE_SPACE)));
    node->get_parameter(name_ + ".low_cost_width", low_cost_width_);
    // node->get_parameter(name_ + ".preferred_cost", preferred_cost_);

    need_recalculation_ = false;
    current_ = true;
}




std::string LaneCostmap::classify_contour(const std::vector<cv::Point>& contour, cv::Point line_pos, cv::Point line_dir) {
    cv::Moments M = cv::moments(contour);
    if (M.m00 != 0) {
        int cX = static_cast<int>(M.m10 / M.m00);
        int cY = static_cast<int>(M.m01 / M.m00);

        cv::Point centroid_vector = cv::Point(cX - line_pos.x, cY - line_pos.y);
        int cross_product = line_dir.x * centroid_vector.y - line_dir.y * centroid_vector.x;

        // Use a small epsilon to avoid issues with numerical precision
        int epsilon = 10; 
        if (cross_product > epsilon) {
            return "left";
        } else if (cross_product < -epsilon) {
            return "right";
        }
    }
    return "";
}




void LaneCostmap::detectRoadLineCV(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {

    const int width = msg->info.width;
    const int height = msg->info.height;
    cv::Mat occupancyGrid(height, width, CV_8UC1);

    for (int i = 0; i < width * height; ++i) {
        auto data = msg->data[i];
        occupancyGrid.data[i] = (data == -1) ? 0 : static_cast<unsigned char>((data / 100.0) * 255);
    }

    cv::GaussianBlur(occupancyGrid, occupancyGrid, cv::Size(3, 3), 0.5, 0.5);
    cv::Mat thresholded;
    cv::threshold(occupancyGrid, thresholded, 200, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point imgCenter(width / 2, height / 2);

    std::vector<cv::Point> rightMostContour;
    int rightMostX = -1;

    // Convert robot_yaw to a unit vector pointing to the robot's direction
    cv::Point2f robot_dir_vector = cv::Point2f(std::cos(robot_yaw_) + 1, std::sin(robot_yaw_) + 1);
    // Normalize the robot direction vector for consistent use in cross product
    float norm = std::sqrt(robot_dir_vector.x * robot_dir_vector.x + robot_dir_vector.y * robot_dir_vector.y);
    robot_dir_vector.x /= norm;
    robot_dir_vector.y /= norm;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 70.0 || area > 220.0) continue;  // Ignore contours with area less than 30 or greater than 300


        std::string classification = classify_contour(contour, imgCenter, robot_dir_vector);
        if (classification == "right") {
            int cx = cv::boundingRect(contour).x + cv::boundingRect(contour).width;  // x position of contour's right edge
            if (cx > rightMostX) {
                rightMostX = cx;
                rightMostContour = contour;
            }
        }

    }

    if (!rightMostContour.empty()) {

        cv::Mat contourOutput = cv::Mat::zeros(height, width, CV_8UC1);
        cv::drawContours(contourOutput, std::vector<std::vector<cv::Point>>{rightMostContour}, -1, cv::Scalar(255), 2);

        nav_msgs::msg::OccupancyGrid lanesCV_msg;
        lanesCV_msg.header = msg->header;
        lanesCV_msg.info = msg->info;
        lanesCV_msg.data.resize(width * height);
        for (int i = 0; i < width * height; ++i) {
            lanesCV_msg.data[i] = (contourOutput.data[i] == 255) ? 100 : 0;
        }
        lanesCV_publisher_->publish(lanesCV_msg);
    }
}






// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
LaneCostmap::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;


  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
LaneCostmap::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "LaneCostmap::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// DEFAULT IMPLEMENTATION
// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
LaneCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Simply computing one-by-one cost per each cell
  int gradient_index;
  for (int j = min_j; j < max_j; j++) {
    // Reset gradient_index each time when reaching the end of re-calculated window
    // by OY axis.
    gradient_index = 0;
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);
      // setting the gradient cost
      unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
      if (gradient_index <= GRADIENT_SIZE) {
        gradient_index++;
      } else {
        gradient_index = 0;
      }
      master_array[index] = cost;
    }
  }
}



// Start of implementation, needs work done. Replace with current default implementation to continue working on it.

// void LaneCostmap::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
// {
//     if (!enabled_) {
//         return;
//     }

//     unsigned char * master_array = master_grid.getCharMap();
//     unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

//     // Ensure the update window is within the map boundaries
//     min_i = std::max(0, min_i);
//     min_j = std::max(0, min_j);
//     max_i = std::min(static_cast<int>(size_x), max_i);
//     max_j = std::min(static_cast<int>(size_y), max_j);

//     // Iterate through the costmap update window
//     for (int j = min_j; j < max_j; j++) {
//         for (int i = min_i; i < max_i; i++) {
//             int index = master_grid.getIndex(i, j);
//             // If this cell is part of the right lane, set it to preferred_cost_
//             if (lane_data_.at<unsigned char>(j, i) == 1) {
//                 master_array[index] = preferred_cost_;
//             } else {
//                 // Optionally, you could set other cells to a higher cost to discourage the robot from leaving the lane
//                 master_array[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
//             }
//         }
//     }
// }




}  



// This is the macro allowing a nav2_gradient_costmap_plugin::LaneCostmap class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(road_navigate_cpp::LaneCostmap, nav2_costmap_2d::Layer)