#ifndef LANE_COSTMAP_HPP_
#define LANE_COSTMAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <opencv2/core.hpp> // Consolidated include
#include <deque>
#include <algorithm>
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace road_navigate_cpp
{

class LaneCostmap : public nav2_costmap_2d::Layer
{
public:
  LaneCostmap();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
  rclcpp::Logger logger_;

  unsigned char calculateNormalCost(int i, int j);
  std::string classify_contour(const std::vector<cv::Point>& contour, cv::Point line_pos, cv::Point line_dir);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  int determineRoadLinePosition();

private:
  double robot_yaw_;
  double robot_x_;
  double robot_y_;
  cv::Mat lane_data_;

  std::vector<cv::Point> currentRightContour;
  std::vector<cv::Point> preferred_path_;
  std::vector<cv::Point> rightMostContour;

  std::deque<std::tuple<float, float, float>> measurementHistory; // History of valid measurements
  float lastMeasuredPosX; // To store last measured X position

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;

  // Subscriber for OccupancyGrid messages
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr lanesCV_publisher_;

  int low_cost_width_;      // Width of the low cost area to the left of the detected road line
  unsigned char preferred_cost_;  // Cost value for the low cost area

  int buffer_zone_ = 15;  // Default buffer zone width, can be adjusted or made dynamic

  void detectRoadLineCV(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
  bool convertRoadlineCVToCostmap(const cv::Point &roadlineCVPoint, unsigned int &mx, unsigned int &my, nav2_costmap_2d::Costmap2D &master_grid);
};

}  // namespace road_navigate_cpp

#endif  // LANE_COSTMAP_HPP_
