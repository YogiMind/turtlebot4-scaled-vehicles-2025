// #include "road_navigate_cpp/gradient_layer.hpp"

// #include "nav2_costmap_2d/costmap_math.hpp"
// #include "nav2_costmap_2d/footprint.hpp"
// #include "rclcpp/parameter_events_filter.hpp"
// #include <cmath> 
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include "rclcpp/rclcpp.hpp"
// #include <opencv2/core/types.hpp>
// #include <deque>
// #include <algorithm>
// #include <numeric>
// #include <opencv2/video/tracking.hpp>

// using nav2_costmap_2d::LETHAL_OBSTACLE;
// using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
// using nav2_costmap_2d::NO_INFORMATION;

// // This i



// namespace road_navigate_cpp
// {

// // Corrected constructor initialization order
// GradientLayer::GradientLayer()
// : logger_(rclcpp::get_logger("GradientLayer")),
//   robot_yaw_(0.0),
//   last_min_x_(-std::numeric_limits<float>::max()),
//   last_min_y_(-std::numeric_limits<float>::max()),
//   last_max_x_(std::numeric_limits<float>::max()),
//   last_max_y_(std::numeric_limits<float>::max()),
//   need_recalculation_(false),
//   low_cost_width_(10),
//   preferred_cost_(static_cast<unsigned char>(nav2_costmap_2d::FREE_SPACE)),  // Assuming this should also be initialized here
//   lane_data_(cv::Mat())  // Initialize an empty cv::Mat; proper sizing will be done later
// {
// }




// void GradientLayer::onInitialize()
// {
//     auto node = node_.lock();
//     if (!node) {
//         throw std::runtime_error("Failed to lock node_");
//     }

//     declareParameter("enabled", rclcpp::ParameterValue(true));
//     node->get_parameter(name_ + "." + "enabled", enabled_);

//     // Setting up QoS profile for the publisher with Transient Local durability
//     rclcpp::QoS qos_settings(10); // You can adjust the depth as needed
//     qos_settings.transient_local(); // Set durability to Transient Local
//     qos_settings.reliable(); // Set reliability to Reliable (optional, but often paired with Transient Local)

//     lanesCV_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/lanesCV", qos_settings);

//     occupancy_grid_subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/myRoadPositionCentered", 10,
//         [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
//             this->detectRoadLineCV(msg);
//         });

//     declareParameter("low_cost_width", rclcpp::ParameterValue(10));  // Default width
//     declareParameter("preferred_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::FREE_SPACE)));
//     node->get_parameter(name_ + ".low_cost_width", low_cost_width_);
//     node->get_parameter(name_ + ".preferred_cost", preferred_cost_);

//     need_recalculation_ = false;
//     current_ = true;
// }




// std::string GradientLayer::classify_contour(const std::vector<cv::Point>& contour, cv::Point line_pos, cv::Point line_dir) {
//     cv::Moments M = cv::moments(contour);
//     if (M.m00 != 0) {
//         int cX = static_cast<int>(M.m10 / M.m00);
//         int cY = static_cast<int>(M.m01 / M.m00);

//         cv::Point centroid_vector = cv::Point(cX - line_pos.x, cY - line_pos.y);
//         int cross_product = line_dir.x * centroid_vector.y - line_dir.y * centroid_vector.x;

//         // Use a small epsilon to avoid issues with numerical precision
//         int epsilon = 10; 
//         if (cross_product > epsilon) {
//             return "left";
//         } else if (cross_product < -epsilon) {
//             return "right";
//         }
//     }
//     return "";
// }



// void GradientLayer::detectRoadLineCV(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
//     const int width = msg->info.width;
//     const int height = msg->info.height;
//     cv::Mat occupancyGrid(height, width, CV_8UC1);

//     // Resize lane_data_ to match the occupancy grid dimensions
//     lane_data_ = cv::Mat::zeros(height, width, CV_8UC1);

//     // Initialize the occupancy grid from the incoming data
//     for (int i = 0; i < width * height; ++i) {
//         auto data = msg->data[i];
//         occupancyGrid.data[i] = (data == -1) ? 0 : static_cast<unsigned char>((data / 100.0) * 255);
//     }

//     // Apply Gaussian blur and threshold
//     cv::GaussianBlur(occupancyGrid, occupancyGrid, cv::Size(3, 3), 0.5, 0.5);
//     cv::Mat thresholded;
//     cv::threshold(occupancyGrid, thresholded, 200, 255, cv::THRESH_BINARY);

//     // Find and classify contours
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//     cv::Point robotPosition(static_cast<int>(robot_x_), static_cast<int>(robot_y_));
//     std::vector<std::vector<cv::Point>> rightContours;
//     std::vector<cv::Point> closestRightContour;
//     double minDistance = std::numeric_limits<double>::max();

//     for (const auto& contour : contours) {
//         if (cv::contourArea(contour) > 30.0) {
//             std::string classification = classify_contour(contour, robotPosition, cv::Point2f(std::cos(robot_yaw_), std::sin(robot_yaw_)));
//             if (classification == "right") {
//                 rightContours.push_back(contour);
//                 double distance = cv::pointPolygonTest(contour, robotPosition, true);
//                 if (distance < minDistance) {
//                     minDistance = distance;
//                     closestRightContour = contour;
//                 }
//             }
//         }
//     }

//     // Determine the necessary offset by observation or calculation
//     int offset_x = -40.0; // Replace with actual offset value
//     int offset_y = 0.0; // Replace with actual offset value

//     // Draw the closest right contour and project the right lane
//     cv::Mat contourOutput = cv::Mat::zeros(height, width, CV_8UC1);
//     if (!closestRightContour.empty()) {
//         cv::drawContours(contourOutput, std::vector<std::vector<cv::Point>>{closestRightContour}, -1, cv::Scalar(255), 2);
//         int lane_width = 15;

//         for (const auto& point : closestRightContour) {
//             for (int offset = 1; offset <= lane_width; ++offset) {
//                 // Adjust the lane position based on determined offsets
//                 cv::Point lane_point(point.x - offset + offset_x, point.y + offset_y); 
//                 if (lane_point.x >= 0 && lane_point.x < width && lane_point.y >= 0 && lane_point.y < height) {
//                     cv::circle(contourOutput, lane_point, 1, cv::Scalar(255), -1);
//                     lane_data_.at<unsigned char>(lane_point.y, lane_point.x) = 1;
//                 }
//             }
//         }
//     }

//     // Publish the modified occupancy grid as a new message
//     nav_msgs::msg::OccupancyGrid lanesCV_msg;
//     lanesCV_msg.header = msg->header;
//     lanesCV_msg.info = msg->info;
//     lanesCV_msg.data.resize(width * height);
//     for (int i = 0; i < width * height; ++i) {
//         lanesCV_msg.data[i] = (contourOutput.data[i] == 255) ? 100 : 0;
//     }
//     lanesCV_publisher_->publish(lanesCV_msg);
// }






// // The method is called to ask the plugin: which area of costmap it needs to update.
// // Inside this method window bounds are re-calculated if need_recalculation_ is true
// // and updated independently on its value.
// void
// GradientLayer::updateBounds(
//   double robot_x, double robot_y, double robot_yaw, double * min_x,
//   double * min_y, double * max_x, double * max_y)
// {
//     robot_x_ = robot_x;
//     robot_y_ = robot_y;
//     robot_yaw_ = robot_yaw;


//   if (need_recalculation_) {
//     last_min_x_ = *min_x;
//     last_min_y_ = *min_y;
//     last_max_x_ = *max_x;
//     last_max_y_ = *max_y;
//     // For some reason when I make these -<double>::max() it does not
//     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
//     // -<float>::max() instead.
//     *min_x = -std::numeric_limits<float>::max();
//     *min_y = -std::numeric_limits<float>::max();
//     *max_x = std::numeric_limits<float>::max();
//     *max_y = std::numeric_limits<float>::max();
//     need_recalculation_ = false;
//   } else {
//     double tmp_min_x = last_min_x_;
//     double tmp_min_y = last_min_y_;
//     double tmp_max_x = last_max_x_;
//     double tmp_max_y = last_max_y_;
//     last_min_x_ = *min_x;
//     last_min_y_ = *min_y;
//     last_max_x_ = *max_x;
//     last_max_y_ = *max_y;
//     *min_x = std::min(tmp_min_x, *min_x);
//     *min_y = std::min(tmp_min_y, *min_y);
//     *max_x = std::max(tmp_max_x, *max_x);
//     *max_y = std::max(tmp_max_y, *max_y);
//   }
// }

// // The method is called when footprint was changed.
// // Here it just resets need_recalculation_ variable.
// void
// GradientLayer::onFootprintChanged()
// {
//   need_recalculation_ = true;

//   RCLCPP_DEBUG(rclcpp::get_logger(
//       "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
//     layered_costmap_->getFootprint().size());
// }

// // Assume lane_data_ is a cv::Mat where the lane positions are marked as 1 (lane) and 0 (no lane).
// // This data needs to be updated whenever the lane is detected and transformed appropriately to match
// // the costmap's resolution and coordinate system.

// void GradientLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
//   int max_i, int max_j)
// {
//   if (!enabled_) {
//     return;
//   }

//   unsigned char * master_array = master_grid.getCharMap();
//   unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

//   // Ensure update bounds are within the map dimensions
//   min_i = std::max(0, min_i);
//   min_j = std::max(0, min_j);
//   max_i = std::min(static_cast<int>(size_x), max_i);
//   max_j = std::min(static_cast<int>(size_y), max_j);

//   for (int j = min_j; j < max_j; j++) {
//     for (int i = min_i; i < max_i; i++) {
//       int index = master_grid.getIndex(i, j);

//       // Access the lane_data_ using proper checks and typecasting
//         if (j < lane_data_.rows && i < lane_data_.cols && lane_data_.at<unsigned char>(j, i) == 1) {
//         // Set a low cost for lane cells
//         master_array[index] = preferred_cost_;
//       } else {
//         // Apply normal cost calculations for other cells
//         unsigned char cost = calculateNormalCost(i, j);
//         master_array[index] = cost;
//       }
//     }
//   }
// }


// unsigned char GradientLayer::calculateNormalCost(int i, int j) {
//   // Implement the normal cost calculation logic here
//   // Example: Gradient cost calculation or any other logic used previously
//   return static_cast<unsigned char>((LETHAL_OBSTACLE - i % GRADIENT_FACTOR) % 255);
// }




// }  



// // This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// // to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// // Usually places in the end of cpp-file where the loadable class written.
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(road_navigate_cpp::GradientLayer, nav2_costmap_2d::Layer)