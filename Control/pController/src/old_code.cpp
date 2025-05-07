// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double ATTRACTIVE_GAIN = 1.2;
//     const double REPULSIVE_GAIN = 0.5;
//     const double INFLUENCE_RADIUS = 1.0;
//     const double GOAL_STEP = 0.5;

//     // --- 1. Attractive Force (towards original goal) ---
//     double fx = ATTRACTIVE_GAIN * (next_goal.pose.position.x - robot_pos.x);
//     double fy = ATTRACTIVE_GAIN * (next_goal.pose.position.y - robot_pos.y);

//     // --- 2. Repulsive Forces (away from obstacles) ---
//     for (const auto &obs : obstacles)
//     {
//         double dx = robot_pos.x - obs.x;
//         double dy = robot_pos.y - obs.y;
//         double dist = std::hypot(dx, dy);

//         if (dist < INFLUENCE_RADIUS && dist > 0.05)
//         {
//             double repulsion = REPULSIVE_GAIN * (1.0 / dist - 1.0 / INFLUENCE_RADIUS) / (dist * dist);
//             fx += repulsion * (dx / dist);
//             fy += repulsion * (dy / dist);
//         }
//     }

//     // --- 3. Compute new position ---
//     double magnitude = std::hypot(fx, fy);
//     if (magnitude == 0)
//     {
//         return original_goal;
//     }

//     geometry_msgs::msg::PoseStamped new_goal;
//     new_goal.pose.position.x = robot_pos.x + GOAL_STEP * (fx / magnitude);
//     new_goal.pose.position.y = robot_pos.y + GOAL_STEP * (fy / magnitude);
//     new_goal.pose.position.z = 0;

//     // --- 4. Set orientation to face next_goal ---
//     double dx_orient = next_goal.pose.position.x - new_goal.pose.position.x;
//     double dy_orient = next_goal.pose.position.y - new_goal.pose.position.y;
//     double yaw = std::atan2(dy_orient, dx_orient);

//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     new_goal.pose.orientation = tf2::toMsg(q);

//     // RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"),
//     //             "Suggesting new goal: (%.2f, %.2f) facing toward (%.2f, %.2f)",
//     //             new_goal.pose.position.x, new_goal.pose.position.y,
//     //             next_goal.pose.position.x, next_goal.pose.position.y);

//     return new_goal;
// }


// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double ATTRACTIVE_GAIN = 1.0;
//     const double REPULSIVE_GAIN = 0.4;
//     const double INFLUENCE_RADIUS = 0.8;
//     const double GOAL_STEP = 0.2;
//     const double MAX_LATERAL_DEVIATION = 0.6;

//     // --- 1. Attractive Force towards path direction (robot → next_goal) ---
//     double fx = ATTRACTIVE_GAIN * (next_goal.pose.position.x - robot_pos.x);
//     double fy = ATTRACTIVE_GAIN * (next_goal.pose.position.y - robot_pos.y);

//     // --- 2. Repulsive Forces from nearby obstacles ---
//     for (const auto &obs : obstacles)
//     {
//         double dx = robot_pos.x - obs.x;
//         double dy = robot_pos.y - obs.y;
//         double dist = std::hypot(dx, dy);

//         if (dist < INFLUENCE_RADIUS && dist > 0.05)
//         {
//             double repulsion = REPULSIVE_GAIN * (1.0 / dist - 1.0 / INFLUENCE_RADIUS) / (dist * dist);
//             fx += repulsion * (dx / dist);
//             fy += repulsion * (dy / dist);
//         }
//     }

//     // --- 3. Normalize and step forward ---
//     double magnitude = std::hypot(fx, fy);
//     if (magnitude == 0)
//         return original_goal;

//     // Base new goal on direction vector
//     double new_x = robot_pos.x + GOAL_STEP * (fx / magnitude);
//     double new_y = robot_pos.y + GOAL_STEP * (fy / magnitude);

//     // --- 4. Clamp lateral deviation from path line ---
//     // Line: robot_pos → next_goal
//     double dx_path = next_goal.pose.position.x - robot_pos.x;
//     double dy_path = next_goal.pose.position.y - robot_pos.y;
//     double path_length = std::hypot(dx_path, dy_path);

//     if (path_length > 1e-3)
//     {
//         double nx = dx_path / path_length;
//         double ny = dy_path / path_length;

//         // Vector from robot to new proposed point
//         double dx_new = new_x - robot_pos.x;
//         double dy_new = new_y - robot_pos.y;

//         // Project onto path direction
//         double proj = dx_new * nx + dy_new * ny;

//         // Orthogonal deviation
//         double ortho_x = dx_new - proj * nx;
//         double ortho_y = dy_new - proj * ny;

//         double ortho_dist = std::hypot(ortho_x, ortho_y);

//         if (ortho_dist > MAX_LATERAL_DEVIATION)
//         {
//             // Clamp orthogonal deviation
//             ortho_x *= MAX_LATERAL_DEVIATION / ortho_dist;
//             ortho_y *= MAX_LATERAL_DEVIATION / ortho_dist;

//             new_x = robot_pos.x + proj * nx + ortho_x;
//             new_y = robot_pos.y + proj * ny + ortho_y;
//         }
//     }

//     // --- 5. Safety check: avoid suggesting a point inside an obstacle ---
//     geometry_msgs::msg::Point temp_point;
//     temp_point.x = new_x;
//     temp_point.y = new_y;
//     temp_point.z = 0;

//     if (isGoalInsideObstacle(odo_, temp_point))
//     {
//         RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), "Suggested goal inside obstacle, fallback to current.");
//         return original_goal;
//     }

//     geometry_msgs::msg::PoseStamped new_goal;
//     new_goal.pose.position = temp_point;

//     // --- 6. Orientation towards next goal ---
//     double yaw = std::atan2(next_goal.pose.position.y - new_y, next_goal.pose.position.x - new_x);
//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     new_goal.pose.orientation = tf2::toMsg(q);

//     return new_goal;
// }

// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal,
//     double attraction,
//     double repulsion,
//     double goal_step)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double Q_attraction = attraction;//100;
//     const double Q_repulsion = repulsion;//15;
//     const double MIN_DISTANCE = 0.1;
//     const double MAX_DISTANCE = 2.0;
//     const double GOAL_STEP = goal_step;//0.5;

//     // --- Attraction Vector ---
//     double dx = original_goal.pose.position.x - robot_pos.x;
//     double dy = original_goal.pose.position.y - robot_pos.y;
//     double dist = std::hypot(dx, dy);

//     // const double MIN_DIST = 0.05;
//     // if (dist < MIN_DIST) dist = MIN_DIST;

//     double fx_att = 0.0;
//     double fy_att = 0.0;
//     if (dist > 1e-3) {
//         double F_att = Q_attraction / (4 * M_PI * dist * dist);
//         fx_att = F_att * dx / dist;
//         fy_att = F_att * dy / dist;
//     }

//     // --- Repulsion Vector ---
//     double fx_rep = 0.0;
//     double fy_rep = 0.0;
//     for (const auto &obs : obstacles) {
//         double dx_o = robot_pos.x - obs.x;
//         double dy_o = robot_pos.y - obs.y;
//         double d_o = std::hypot(dx_o, dy_o);
//         // if (d_o < MIN_DISTANCE || d_o > MAX_DISTANCE) continue;
//         if (d_o > MAX_DISTANCE) continue;

//         // const double MIN_DIST = 0.05;
//         // if (d_o < MIN_DIST) d_o = MIN_DIST;

//         double F_rep = Q_repulsion / (4 * M_PI * d_o * d_o);
//         fx_rep += -F_rep * dx_o / d_o;
//         fy_rep += -F_rep * dy_o / d_o;
//     }

//     // --- Final vector ---
//     double fx = fx_att + fx_rep;
//     double fy = fy_att + fy_rep;
//     double norm = std::hypot(fx, fy);

//     geometry_msgs::msg::PoseStamped suggested;
//     suggested.header = original_goal.header;
//     suggested.pose.position.x = robot_pos.x + (norm > 1e-5 ? GOAL_STEP * fx / norm : 0.0);
//     suggested.pose.position.y = robot_pos.y + (norm > 1e-5 ? GOAL_STEP * fy / norm : 0.0);
//     suggested.pose.position.z = 0.0;
//     suggested.pose.orientation.w = 1.0;

//     RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), "Suggesting new goal");

//     return suggested;
// }



// bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
// {
//     std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

//     // Get robot's orientation
//     tf2::Quaternion q(
//         odom.pose.pose.orientation.x,
//         odom.pose.pose.orientation.y,
//         odom.pose.pose.orientation.z,
//         odom.pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);  // yaw = robot’s heading

//     const double FRONT_ANGLE_CONE = M_PI; // 90 degrees total (±45°)
//     const double COLLISION_DISTANCE = 0.35;    // danger zone

//     for (const auto &point : points)
//     {
//         double dx = point.x - odom.pose.pose.position.x;
//         double dy = point.y - odom.pose.pose.position.y;
//         double distance = std::hypot(dx, dy);

//         if (distance > COLLISION_DISTANCE)
//             continue;

//         double angle_to_point = std::atan2(dy, dx);
//         double angle_diff = angle_to_point - yaw;

//         // Normalize to [-π, π]
//         while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//         while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

//         // Check if within frontal cone
//         if (std::fabs(angle_diff) < FRONT_ANGLE_CONE / 2)
//         {
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), 
//                         "collision avoidance kicking in, Angle: %.1f°, Distance: %.2f m",
//                         angle_diff * 180.0 / M_PI, distance);
//             return true;
//         }
//     }

//     return false;
// }


// bool ObstacleAvoidance::isGoalObstructed(const nav_msgs::msg::Odometry &odom, const geometry_msgs::msg::Point &goal)
// {
//     std::vector<geometry_msgs::msg::Point> points = getLaserPoints();

//     const double COLLISION_DISTANCE = 0.325;    // danger zone

//     for (const auto &point : points)
//     {
//         double dx = point.x - odom.pose.pose.position.x;
//         double dy = point.y - odom.pose.pose.position.y;
//         double distance = std::hypot(dx, dy);

//         if (distance < COLLISION_DISTANCE){
//             RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), "collision avoidance kicking in");
//             return true;
//         }
//     }

//     return false;
// }



//////////////////////////////////////////////////

////with extra repulsion
// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal,
//     double attraction,
//     double repulsion,
//     double goal_step)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double Q_attraction = attraction;
//     const double Q_repulsion = repulsion;
//     const double GOAL_STEP = goal_step;

//     const double INFLUENCE_RADIUS = 0.325;  // distance within which obstacles repel
//     const double SAFE_DISTANCE = 0.35;     // min allowed distance from obstacles
//     const double ADJUST_STEP = 0.05;      // how much to nudge goal per adjustment
//     const int MAX_ITERATIONS = 20;        // avoid infinite loops

//     // --- Attraction Vector (towards original goal) ---
//     double dx = original_goal.pose.position.x - robot_pos.x;
//     double dy = original_goal.pose.position.y - robot_pos.y;
//     double dist = std::hypot(dx, dy);

//     double fx_att = 0.0, fy_att = 0.0;
//     if (dist > 1e-3) {
//         double F_att = Q_attraction / (4 * M_PI * dist * dist);
//         fx_att = F_att * dx / dist;
//         fy_att = F_att * dy / dist;
//     }

//     // --- Repulsion Vector (away from nearby obstacles) ---
//     double fx_rep = 0.0, fy_rep = 0.0;
//     for (const auto &obs : obstacles) {
//         double dx_o = robot_pos.x - obs.x;
//         double dy_o = robot_pos.y - obs.y;
//         double d_o = std::hypot(dx_o, dy_o);

//         // if (d_o < INFLUENCE_RADIUS && d_o > 1e-3) {
//         //     double scale = (1.0 / d_o - 1.0 / INFLUENCE_RADIUS);
//         //     double F_rep = Q_repulsion * scale / (d_o * d_o);
//         //     fx_rep += F_rep * dx_o / d_o;
//         //     fy_rep += F_rep * dy_o / d_o;
//         // }
//     }

//     // --- Resultant Force Vector ---
//     double fx = fx_att + fx_rep;
//     double fy = fy_att + fy_rep;
//     double norm = std::hypot(fx, fy);

//     geometry_msgs::msg::PoseStamped suggested;
//     suggested.header = original_goal.header;
//     suggested.pose.position.x = robot_pos.x + (norm > 1e-5 ? GOAL_STEP * fx / norm : 0.0);
//     suggested.pose.position.y = robot_pos.y + (norm > 1e-5 ? GOAL_STEP * fy / norm : 0.0);
//     suggested.pose.position.z = 0.0;
//     suggested.pose.orientation.w = 1.0;

//     // --- Safety Check: Move away from obstacles if too close ---
//     // bool safe = false;
//     // int count = 0;

//     // while (!safe && count++ < MAX_ITERATIONS) {
//     //     safe = true;
//     //     for (const auto &point : obstacles) {
//     //         double dx = suggested.pose.position.x - point.x;
//     //         double dy = suggested.pose.position.y - point.y;
//     //         double dist_to_obs = std::hypot(dx, dy);

//     //         if (dist_to_obs < SAFE_DISTANCE && dist_to_obs > 1e-3) {
//     //             // Nudge away from obstacle
//     //             double unit_dx = dx / dist_to_obs;
//     //             double unit_dy = dy / dist_to_obs;
//     //             suggested.pose.position.x += ADJUST_STEP * unit_dx;
//     //             suggested.pose.position.y += ADJUST_STEP * unit_dy;
//     //             safe = false;
//     //             break;  // recheck all obstacles after each nudge
//     //         }
//     //     }
//     // }

//     RCLCPP_WARN(rclcpp::get_logger("ObstacleAvoidance"), 
//                 "Suggesting new goal: (%.2f, %.2f)",
//                 suggested.pose.position.x, suggested.pose.position.y);

//     return suggested;
// }

// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoalSafe()
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     geometry_msgs::msg::Point target = getFurthestFrontLaserPoint(robot_pos, getLaserPoints());

//     // Step 1: compute direction vector
//     double dx = target.x - robot_pos.x;
//     double dy = target.y - robot_pos.y;
//     double dist = std::hypot(dx, dy);

//     // Step 2: normalize and apply fixed step
//     const double STEP_SIZE = 0.25;  // limit how far the robot escapes

//     geometry_msgs::msg::Point safe_point;
//     if (dist > 1e-3)
//     {
//         safe_point.x = robot_pos.x + STEP_SIZE * dx / dist;
//         safe_point.y = robot_pos.y + STEP_SIZE * dy / dist;
//     }
//     else
//     {
//         safe_point = robot_pos;  // fallback (no clear direction)
//     }

//     // Step 3: create PoseStamped goal
//     geometry_msgs::msg::PoseStamped safe_goal;
//     safe_goal.header.frame_id = "odom";
//     safe_goal.header.stamp = this->get_clock()->now();
//     safe_goal.pose.position = safe_point;

//     // Face in direction of motion
//     double yaw = std::atan2(dy, dx);
//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     safe_goal.pose.orientation = tf2::toMsg(q);

//     RCLCPP_INFO(this->get_logger(), "Suggesting minimal escape goal (%.2f, %.2f)", safe_point.x, safe_point.y);
//     return safe_goal;
// }



/////////////////////////////////////////////////////////////////////////////////////////

// geometry_msgs::msg::PoseStamped ObstacleAvoidance::suggestNewGoal(
//     const geometry_msgs::msg::PoseStamped &original_goal,
//     const geometry_msgs::msg::PoseStamped &next_goal,
//     double attraction,
//     double repulsion,
//     double goal_step)
// {
//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;
//     std::vector<geometry_msgs::msg::Point> obstacles = getLaserPoints();

//     const double Q_attraction = attraction;
//     const double Q_repulsion = repulsion;
//     const double GOAL_STEP = goal_step;
//     const double OBSTACLE_THRESHOLD = 0.3;  // for raycast
//     const double ANGLE_ALIGNMENT_THRESHOLD = M_PI / 10.0;  // 18 degrees

//     // --- 1. Compute Attractive Force ---
//     double dx = original_goal.pose.position.x - robot_pos.x;
//     double dy = original_goal.pose.position.y - robot_pos.y;
//     double dist = std::hypot(dx, dy);

//     double fx_att = Q_attraction * dx;
//     double fy_att = Q_attraction * dy;

//     // --- 2. Compute Repulsive Forces ---
//     double fx_rep = 0.0;
//     double fy_rep = 0.0;

//     for (const auto &obs : obstacles)
//     {
//         double ox = obs.x - robot_pos.x;
//         double oy = obs.y - robot_pos.y;
//         double d = std::hypot(ox, oy);
//         if (d < 1e-2 || d > 1.0) continue;

//         double rep_factor = Q_repulsion * (1.0 / d - 1.0) / (d * d);
//         fx_rep -= rep_factor * (ox / d);
//         fy_rep -= rep_factor * (oy / d);
//     }

//     // --- 3. Combine Forces ---
//     double fx = fx_att + fx_rep;
//     double fy = fy_att + fy_rep;

//     // --- 4. Check alignment and path to goal ---
//     bool path_blocked = isPathToGoalObstructed(robot_pos, original_goal.pose.position, 0.2, 0.1);
//     double mag_att = std::hypot(fx_att, fy_att);
//     double mag_rep = std::hypot(fx_rep, fy_rep);
//     double dot = fx_att * fx_rep + fy_att * fy_rep;
//     double angle = std::acos(dot / (mag_att * mag_rep + 1e-5));

//     bool poorly_aligned = (angle < ANGLE_ALIGNMENT_THRESHOLD);
//     bool weak_force = std::hypot(fx, fy) < 1e-3;

//     // --- 5. If bad suggestion, do side-step escape ---
//     if (path_blocked || poorly_aligned || weak_force)
//     {
//         RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "⚠️ Path blocked or attraction poorly aligned. Side-stepping...");

//         double angle = M_PI / 4.0;  // 45 degrees in radians
//         double side_fx = dx * std::cos(angle) - dy * std::sin(angle);
//         double side_fy = dx * std::sin(angle) + dy * std::cos(angle);

//         double norm = std::hypot(side_fx, side_fy);
//         side_fx /= norm;
//         side_fy /= norm;

//         geometry_msgs::msg::PoseStamped sidestep_goal = original_goal;
//         sidestep_goal.pose.position.x = robot_pos.x + side_fx * GOAL_STEP;
//         sidestep_goal.pose.position.y = robot_pos.y + side_fy * GOAL_STEP;
//         return sidestep_goal;
//     }

//     // --- 6. Return new suggested goal ---
//     geometry_msgs::msg::PoseStamped new_goal = original_goal;
//     new_goal.pose.position.x = robot_pos.x + fx / std::hypot(fx, fy) * GOAL_STEP;
//     new_goal.pose.position.y = robot_pos.y + fy / std::hypot(fx, fy) * GOAL_STEP;
//     return new_goal;
// }

// bool ObstacleAvoidance::isPathToGoalObstructed(const geometry_msgs::msg::Point& start,
//                                                const geometry_msgs::msg::Point& goal) const
// {
//     std::vector<geometry_msgs::msg::Point> laser_points = getLaserPoints();
//     const double STEP = 0.05;
//     const double OBSTACLE_THRESHOLD = 0.25;

//     double dx = goal.x - start.x;
//     double dy = goal.y - start.y;
//     double dist = std::hypot(dx, dy);
//     int steps = std::max(1, static_cast<int>(dist / STEP));

//     for (int i = 1; i <= steps; ++i)
//     {
//         double ratio = static_cast<double>(i) / steps;
//         double x = start.x + dx * ratio;
//         double y = start.y + dy * ratio;

//         for (const auto& obs : laser_points)
//         {
//             if (std::hypot(obs.x - x, obs.y - y) < OBSTACLE_THRESHOLD)
//                 return true;
//         }
//     }
//     return false;
// }


// bool ObstacleAvoidance::isPathToGoalObstructed(const geometry_msgs::msg::Point& start,
//                                                const geometry_msgs::msg::Point& goal, double front_dist, double side_dist) const
// {
//     // Define safety distances and angular thresholds
//     const double SAFE_DIST_FRONT = front_dist;//0.325;
//     const double SAFE_DIST_SIDE = side_dist;//0.15;
//     const double FRONT_ANGLE = M_PI / 6.0;   // ±30°
//     const double SIDE_ANGLE = M_PI / 2.0;    // ±90°

//     // Extract robot's orientation (yaw) from odometry
//     tf2::Quaternion q(
//         odo_.pose.pose.orientation.x,
//         odo_.pose.pose.orientation.y,
//         odo_.pose.pose.orientation.z,
//         odo_.pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     geometry_msgs::msg::Point robot_pos = odo_.pose.pose.position;

//     // Initialize blockage flags
//     bool front_blocked = false;
//     bool left_blocked = false;
//     bool right_blocked = false;

//     // Analyze each laser point
//     for (const auto &point : getLaserPoints())
//     {
//         double dx = point.x - robot_pos.x;
//         double dy = point.y - robot_pos.y;
//         double angle = std::atan2(dy, dx) - yaw;
//         double dist = std::hypot(dx, dy);

//         // Normalize angle to [-π, π]
//         while (angle > M_PI) angle -= 2 * M_PI;
//         while (angle < -M_PI) angle += 2 * M_PI;

//         // Check for obstacles in front
//         if (std::fabs(angle) < FRONT_ANGLE && dist < SAFE_DIST_FRONT)
//             front_blocked = true;

//         // Check for obstacles on the left
//         if (angle > FRONT_ANGLE && angle < SIDE_ANGLE && dist < SAFE_DIST_SIDE)
//             left_blocked = true;

//         // Check for obstacles on the right
//         if (angle < -FRONT_ANGLE && angle > -SIDE_ANGLE && dist < SAFE_DIST_SIDE)
//             right_blocked = true;
//     }

//     // Log warnings if obstacles are detected
//     if (front_blocked)
//         RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle ahead!");

//     if (left_blocked || right_blocked)
//         RCLCPP_WARN(this->get_logger(), "⚠️ Obstacle on the %s side!", left_blocked ? "left" : "right");

//     // Return true if any path is obstructed
//     return front_blocked || left_blocked || right_blocked;
// }



// void pController::move() {
//     if (path_.poses.empty()) return;
//     if (path_index_ >= path_.poses.size()) {
//         RCLCPP_INFO(this->get_logger(), "✅ All waypoints reached.");
//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);
//         path_index_ = 0;
//         path_.poses.clear();
//         return;
//     }

//     geometry_msgs::msg::PoseStamped current_goal = path_.poses[path_index_];
//     geometry_msgs::msg::Point robot_pos = getOdometry().pose.pose.position;

//     if (obstacleavoidance_.isGoalInsideObstacle(getOdometry(),path_.poses[path_index_].pose.position)) {
//         RCLCPP_WARN(this->get_logger(), "🚫 Goal %zu is inside obstacle. Deleting...", path_index_ + 1);
//         path_.poses.erase(path_.poses.begin() + path_index_);
//         goals_in_obstacles_++;
//         return;
//     }

//     if (obstacleavoidance_.isPathToGoalObstructed(robot_pos, current_goal.pose.position, 0.5, 0.15)) {
//         RCLCPP_WARN(this->get_logger(), "⚠️ Obstruction detected. suggesting initial goal...");
//         geometry_msgs::msg::PoseStamped next_goal = (path_index_ + 1 < path_.poses.size()) ? path_.poses[path_index_ + 1] : current_goal;
//         geometry_msgs::msg::PoseStamped sidestep = obstacleavoidance_.suggestNewGoal(current_goal, next_goal, 15, 2000, 0.5);
//         path_.poses.insert(path_.poses.begin() + path_index_, sidestep);
//         return;
//     }

//     if (getDistanceError(current_goal) < tolerance) {
//         RCLCPP_INFO(this->get_logger(), "🏁 Reached goal %zu", path_index_ + 1);
//         geometry_msgs::msg::Twist stop_msg;
//         cmd_vel_pub_->publish(stop_msg);
//         path_index_++;
//         return;
//     }

//     double dist = getDistanceError(current_goal);
//     double angle = getAngularError(current_goal);

//     geometry_msgs::msg::Twist cmd_msg;
//     cmd_msg.linear.x = std::min(Kp_linear * dist, 0.08);
//     cmd_msg.angular.z = Kp_angular * angle;

//     static geometry_msgs::msg::Twist prev_cmd;
//     const double ALPHA = 0.7;
//     cmd_msg.linear.x = ALPHA * cmd_msg.linear.x + (1 - ALPHA) * prev_cmd.linear.x;
//     cmd_msg.angular.z = ALPHA * cmd_msg.angular.z + (1 - ALPHA) * prev_cmd.angular.z;
//     prev_cmd = cmd_msg;

//     cmd_vel_pub_->publish(cmd_msg);
// }


// visualization_msgs::msg::Marker pController::produceMarkerTable(geometry_msgs::msg::Point pt)
// {

//     visualization_msgs::msg::Marker marker;

//     // We need to set the frame
//     //  Set the frame ID and time stamp.
//     marker.header.frame_id = "odom";
//     marker.header.stamp = this->get_clock()->now();
//     // We set lifetime (it will dissapear in this many seconds)
//     marker.lifetime = rclcpp::Duration(1000, 0); // zero is forever

//     // Set the namespace and id for this marker.  This serves to create a unique ID
//     // Any marker sent with the same namespace and id will overwrite the old one
//     marker.ns = "goals"; // This is namespace, markers can be in diofferent namespace
//     marker.id = ct_++;   // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

//     // The marker type
//     marker.type = visualization_msgs::msg::Marker::CYLINDER;

//     // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
//     marker.action = visualization_msgs::msg::Marker::ADD;

//     marker.pose.position.x = pt.x;
//     marker.pose.position.y = pt.y;
//     marker.pose.position.z = pt.z;

//     // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;

//     // Set the scale of the marker -- 1m side
//     marker.scale.x = 0.3;
//     marker.scale.y = 0.3;
//     marker.scale.z = 2.0;

//     // Let's send a marker with color (green for reachable, red for now)
//     std_msgs::msg::ColorRGBA color;
//     color.a = 0.5; // a is alpha - transparency 0.5 is 50%;
//     color.r = 250.0 / 255.0;
//     color.g = 0;
//     color.b = 0;

//     marker.color = color;

//     return marker;
// }

// void pController::publishFinalGoalMarkers()
// {
//     std::vector<geometry_msgs::msg::Point> marker_points = {
//         createPoint(0.81, 1.21, 0.0),
//         createPoint(0.9, 2.43, 0.0),
//         createPoint(2.0, 1.8, 0.0),
//         createPoint(2.91, 1.0, 0.0),
//         createPoint(2.91, 2.2, 0.0)

//         // tables 1-5
//     };

//     std::vector<geometry_msgs::msg::Point> robot_positions_at_table = {
//         createPoint(1.22, 1.22, 0.0),
//         createPoint(1.25, 2.18, 0.0),
//         createPoint(1.7, 1.6, 0.0),
//         createPoint(2.66, 1.34, 0.0),
//         createPoint(2.76, 1.8, 0.0)

//         // tables 1-5
//     };

//     visualization_msgs::msg::MarkerArray marker_array;
//     for (const auto &pt : marker_points)
//     {
//         marker_array.markers.push_back(produceMarkerTable(pt));
//     }

//     pub_table_marker->publish(marker_array);
// }

// geometry_msgs::msg::Point pController::createPoint(double x, double y, double z)
// {
//     geometry_msgs::msg::Point pt;
//     pt.x = x;
//     pt.y = y;
//     pt.z = z;
//     return pt;
// }

// double pController::getAngleToGoal(const geometry_msgs::msg::Point &goal)
// {
//     geometry_msgs::msg::Point robot_pos = getOdometry().pose.pose.position;
//     double dx = goal.x - robot_pos.x;
//     double dy = goal.y - robot_pos.y;
//     double goal_angle = std::atan2(dy, dx);

//     tf2::Quaternion q(
//         getOdometry().pose.pose.orientation.x,
//         getOdometry().pose.pose.orientation.y,
//         getOdometry().pose.pose.orientation.z,
//         getOdometry().pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     double angle_diff = goal_angle - yaw;
//     while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//     while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
//     return angle_diff;
// }