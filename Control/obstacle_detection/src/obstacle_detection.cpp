#include "obstacle_detection.hpp"
    
ObstacleDetection::ObstacleDetection() : Node("obstacle_detection"){

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ObstacleDetection::laserCallback, this, std::placeholders::_1));

    pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_objects",1000);

    sub_odo_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&ObstacleDetection::odomCallback, this, std::placeholders::_1));


}

void ObstacleDetection::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    odo_ = *msg;


}

nav_msgs::msg::Odometry ObstacleDetection::getOdometry(void){
    
    nav_msgs::msg::Odometry pose = odo_;
    return pose;
}

void ObstacleDetection::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{

    scan_ = *msg;

    unsigned int segments = countSegments(getOdometry());

    RCLCPP_INFO_STREAM(this->get_logger(), "Segments detected: " << segments);

    for(auto segment : segmentVector_){
        RCLCPP_INFO_STREAM(this->get_logger(), "Segments first x " << segment.first.x);
        visualization_msgs::msg::Marker marker = produceMarker(segment.first);
        markerArray_.markers.push_back(marker);
        
    }

    pub_marker->publish(markerArray_);
}

geometry_msgs::msg::Point ObstacleDetection::localToGlobal(nav_msgs::msg::Odometry global, geometry_msgs::msg::Point local){
    geometry_msgs::msg::Point pt;

    pt.x = global.pose.pose.position.x + (local.x * cos(tf2::getYaw(global.pose.pose.orientation))) - (local.y * sin(tf2::getYaw(global.pose.pose.orientation)));
    pt.y = global.pose.pose.position.y + (local.x * sin(tf2::getYaw(global.pose.pose.orientation))) + (local.y * cos(tf2::getYaw(global.pose.pose.orientation)));
    pt.z = global.pose.pose.position.z;

    return pt;
}

unsigned int ObstacleDetection::countSegments(nav_msgs::msg::Odometry odom){
    
    unsigned int count = 0;

    geometry_msgs::msg::Point current,previous;
    geometry_msgs::msg::Point start, end;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> segmentVector;

    for(float i = 1; i < scan_.ranges.size(); i++){
        bool ptCount = true;
        bool segStarted = false;
        // unsigned int st = i,en = i;



        if(isinf(scan_.ranges.at(i-1)) || std::isnan(scan_.ranges.at(i-1))){
            //  std::cout<<"skip cause inf or nan" << std::endl;
            //i++;
            continue;
        }

        while(ptCount && (!std::isnan(scan_.ranges.at(i))) && (scan_.ranges.at(i) < scan_.range_max) && i < scan_.ranges.size()){
            // std::cout << "valid" << std::endl;
            if(isinf(scan_.ranges.at(i))){
                ptCount = false;
            }else{
                float previousAngle = scan_.angle_min + scan_.angle_increment * (i-1);

                previous.x = scan_.ranges.at(i-1)*cos(previousAngle);
                previous.y = scan_.ranges.at(i-1)*sin(previousAngle);
                previous.z = 0;

                float currentAngle = scan_.angle_min + scan_.angle_increment * (i);

                current.x = scan_.ranges.at(i)*cos(currentAngle);
                current.y = scan_.ranges.at(i)*sin(currentAngle);
                current.z = 0;

                float dist = hypot((current.x - previous.x), (current.y - previous.y));

                if(dist < 0.15){
                    if(!segStarted){
                        segStarted = true;
                        start = localToGlobal(odom, current);
                    }
                }else{
                    ptCount = false;
                }
                
                if(i < scan_.ranges.size()-1){
                    i++;
                }else{
                    break;
                }
            }

            if(!ptCount){
                if(segStarted){
                    end = localToGlobal(odom, previous);
                    segmentVector.push_back(std::make_pair(start, end));
                }
                count++;
            }


        }
    }

    segmentVector_ = segmentVector;

    return count;


}

visualization_msgs::msg::Marker ObstacleDetection::produceMarker(geometry_msgs::msg::Point pt){

      visualization_msgs::msg::Marker marker;

      //We need to set the frame
      // Set the frame ID and time stamp.
      marker.header.frame_id = "odom";
      marker.header.stamp = this->get_clock()->now();
      //We set lifetime (it will dissapear in this many seconds)
      marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
      marker.id = ct_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

      // The marker type
      marker.type = visualization_msgs::msg::Marker::CYLINDER;

      // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = pt.x;
      marker.pose.position.y = pt.y;
      marker.pose.position.z = pt.z;


      //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;


      // Set the scale of the marker -- 1m side
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 2.0;

      
      std_msgs::msg::ColorRGBA color;
      color.a=0.5;//a is alpha - transparency 0.5 is 50%;
      color.r=250.0/255.0;
      color.g=0;
      color.b=0;

      marker.color = color;

      return marker;

}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}