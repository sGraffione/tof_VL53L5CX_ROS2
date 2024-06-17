#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>
#include <memory>
#include <fcntl.h>

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
//#include <tf2_geometry_msgs/msg/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "minicar_interfaces/msg/tof_distance_sensor.hpp"

#include "vl53l5cx_api.h"

#define ToRadians M_PI/180.0
#define ToDegree 180.0/M_PI

using namespace std::chrono_literals;

class RangingSensor : public rclcpp::Node {
    public:
        RangingSensor() : Node("vl53l5cx_node"), count_(0) {

            // parameters acquiring
            get_parameter_or("resolution",resolution_,resolution_);
            get_parameter_or("mode",mode_,mode_); // TODO: add control on parameters to avoid wrong params
            

            // check parameters quality
            if(resolution_!=4 && resolution_!=8){
                RCUTILS_LOG_ERROR_NAMED(get_name(), "Wrong resolution parameter (passed parameter is %i). Must be '4' or '8'.\n\r",resolution_);
            }
            if(mode_.compare("XYZ")!=0 && mode_.compare("distance")!=0){
                RCUTILS_LOG_ERROR_NAMED(get_name(), "Wrong mode parameter (passed parameter is %s). Must be 'XYZ' or 'distance'.\n\r",mode_.c_str());
            }


            width_ = resolution_;
            height_ = resolution_;
            // update number of beams depending on the current resolution
            n_beams_ = width_*height_;

            RCLCPP_DEBUG(this->get_logger(),"Sensor starting in %s mode with %ix%i resolution\n",mode_.c_str(),width_,height_);
            // setting up publisher and timer
            if(mode_.compare("XYZ")){
                pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tof_sensor",1);
            }else if(mode_.compare("distance")){
                tof_publisher_ = this->create_publisher<minicar_interfaces::msg::TofDistanceSensor>("tof_sensor",1);
            }
            
            timer_ = this->create_wall_timer(100ms, std::bind(&RangingSensor::publishMessage, this));


            /*********************************/
            /*   Power on sensor and init    */
            /*********************************/

            /* Initialize channel com */
            status_ = vl53l5cx_comms_init(&Dev.platform);
            if(status_)
            {
                RCUTILS_LOG_ERROR_NAMED(get_name(), "VL53L5CX comms init failed.\n\r");
                closeConnection();
            }

            status_ = simpleRangingData(&Dev);
            closeConnection();
        };
        ~RangingSensor(){};

        void closeConnection(){
            vl53l5cx_stop_ranging(&Dev);
            vl53l5cx_comms_close(&Dev.platform);
        }

        void publishMessage(){
            //RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing: '%s'", message_.); TODO: to prepare a debug message
            // Publish
            if(mode_.compare("XYZ")){
                pcl_publisher_->publish(*pc2_msg_);
            }else if(mode_.compare("distance")){
                tof_publisher_->publish(*tof_msg_);
            }
        }

        int simpleRangingData(VL53L5CX_Configuration *p_dev)
        {

            /*********************************/
            /*   VL53L5CX ranging variables  */
            /*********************************/

            uint8_t 				status, isAlive, isReady;
            VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


            /*********************************/
            /*   Power on sensor and init    */
            /*********************************/

            /* (Optional) Check if there is a VL53L5CX sensor connected */
            status = vl53l5cx_is_alive(p_dev, &isAlive);
            if(!isAlive || status)
            {
                RCUTILS_LOG_ERROR_NAMED(get_name(), "VL53L5CX not detected at requested address\n");
                return status;
            }

            /* (Mandatory) Init VL53L5CX sensor */
            status = vl53l5cx_init(p_dev);
            if(status)
            {
                RCUTILS_LOG_ERROR_NAMED(get_name(), "VL53L5CX ULD Loading failed\n");
                return status;
            }

            RCLCPP_INFO(this->get_logger(),"VL53L5CX ULD ready ! (Version : %s)\n", VL53L5CX_API_REVISION);


            /*********************************/
            /*         Ranging loop          */
            /*********************************/

            status = vl53l5cx_start_ranging(p_dev);

            while(rclcpp::ok())
            {
                /* Use polling function to know when a new measurement is ready.
                * Another way can be to wait for HW interrupt raised on PIN A3
                * (GPIO 1) when a new measurement is ready */

        
                isReady = wait_for_dataready(&p_dev->platform);

                if(isReady)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Data ready\n");
                    vl53l5cx_get_ranging_data(p_dev, &Results);
                    createScanningMessage(Results);
                }
            }
            return status;
        }

    private:
        void createScanningMessage(VL53L5CX_ResultsData Results)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Creating new message");

            if(mode_.compare("XYZ")){
                pcl::PointXYZ pt;
                cloud_.clear();

                for (int counter = 0; counter < n_beams_; counter++){
                    int i = counter / width_; // rows
                    int j = counter % height_; // columns
                    
                    float phi_j = (j-1.5)*hStep_;
                    float theta_i = (i-1.5)*vStep_;
                    float distance = Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*counter]/1000.0; // TODO: conversion to meters

                    pt = pcl::PointXYZ();

                    //RCLCPP_DEBUG_STREAM(this->get_logger(), "Zone: %i, distance: %f",counter,distance);

                    pt.x = distance*cos(phi_j)*cos(theta_i);
                    pt.y = distance*cos(phi_j)*sin(theta_i);
                    pt.z = distance*sin(phi_j);

                    // add point to point cloud
                    cloud_.points.push_back(pt);
                }

                // Convert list of points into a cloud
                pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(cloud_, *pc2_msg_);
                // pcl::toROSMsg()
                pc2_msg_->header.frame_id = frame_id_;
                pc2_msg_->header.stamp = now();
                pc2_msg_->height = height_;
                pc2_msg_->width = width_;
            }else if (mode_.compare("distance")){
                std::vector<float> distances;
                for (int counter = 0; counter < n_beams_; counter++){
                    distances.push_back(Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*counter]/1000.0);
                }
                tof_msg_->distance = distances;
                tof_msg_->resolution = resolution_;
            }
        }

        int status_;
        VL53L5CX_Configuration Dev;

        // Sensor configuration
        // Default configuration
        std::string mode_ = "XYZ"; // or "distance"
        int n_beams_ = 16; // 4 x 4 matrix
        int resolution_ = 4;
        int height_ = 4;
        int width_ = 4;
        float hFoV_ = 45*ToRadians;
        float vFoV_ = 45*ToRadians;
        float hStep_ = hFoV_/width_;
        float vStep_ = vFoV_/height_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
        rclcpp::Publisher<minicar_interfaces::msg::TofDistanceSensor>::SharedPtr tof_publisher_;
        
        // Point Cloud variables
        std::string frame_id_ = "laser_detection";
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
        minicar_interfaces::msg::TofDistanceSensor::SharedPtr tof_msg_;

        size_t count_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RangingSensor>());
    if(!rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}