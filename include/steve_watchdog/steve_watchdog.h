#ifndef STEVE_WATCHDOG_H
#define STEVE_WATCHDOG_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <steve_watchdog/TopicArray.h>

class TopicMonitor
{
    public:
        TopicMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh, std::string name, std::string topic_name, 
                     float min_freq, bool use_average, float rate);
        void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg);
        void printTopicMonitorInfo();
        void start();
        bool getStatus();
        std::string getName();

    private:
        void run();

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        std::string name_;
        std::string topic_name_;
        float min_freq_;
        float min_time_;
        bool use_average_;
        float rate_;
        std::vector<ros::Time> stamps_;
        bool status_ = false;
        ros::Subscriber sub_;
        std::thread thread_;
        std::mutex mu_;
};

class SteveWatchdog
{
    public:
        SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh);
        int getNbOfTopics();
        void run();

    private:
        bool createTopicMonitors();
        void cmdVelCB(const geometry_msgs::Twist::ConstPtr msg);

        // ROS variables
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher status_pub_;
        ros::Publisher info_pub_;
        std::vector<std::shared_ptr<TopicMonitor>> topic_list_;
        int nb_of_topics_;
        bool status_;
        float rate_;
};


#endif