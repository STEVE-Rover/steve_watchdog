#ifndef STEVE_WATCHDOG_H
#define STEVE_WATCHDOG_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Bool.h>

class TopicMonitor
{
    public:
        TopicMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh);
        void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg);
        void printTopicMonitorInfo();
        void createSubscription();
        void start();
        bool getStatus();
        void setName();
        void setTopicName();
        void minFreq();
        void maxFreq();

        std::string name_;
        std::string topic_name_;
        float min_freq_;
        float max_freq_;

    private:
        void run();

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        int ticks_ = 0;
        bool status_ = false;
        ros::Subscriber sub_;
        std::thread thread_;
};

class SteveWatchdog
{
    public:
        SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh);
        int getNbOfTopics();
        void run();

    private:
        bool createTopicMonitors();

        // ROS variables
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher status_pub_;
        std::vector<std::shared_ptr<TopicMonitor>> topic_list_;
        int nb_of_topics_;
        bool status_;
};


#endif