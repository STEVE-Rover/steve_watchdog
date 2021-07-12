#ifndef STEVE_WATCHDOG_H
#define STEVE_WATCHDOG_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <ros/ros.h>
#include <ros/console.h>
#include <topic_tools/shape_shifter.h>

class MonitoredTopic
{
    public:
        MonitoredTopic(ros::NodeHandle nh, ros::NodeHandle private_nh);
        void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg);
        void printTopicInfo();

        // TODO: should these variables be private?
        std::string name_;
        std::string topic_;
        float min_freq_;
        float max_freq_;
        ros::Subscriber sub_;

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Time previous_time_;
};

class SteveWatchdog
{
    public:
        SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh);
        int getNbOfTopics();

    private:
        bool getTopics(std::vector<std::shared_ptr<MonitoredTopic>>& topic_list);

        // ROS variables
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        std::vector<std::shared_ptr<MonitoredTopic>> topic_list_;
        int nb_of_topics_;
};


#endif