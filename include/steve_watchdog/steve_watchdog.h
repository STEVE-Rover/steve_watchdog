#ifndef STEVE_WATCHDOG_H
#define STEVE_WATCHDOG_H

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <topic_tools/shape_shifter.h>

struct TopicInfo
{
    std::string name;
    std::string topic;
    float min_freq;
    float max_freq;
    ros::Subscriber sub;
};

class SteveWatchdog
{
    public:
        SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        bool getTopics(std::vector<TopicInfo>& topic_list);
        void printTopicInfo(TopicInfo topic);
        void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg);

        // ROS variables
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        std::vector<TopicInfo> topic_list_;
};


#endif