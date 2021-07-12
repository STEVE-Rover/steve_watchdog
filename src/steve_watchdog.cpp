#include <steve_watchdog/steve_watchdog.h>


SteveWatchdog::SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
    getTopics(topic_list_);
}

bool SteveWatchdog::getTopics(std::vector<TopicInfo>& topic_list)
{
    // get how many topics need to be monitored
    int nb_of_topics;
    bool nb_of_topics_exists = private_nh_.getParam("nb_of_topics", nb_of_topics);
    if(!nb_of_topics_exists)
    {
        ROS_FATAL("Missing nb_of_topics parameter");
        return false;
    }
    ROS_INFO("Monitoring %d topics", nb_of_topics);

    // retrieve all topics
    for(int i=0; i<nb_of_topics; i++)
    {
        TopicInfo topic;
        std::string sensor = "sensor_" + std::to_string(i+1);

        bool name_exists = private_nh_.getParam( sensor + "/name" , topic.name);
        bool topic_exists = private_nh_.getParam( sensor + "/topic" , topic.topic);
        bool min_freq_exists = private_nh_.getParam( sensor + "/min_freq" , topic.min_freq);
        bool max_freq_exists = private_nh_.getParam( sensor + "/max_freq" , topic.max_freq);
        if(!(name_exists && topic_exists && min_freq_exists && max_freq_exists))
        {
            ROS_FATAL("One or more parameter for %s is missing", sensor.c_str());
            return false;
        }
        // TODO: is there a way to subscribe only to the message event instead of receiving the message data?
        topic.sub = nh_.subscribe<topic_tools::ShapeShifter>(topic.topic, 1, boost::bind(&SteveWatchdog::topicCB, this, _1));
        std::cout << sensor << std::endl;
        printTopicInfo(topic);
        topic_list_.push_back(topic);
    }
}

void SteveWatchdog::printTopicInfo(TopicInfo topic)
{
    std::cout << "name: " << topic.name << std::endl;
    std::cout << "topic: " << topic.topic << std::endl;
    std::cout << "min_freq: " << topic.min_freq << std::endl;
    std::cout << "max_freq: " << topic.max_freq << std::endl << std::endl;
}

void SteveWatchdog::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steve_watchdog");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    SteveWatchdog steve_watchdog(nh, private_nh);
    ros::spin();
    return 0;
}