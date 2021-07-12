#include <steve_watchdog/steve_watchdog.h>


SteveWatchdog::SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
    status_pub_ = nh_.advertise<std_msgs::Bool>("status", 1);
    getTopics(topic_list_);
}

bool SteveWatchdog::getTopics(std::vector<std::shared_ptr<TopicMonitor>>& topic_list)
{
    // get how many topics need to be monitored
    bool nb_of_topics_exists = private_nh_.getParam("nb_of_topics", nb_of_topics_);
    if(!nb_of_topics_exists)
    {
        ROS_FATAL("Missing nb_of_topics parameter");
        return false;
    }
    ROS_INFO("Monitoring %d topics", nb_of_topics_);

    // retrieve all topics
    for(int i=0; i<nb_of_topics_; i++)
    {
        std::shared_ptr<TopicMonitor> topic = std::make_shared<TopicMonitor>(nh_, private_nh_);
        std::string sensor = "sensor_" + std::to_string(i+1);

        bool name_exists = private_nh_.getParam( sensor + "/name" , topic->name_);
        bool topic_exists = private_nh_.getParam( sensor + "/topic" , topic->topic_);
        bool min_freq_exists = private_nh_.getParam( sensor + "/min_freq" , topic->min_freq_);
        bool max_freq_exists = private_nh_.getParam( sensor + "/max_freq" , topic->max_freq_);
        if(!(name_exists && topic_exists && min_freq_exists && max_freq_exists))
        {
            ROS_FATAL("One or more parameter for %s is missing", sensor.c_str());
            return false;
        }
        // TODO: is there a way to subscribe only to the message event instead of receiving the message data?
        topic->start();
        topic->createSubscription();
        std::cout << sensor << std::endl;
        topic->printTopicInfo();
        topic_list_.push_back(topic);
    }
}

int SteveWatchdog::getNbOfTopics()
{
    return nb_of_topics_;
}

void SteveWatchdog::run()
{
    ros::Rate r(10);
    while (ros::ok())
    {
        std_msgs::Bool out_msg;
        status_ = true;
        for(std::shared_ptr<TopicMonitor> t : topic_list_)
        {
            if(t->getStatus() == false)
                status_ = false;                
        }
        out_msg.data = status_;
        status_pub_.publish(out_msg);
        r.sleep();
    }
}

TopicMonitor::TopicMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
}

void TopicMonitor::printTopicInfo()
{
    std::cout << "name: " << name_ << std::endl;
    std::cout << "topic: " << topic_ << std::endl;
    std::cout << "min_freq: " << min_freq_ << std::endl;
    std::cout << "max_freq: " << max_freq_ << std::endl << std::endl;
}

void TopicMonitor::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg)
{
    ticks_++;
}

void TopicMonitor::run()
{
    // Check if two messages have been received in the last two periods.
    // Only checking one period is not sufficient because there will always eventually be a period
    // containing one message for any lower publishing frequency.
    ros::Rate r(min_freq_/2);
    while (ros::ok())
    {
        if(ticks_ < 2)
        {
            status_ = false;
        }
        else
        {
            status_ = true;
        }
        ticks_ = 0;
        r.sleep();
    }
}

void TopicMonitor::createSubscription()
{
    sub_ = nh_.subscribe<topic_tools::ShapeShifter>(topic_, 1, boost::bind(&TopicMonitor::topicCB, this, _1));
}

void TopicMonitor::start()
{
    thread_ = std::thread(&TopicMonitor::run, this);
}

bool TopicMonitor::getStatus()
{
    return status_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steve_watchdog");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    SteveWatchdog steve_watchdog(nh, private_nh);
    ros::AsyncSpinner spinner(steve_watchdog.getNbOfTopics());
    spinner.start();
    steve_watchdog.run();
    ros::waitForShutdown();
    return 0;
}