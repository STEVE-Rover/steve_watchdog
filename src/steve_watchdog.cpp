#include <steve_watchdog/steve_watchdog.h>


SteveWatchdog::SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
    status_pub_ = nh_.advertise<std_msgs::Bool>("status", 1);
    info_pub_ = nh_.advertise<steve_watchdog::TopicArray>("info", 1);
    createTopicMonitors();
}

/*!
   * Fetches the topic information from the parameter server and creates the TopicMonitor objects
   */
bool SteveWatchdog::createTopicMonitors()
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
        std::string topic_id = "topic_" + std::to_string(i+1);

        bool name_exists = private_nh_.getParam( topic_id + "/name" , topic->name_);
        bool topic_exists = private_nh_.getParam( topic_id + "/topic_name" , topic->topic_name_);
        bool min_freq_exists = private_nh_.getParam( topic_id + "/min_freq" , topic->min_freq_);
        if(!(name_exists && topic_exists && min_freq_exists))
        {
            ROS_FATAL("One or more parameter for %s is missing", topic_id.c_str());
            return false;
        }
        // TODO: is there a way to subscribe only to the message event instead of receiving the message data?
        topic->start();
        topic->createSubscription();
        std::cout << topic_id << std::endl;
        topic->printTopicMonitorInfo();
        topic_list_.push_back(topic);
    }
}

/*!
   * Returns the number of topics
   */
int SteveWatchdog::getNbOfTopics()
{
    return nb_of_topics_;
}

/*!
   * Main loop
   */
void SteveWatchdog::run()
{
    ros::Rate r(10);
    while (ros::ok())
    {
        std_msgs::Bool status_msg;
        steve_watchdog::TopicArray topic_array_msg;
        status_ = true;

        for(std::shared_ptr<TopicMonitor> t : topic_list_)
        {
            steve_watchdog::TopicStatus topic_status_msg;
            topic_status_msg.name = t->name_;
            topic_status_msg.status = t->getStatus();
            topic_array_msg.status.push_back(topic_status_msg);
            if(t->getStatus() == false)
                status_ = false;                
        }
        status_msg.data = status_;
        topic_array_msg.header.stamp = ros::Time::now();
        status_pub_.publish(status_msg);
        info_pub_.publish(topic_array_msg);
        r.sleep();
    }
}

TopicMonitor::TopicMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
}

/*!
   * Prints the TopicMonitor info for debugging
   */
void TopicMonitor::printTopicMonitorInfo()
{
    std::cout << "name: " << name_ << std::endl;
    std::cout << "topic_name: " << topic_name_ << std::endl;
    std::cout << "min_freq: " << min_freq_ << std::endl;
}

/*!
   * Callback for when a new message is received
   */
void TopicMonitor::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg)
{
    ticks_++;
}

/*!
   * Main loop
   */
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

/*!
   * Create subscription to the topic
   */
void TopicMonitor::createSubscription()
{
    sub_ = nh_.subscribe<topic_tools::ShapeShifter>(topic_name_, 1, boost::bind(&TopicMonitor::topicCB, this, _1));
}

/*!
   * Start the topic monitoring thread
   */
void TopicMonitor::start()
{
    thread_ = std::thread(&TopicMonitor::run, this);
}

/*!
   * Returns the TopicMonitor status
   */
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