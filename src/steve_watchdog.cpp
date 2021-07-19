#include <steve_watchdog/steve_watchdog.h>


SteveWatchdog::SteveWatchdog(ros::NodeHandle nh, ros::NodeHandle private_nh):
    nh_(nh),
    private_nh_(private_nh)
{
    cmd_vel_sub_ = nh_.subscribe("cmd_vel_in", 1, &SteveWatchdog::cmdVelCB, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
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

        // TODO: TopicMonitor params should be passed to constructor intead of member variables
        bool name_exists = private_nh_.getParam( topic_id + "/name" , topic->name_);
        bool topic_exists = private_nh_.getParam( topic_id + "/topic_name" , topic->topic_name_);
        bool min_freq_exists = private_nh_.getParam( topic_id + "/min_freq" , topic->min_freq_);
        if(!(name_exists && topic_exists && min_freq_exists))
        {
            ROS_FATAL("One or more parameter for %s is missing", topic_id.c_str());
            return false;
        }
        // TODO: is there a way to subscribe only to the message event instead of receiving the message data?
        topic->min_time_ = 1 / topic->min_freq_;
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

 void SteveWatchdog::cmdVelCB(const geometry_msgs::Twist::ConstPtr msg)
 {
     if(status_ == true)
     {
         cmd_vel_pub_.publish(msg);
     }
     else
     {
         cmd_vel_pub_.publish(geometry_msgs::Twist());
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
    std::cout << "min_freq: " << min_freq_ << std::endl << std::endl;
}

/*!
   * Callback for when a new message is received
   */
void TopicMonitor::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter>& msg)
{
    const std::lock_guard<std::mutex> lock(mu_);
    ticks_++;
    stamps_.push_back(ros::Time::now());
}

/*!
   * Main loop
   */
void TopicMonitor::run()
{
    // TODO: change this explanation
    // Check if two messages have been received in the last two periods.
    // Only checking one period is not sufficient because there will always eventually be a period
    // containing one message for any lower publishing frequency.
    ros::Rate r(min_freq_);
    while (ros::ok())
    {
        {
            const std::lock_guard<std::mutex> lock(mu_);
            if(stamps_.size() >= 2)
            {
                std::cout << "stamps_.size() >= 2" <<std::endl;
                std::cout << "min_time_: " << min_time_ << std::endl;
                for(int i=0; i<stamps_.size()-1; i++)
                {
                    float elapsed_time = (stamps_[i+1] - stamps_[i]).toSec();
                    std::cout << "elapsed_time: " << elapsed_time << std::endl;
                    if(elapsed_time <= min_time_)
                    {
                        status_ = true;
                    }
                    else
                    {
                        status_ = false;
                    }
                }
            }
            else
            {
                status_ = false;
            }
            if(stamps_.size() >= 1)
            {
                ros::Time last_stamp = stamps_[stamps_.size()-1];
                stamps_.clear();
                stamps_.push_back(last_stamp);
            }
            ticks_ = 0;
            std::cout << "status_: " << status_ << std::endl;
        }
        //TODO: need mutex for ticks, stamps and status
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
    ros::AsyncSpinner spinner(1);  // steve_watchdog.getNbOfTopics()
    spinner.start();
    steve_watchdog.run();
    ros::waitForShutdown();
    return 0;
}