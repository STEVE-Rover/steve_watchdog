# steve_watchdog
This package subscribes and monitors the publishing frequency of any number and type of topic. If one or more of the topics is publishing under the minimum rate, the status of the watchdog will be false. Otherwise it will be true.

## How it works
The topic is monitored by measuring the time elapsed between every every messages. If the time is longer than the period of the minimum frequency, the topic status will be false.

## Subscribed topics
The list of topics the watchdog is subscribed to is listed in `config/watchdog_topics.yaml` which has the following structure:
```yaml
nb_of_topics: 1  # Number of topics to monitor
rate: 10  # Rate at which the info and status topics will be published
topic_1:  # Must be topic_x (starts at 1)
  name: topic1  # Human readable name, can be anything
  topic_name: /topic1  # Name of the topic
  min_freq: 40  # Minimum frequency
  use_average: false  # Use the average frequency from the accumulated messages. Usefull if from time to time a message comes in late but we don't want it to cause the status to be false.
  monitoring_rate: 10  # Rate at which to monitor this topic
```
Topic numbers start at 1 and should enf at the the specified number of topics without any gaps.
A topic is considered to have a good publishing frequency if it lies above the minimum.


## Published topics
* ~status (std_msgs/Bool): true if all of the topic frequencies are good, false otherwise.
* ~info (steve_watchdog/TopicArray): List of all monitored topics with their status.

## Common bugs
* High frequency topics oscillate between true and false in Gazebo. This is caused by the simulated clock which has a bigger time step than the wall clock. To solve this, reduce the publishing frequency of the topic or make the time step smaller.

## Possible improvements
* Add option to use message time stamp instead of time of receipt.