# steve_watchdog
This package subscribes and monitors the publishing frequency of any number and type of topic. If one or more of the topics is publishing under the minimum rate, the status of the watchdog will be false. Otherwise it will be true.

## How it works
The topic is monitored by measuring the time elapsed between every every messages. If the time is longer than the period of the minimum frequency, the topic status will be false.

## Subscribed topics
The list of topics the watchdog is subscribed to is listed in `config/watchdog_topics.yaml` which has the following structure:
```yaml
nb_of_topics: 1
topic_1:  # Must have the structure topic_X
  name: imu  # Human readable name, can be anything
  topic_name: /imu/data
  min_freq: 90  # Hz
```
Topic numbers start at 1 and should enf at the the specified number of topics without any gaps.
A topic is considered to have a good publishing frequency if it lies above the minimum.


## Published topics
* ~status (std_msgs/Bool): true if all of the topic frequencies are good, false otherwise.
* ~info (steve_watchdog/TopicArray): List of all monitored topics with their status.

## Common bugs
* High frequency topics oscillate between true and false in Gazebo. This is caused by the simulated clock which has a bigger time step than the wall clock. To solve this, reduce the publishing frequency of the topic or make the time step smaller.