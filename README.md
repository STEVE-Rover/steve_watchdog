# steve_watchdog
This package subscribes and monitors the publishing frequency of any number and type of topic. If one or more of the topics is publishing under the minimum rate, the status of the watchdog will be false. Otherwise it will be true.

## How it works
Every time a message from a topic is received, a counter is incremented. If the number of received messages for a two periods is lower than it should be, that means there is a problem with the publishing frequency.

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