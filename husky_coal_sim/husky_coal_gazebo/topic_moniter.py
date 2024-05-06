#!/usr/bin/env python3
import rospy
import sys
import importlib #manages imports
import subprocess

class TopicMonitor:
    def __init__(self, topic_name, topic_type_class):
        self.topic_name = topic_name
        self.reset_metrics()
        rospy.Subscriber(self.topic_name, topic_type_class, self.callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.display_metrics)

    def reset_metrics(self): #resets the metrics 
        self.data_count = 0
        self.total_bytes = 0
        self.start_time = rospy.Time.now()

    def callback(self, data):
        self.data_count += 1
        self.total_bytes += len(str(data)) #Gives the number of bytes in the message

    def display_metrics(self, event):
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed_time > 0:
            frequency = round(self.data_count / elapsed_time, 2) 
            mbps = round((self.total_bytes * 8) / (elapsed_time * 1024 * 1024), 4)
            rospy.loginfo(f"Frequency: {frequency} Hz, Data Rate: {mbps} Mbps")
        self.reset_metrics()  # Reset metrics after each interval

def get_topic_type(topic_name):
    command = f"rostopic type {topic_name}"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output, _ = process.communicate()
    if process.returncode != 0:
        rospy.logerr("Failed to find the type")
        sys.exit(1)
    return output.decode().strip()

def import_message_type(full_type_name):
    try:
        package, message_type = full_type_name.split('/')
        module = importlib.import_module(f"{package}.msg")
        return getattr(module, message_type)
    except Exception as e:
        rospy.logerr(f"Failed to import {full_type_name}: {e}")
        sys.exit(1)

if __name__ == '__main__':
    rospy.init_node('topic_monitor')
    if len(sys.argv) != 2:
        sys.exit(1)
    topic_name = sys.argv[1]
    topic_type_name = get_topic_type(topic_name)
    if not topic_type_name:
        rospy.logerr("No type found")
        sys.exit(1)
    topic_type_class = import_message_type(topic_type_name)
    monitor = TopicMonitor(topic_name, topic_type_class)
    rospy.spin()

