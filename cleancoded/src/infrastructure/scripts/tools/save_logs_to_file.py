#!/usr/bin/env python3

import rospy
import argparse
import csv
from infrastructure.msg import *



class SaveLogsToFile:
    def __init__(self, filename, fileextention, topic, msg_type):
        self.filename = filename
        if 'txt' in fileextention:
            self.file = open(self.filename, 'a', encoding='utf-8')
            rospy.Subscriber(topic, msg_type, self.write_to_txt_file)
        elif 'csv' in fileextention:
            self.file = open(self.filename, 'a')
            rospy.Subscriber(topic, msg_type, self.write_to_csv_file)
        else:
            rospy.logerr('File extension is not supported. Please use txt or csv.')
            exit(1)

    def write_to_txt_file(self, data):
        self.file.write(str(rospy.get_time().now()) + ',' + str(data) + '\n')
            
    def write_to_csv_file(self, data):
        writer = csv.writer(self.file)
        writer.writerow(str(rospy.get_time().now()) + ',' + str(data) + '\n')

    def close(self):
        if self.file is not None:
            self.file.close()


if __name__ == '__main__':
    try:
        rospy.init_node('save_logs_to_file', anonymous=True)
        parser = argparse.ArgumentParser(
                    description='A sample Argument Parser.' )

        parser.add_argument('-t',
                            '--topics',
                            nargs='+',
                            required=True)
        parser.add_argument('-m',
                            '--topics_msg_types',
                            nargs='+',
                            required=True)
        parser.add_argument('-f',
                            '--file_name',
                            required=False)
        parser.add_argument('-e',
                            '--file_extension',
                            help='File extension (e.g. txt, csv, etc.)',
                            required=False)
        args = parser.parse_args()
        topics = args.topics
        msg_types = args.topics_msg_types
        FILENAME = args.file_name
        FILE_EXTENSION = args.file_extension

        for i,topic in enumerate(topics):
            if FILENAME is None:
                FILENAME = f'{topic}_log.{FILE_EXTENSION}'
            else:
                FILENAME = f'{topic}_{FILENAME}.{FILE_EXTENSION}'
            topic_type = msg_types[i]
            SaveLogsToFile(FILENAME, FILE_EXTENSION, topic, topic_type)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass