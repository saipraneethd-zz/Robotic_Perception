#!/usr/bin/env python
import rospy
from filter import Filter

def main():

    rospy.init_node('filter_node', log_level = rospy.DEBUG)
    filt = Filter()
    rospy.spin()

if __name__ == "__main__":
    main()
