#!/usr/bin/env python
import rospy
from feeding_task.srv import SkewerStatusCheck, SkewerStatusCheckRequest


def call_skewer_status_check(skewer_checker_flag):
    rospy.wait_for_service('skewer_status_checker')
    try:
        skewer_status_checker = rospy.ServiceProxy('skewer_status_checker', SkewerStatusCheck)
        resp = skewer_status_checker(skewer_checker_flag)
        return resp.skewer_status, resp.point_number
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('skewer_status_check_client')
    # Example call to service
    skewer_checker_flag = True # or False, depending on your application logic
    skewer_status, point_number = call_skewer_status_check(skewer_checker_flag)
    print(f"Skewer Status: {skewer_status}, Point Number: {point_number}")
