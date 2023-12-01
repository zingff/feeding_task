#!/usr/bin/env python
import rospy
from feeding_task.srv import SkewerStatusCheck, SkewerStatusCheckRequest
import kortex_driver.msg
import kortex_driver.srv


def call_skewer_status_check():
    rospy.wait_for_service('skewer_status_checker')
    try:
        stop_client = rospy.ServiceProxy('/base/stop_action', kortex_driver.srv.StopAction)
        stop_client(kortex_driver.msg.Empty())
        return True
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('skewer_status_check_client')
    # Example call to service
    skewer_checker_flag = True # or False, depending on your application logic
    call_skewer_status_check()
