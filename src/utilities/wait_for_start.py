#!/home/zing/anaconda3/envs/voice/bin/python

import rospy
from std_msgs.msg import String
from feeding_task.srv import WaitForStart, WaitForStartResponse

class VoiceMonitorService:
    def __init__(self):
        self.voice_text = None
        self.service = rospy.Service('/fsm/wait_for_start', WaitForStart, self.handle_wait_for_voice)
        self.voice_subscriber = rospy.Subscriber('/fsm/voice_recognization_results', String, self.voice_callback)

    def voice_callback(self, msg):
        self.voice_text = msg.data
        rospy.loginfo("Received voice text: %s", self.voice_text)

    def handle_wait_for_voice(self, request):
        desired_words = [u'\u5F00\u59CB',u'\u6211\u8981', u'\u5F00\u59CB\u5403\u996D', u'\u6211\u8981\u5403\u996D']  # kaishi, woyao, kaishichifan, woyaochifan

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
          if self.voice_text is not None:
            # if self.voice_text in desired_words:
            if any(word in self.voice_text for word in desired_words):
                rospy.loginfo("Desired voice text received: %s", self.voice_text)
                return WaitForStartResponse(start_success=True, start_command=self.voice_text)

            rate.sleep()

        return WaitForStartResponse(start_success=False, start_command="null")

if __name__ == '__main__':
    rospy.init_node('voice_monitor_service')
    voice_monitor_service = VoiceMonitorService()
    rospy.spin()
