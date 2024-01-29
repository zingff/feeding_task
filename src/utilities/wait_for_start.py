#!/home/zing/anaconda3/envs/voice/bin/python

import rospy
from std_msgs.msg import String
from feeding_task.srv import WaitForStart, WaitForStartResponse

class VoiceMonitorService:
    def __init__(self):
        self.voice_text = None
        self.service = rospy.Service('/fsm/wait_for_start', WaitForStart, self.handle_wait_for_voice)
        self.voice_subscriber = rospy.Subscriber('/fsm/voice_stream_monitor', String, self.voice_callback)

    def voice_callback(self, msg):
        self.voice_text = msg.data
        rospy.loginfo("Received voice text: %s", self.voice_text)

    def handle_wait_for_voice(self, request):
        desired_texts = [u'\u5F00\u59CB', u'\u7ED3\u675F', u'\u505C\u6B62']

        rate = rospy.Rate(5)  # Adjust the rate

        while not rospy.is_shutdown():
            if self.voice_text in desired_texts:
                rospy.loginfo("Desired voice text received: %s", self.voice_text)
                return WaitForStartResponse(start_success=True, start_command=self.voice_text)

            rate.sleep()

        return WaitForStartResponse(start_success=False, start_command="")

if __name__ == '__main__':
    rospy.init_node('voice_monitor_service')
    voice_monitor_service = VoiceMonitorService()
    rospy.spin()
