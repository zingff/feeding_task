import rospy
from std_msgs.msg import String
import speech_recognition as sr

def speech_to_text(language, pub):
    recognizer = sr.Recognizer()
    # recognizer.energy_threshold = 447
    # recognizer.dynamic_energy_threshold = True

    # Capture audio from a microphone
    with sr.Microphone() as source:
        print("说话!")
        audio = recognizer.listen(source)

    # Recognize speech and convert to text
    try:
        text = recognizer.recognize_google(audio, language=language)
        print("语音识别结果: " + text)

        # Publish the text to a ROS topic
        pub.publish(text)
    except sr.UnknownValueError:
        print("听不懂")
    except sr.RequestError as e:
        print("无响应; {0}".format(e))

def voice_recognition_node():
    # Initialize the ROS node
    rospy.init_node('voice_recognition_node', anonymous=True)

    # Create a publisher for the text message
    text_pub = rospy.Publisher('/fsm/voice_stream_monitor', String, queue_size=10)

    # Set the language for speech recognition
    language = 'cmn-CN'  # Replace with the desired language code

    # Run the voice recognition loop
    while not rospy.is_shutdown():
        speech_to_text(language, text_pub)

if __name__ == '__main__':
    try:
        voice_recognition_node()
    except rospy.ROSInterruptException:
        pass
