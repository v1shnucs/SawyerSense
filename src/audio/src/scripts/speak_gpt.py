#!/usr/bin/env python

from gtts import gTTS
import os
import rospy
import inspect
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int32, String

speak_finished_publisher = None
START_DELIMITER = "speak:"
END_DELIMITER = "actions:"

class SpeakError(Exception):
    def __init__(self, message="There was an error in the speaking process."):
        self.message = message
        super(SpeakError, self).__init__(self.message)

def extract_speak(input_string):
    """
    Given complete GPT response, extract only the string to be spoken.
    """
    global START_DELIMITER, END_DELIMITER
    try:
        start_index = int(input_string.index(START_DELIMITER)) + len(START_DELIMITER)
        end_index = input_string.index(END_DELIMITER)
        return input_string[start_index:end_index]
    except ValueError:
        return ""

def SpeakSentence(response):
    """
    Given text message, use text-to-speech to speak it.
    """
    rospy.loginfo("speak_gpt.py entered SpeakSentence")
    global speak_finished_publisher
    pub = rospy.Publisher('/speak_errors', String, queue_size=10)
    file_name = inspect.getfile(inspect.currentframe())

    try:
        sentence = response.data

        if(len(sentence) > 1):
            # Create gTTS object
            tts = gTTS(text=sentence, lang='en')
            # Save to temporary file
            tts.save("/tmp/temp.mp3")
            # Play the audio using system command
            os.system("mpg123 /tmp/temp.mp3")
            # Clean up
            os.remove("/tmp/temp.mp3")
            rospy.loginfo("speak_gpt.py spoke: " + sentence)
        else:
            rospy.loginfo("speak_gpt.py Empty string received by engine")
        speak_finished_publisher.publish("speak_finished")
    except SpeakError as e:
        error_message = "SpeakError in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        pub.publish(error_message)
    except Exception as e:
        error_message = "An unexpected error occurred in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        pub.publish(error_message)

def speak_output():
    """
    Subscribes to the /gpt_response topic, and when receives GPT response,
    parses for words to be spoken and uses pyttsx3 to speak.
    Publishes to topic /speak_finished when done to let listener know
    can start transcribing speech to text again.
    """
    global speak_finished_publisher
    rospy.init_node('speak_output')
    rospy.loginfo("speak_gpt.py entered")
    speak_finished_publisher = rospy.Publisher('/speak_finished', String, queue_size=10)
    rospy.Subscriber("/speak_text", String, SpeakSentence)
    rospy.spin()
    rospy.loginfo("speak_gpt.py exiting")

if __name__ == '__main__':
    speak_output()