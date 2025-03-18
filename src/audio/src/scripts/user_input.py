#!/usr/bin/env python2
"""
Module Name: listener_node.py

Description:
    Listens for wake word "Hey Sawyer" (just like "Hey Alexa") to start transcribing.
    Publishes transcription to /transcription. 
    Plays a sound, displays an image, and turns on green light when listening after wake word.
    Disables wake words and listening after transcribing speech.
    Re-enables wake words and listening after notified Sawyer done speaking and acting in topic /cycle_status.
    
Usage:
    rosrun speech_recognition listener.py

Author:
    Andrew Ge

Date: 1/13/2025
"""

import rospy
from std_msgs.msg import String
import language_tool_python as ltp
from playsound import playsound
import speech_recognition as sr

SAMPLE_RATE = 44100
WAKE_WORD = "hey Sawyer"
SOUND_FILE_PATH = "listening_alert.mp3"
COMMAND_WRAPPER = "{data: '{\"command_type\": \"{}\", \"{}\": \"{}\"}'}"
REPEAT_REQUEST_STRING = "Sorry, I didn't catch that. Could you say it again?"
REQUEST_ERROR_STRING = "Sorry, it looks like Google transcription services aren't working. Please consult the administrators."
QUIT = "Quit."
ACTION_FINISHED = "action_finished"
SPEECH_FINISHED = "speech_finished"
MAX_FAILURES = 30

class ListenerNode:
    """
    A class that represents the listener node, which listens for the wake words 'hey Sawyer' to 
    begin transcribing text. 

    Methods
    -------
    _execute_command(msg):
        Upon seeing publish on /commands topic, uses devel_sawyer.py or
        prod_sawyer.py to execute actions. 
    """

    def __init__(self, lang_tool, recognizer, microphone):
        self._action_finished = True
        self._speech_finished = True
        self._lang_tool = lang_tool
        self._recognizer = recognizer
        self._microphone = microphone
        self._command_publisher = rospy.Publisher(
            "/commands", String, queue_size=10)
        self._transcription_publisher = rospy.Publisher(
            "/transcription", String, queue_size=10)
        rospy.Subscriber("/cycle_status", String, self._update_cycle_status)
        self._begin_listening_cycle()

    def _action_is_finished(self):
        return self._action_finished

    def _set_action_finished(self, is_finished):
        self._action_finished = is_finished

    def _speech_is_finished(self):
        return self._speech_finished

    def _set_speech_finished(self, is_finished):
        self._speech_finished = is_finished

    def _publish_command(self, command_type, key, value):
        self._command_publisher.publish(
                    COMMAND_WRAPPER.format(command_type, key, value))

    def _begin_listening_cycle(self):
        """
        Start to listening cycle.
        Called as last item of initialization of ListenerNode object.
        """
        self._listen_for_wake_word()

    def _update_cycle_status(self, msg):
        """
        Called when updates received on /cycle_status.
        Expects to receive strings 'action_finished' or 'speech_finished'.
        Updates corresponding variables, then checks if both are true. If both are true, marks
        robot as not busy, and begins listening again.
        """
        if msg.data == ACTION_FINISHED:
            self._set_action_finished(True)
        elif msg.data == SPEECH_FINISHED:
            self._set_speech_finished(True)
        else:
            rospy.logwarn(
                "listener_node.py received unrecognized String in /cycle_status: {}".format(msg.data))
        if self._action_finished and self._speech_finished:
            self._listen_for_wake_word()

    def _listen_for_wake_word(self):
        """
        Begin listening for wake words 'hey Sawyer'.
        """
        num_failures = 0
        with self._microphone as source:
            self._recognizer.adjust_for_ambient_noise(source)
            rospy.roslog("Listener node waiting for wake word 'Hey Sawyer'.")

            while num_failures < MAX_FAILURES:
                try:
                    audio = self._recognizer.listen(source)
                    command = self._recognizer.recognize_google(audio).lower()

                    if WAKE_WORD in command:
                        while self._publish_transcription(source):
                            pass
                        return
                    num_failures += 1
                # detecting unclear audio before wake words indicate best suited to implying
                # user should repeat the wake words
                except sr.UnknownValueError:
                    num_failures += 1
                    continue
                except sr.RequestError:
                    self._publish_command("speaker", "words", REQUEST_ERROR_STRING)
                    rospy.logerror(
                        "RequestError for transcription service in listener_node.py.")
                    rospy.loginfo(
                        "listener_node.py found RequestError, shutting down.")
                    rospy.shutdown()
            if num_failures >= MAX_FAILURES:
                rospy.loginfo("listener_node.py timed out, shutting down.")
                rospy.shutdown()

    def _process_audio(self, audio):
        """
        Use Google Cloud speech-to-text service from the speech_recognition package to transcribe.
        Then use the grammar check from the language_tool_python library.
        """
        transcript = self._recognizer.recognize_google_cloud(audio)
        errors = self._lang_tool.check(transcript)
        corrected_transcript = ltp.utils.correct(transcript, errors)
        return corrected_transcript

    def _publish_transcription(self, source):
        """
        The transcribed words are published to /transcription, then the light is turned off,
        and _action_finished and _speech_finished are set to false.

        Returns true if function needs to be run again to continue to transcribe.
        """
        playsound(SOUND_FILE_PATH)
        self._publish_command("display", "image_name", "listening")
        self._publish_command("light", "light_state", "green")
        self._recognizer.adjust_for_ambient_noise(source)

        try:
            audio = self._recognizer.listen(source)
            corrected_transcript = self._process_audio(audio)

            if corrected_transcript == QUIT:
                rospy.loginfo(
                    "listener_node.py received 'quit' instruction, shutting down.")
                rospy.shutdown()

            self._transcription_publisher.publish(corrected_transcript)
            self._publish_command("display", "image_name", "thinking")
            self._publish_command("light", "light_state", "off")
            self._set_action_finished(False)
            self._set_speech_finished(False)
            return False
        except sr.UnknownValueError:
            self._publish_command("speaker", "words", REPEAT_REQUEST_STRING)
            return True
        except sr.RequestError:
            self._publish_command("speaker", "words", REQUEST_ERROR_STRING)
            rospy.logerror(
                "RequestError for transcription service in listener_node.py.")
            rospy.loginfo(
                "listener_node.py found RequestError, shutting down.")
            rospy.shutdown()

def main():
    """
    Initializes init_node 'listener' and ListenerNode().
    """
    rospy.init_node('listener', anonymous=False)
    ListenerNode(ltp.LanguageTool('en-US'), sr.Recognizer(), sr.Microphone())
    rospy.loginfo(
        "listener_node.py now listening for wake word 'hey Sawyer' and directions")
    rospy.spin()
    rospy.loginfo("listener_node.py now exiting")

if __name__ == '__main__':
    main()
