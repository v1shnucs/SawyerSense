#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess
import os

def listen_and_transcribe():
    """
    Initialize ROS node, use system's speech recognition to transcribe audio
    and publish to /transcription topic
    """
    rospy.init_node('speech_input')
    pub = rospy.Publisher('/transcription', String, queue_size=10)
    
    rospy.loginfo("Speech recognition node started. Ready to listen...")
    
    # Install sox if not already installed
    try:
        subprocess.check_call(['which', 'sox'])
    except subprocess.CalledProcessError:
        rospy.logwarn("Installing sox for audio recording...")
        subprocess.call(['sudo', 'apt-get', 'install', '-y', 'sox'])
    
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Recording audio... Speak now (recording for 5 seconds)")
            
            # Record audio using sox
            audio_file = "/tmp/audio_input.wav"
            subprocess.call(['sox', '-d', '-r', '16000', '-c', '1', audio_file, 'trim', '0', '5'])
            
            rospy.loginfo("Processing speech...")
            
            # Use Google Speech Recognition API
            import speech_recognition as sr
            recognizer = sr.Recognizer()
            
            with sr.AudioFile(audio_file) as source:
                audio = recognizer.record(source)
                text = recognizer.recognize_google(audio)
                rospy.loginfo("Recognized text: " + text)
                pub.publish(text)
            
            # Clean up audio file
            os.remove(audio_file)
            
        except Exception as e:
            rospy.logerr("Error occurred: {0}".format(e))
        
        rospy.sleep(1)  # Small delay before next recording

if __name__ == '__main__':
    try:
        listen_and_transcribe()
    except rospy.ROSInterruptException:
        pass