#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess
import os

# Flag to track if we can record
can_record = True

def speak_finished_callback(msg):
    """Callback when speech output is finished"""
    global can_record
    can_record = True
    rospy.loginfo("Speech finished, ready to record again")

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
    
    # Subscribe to speak_finished topic
    rospy.Subscriber('/speak_finished', String, speak_finished_callback)
    
    while not rospy.is_shutdown():
        global can_record
        try:
            # Only record if speech output is not happening
            if not can_record:
                rospy.sleep(0.1)  # Small delay to prevent busy waiting
                continue
                
            can_record = False  # Set flag to prevent recording while speaking
            rospy.loginfo("Recording audio... Waiting for speech input")
            
            # Record audio using sox with extended silence detection
            audio_file = "/tmp/audio_input.wav"
            subprocess.call([
                    'sox', '-d', '-r', '16000', '-c', '1', audio_file,
                    'silence', '1', '0.05', '0.1%', '1', '1.5', '0.1%'
                ])
            
            # Check if the audio file exists and is not empty
            if not os.path.exists(audio_file) or os.path.getsize(audio_file) == 0:
                rospy.logwarn("No valid audio recorded. Skipping transcription.")
                continue
            
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