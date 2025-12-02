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
                rospy.logwarn("No valid audio recorded. Restarting listener.")
                # Allow recording again since nothing valid was captured
                can_record = True
                try:
                    if os.path.exists(audio_file):
                        os.remove(audio_file)
                except Exception:
                    pass
                continue
            
            rospy.loginfo("Processing speech...")
            
            # Use Google Speech Recognition API
            import speech_recognition as sr
            recognizer = sr.Recognizer()
            
            with sr.AudioFile(audio_file) as source:
                audio = recognizer.record(source)
                try:
                    text = recognizer.recognize_google(audio)
                    rospy.loginfo("Recognized text: " + text)
                    pub.publish(text)
                    # Successful recognition: do NOT re-enable recording here.
                    # Wait for /speak_finished to set can_record = True to avoid overlapping TTS.
                except sr.UnknownValueError:
                    rospy.logwarn("Speech was unintelligible. Restarting listener.")
                    can_record = True
                except sr.RequestError as e:
                    rospy.logerr("Could not request results from Speech Recognition service: %s", e)
                    # Re-enable listening after service error so user can try again
                    can_record = True
                finally:
                    # Clean up audio file
                    try:
                        if os.path.exists(audio_file):
                            os.remove(audio_file)
                    except Exception:
                        pass
            
        except Exception as e:
            rospy.logerr("Error occurred: {0}".format(e))
            # Make sure we can try recording again after unexpected errors
            can_record = True
        
        rospy.sleep(1)  # Small delay before next recording

if __name__ == '__main__':
    try:
        listen_and_transcribe()
    except rospy.ROSInterruptException:
        pass