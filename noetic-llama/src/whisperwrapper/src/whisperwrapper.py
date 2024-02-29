#!/usr/bin/env python3

from ollamamessages.msg import WhisperTranscription

import speech_recognition as sr
import tempfile
import requests
import rospy 
import time
import json
import os

whisper_api_url = rospy.get_param("/stt/whisper_api_url", "192.168.122.1:9000")
pause = rospy.get_param("/stt/speech_recogn_pause_time", 0.8)
energy = rospy.get_param("/stt/speech_recogn_energy", 400) 
dynamic_energy = rospy.get_param("/stt/speech_recogn_dyn_energy_flag", False)
microphone_device = rospy.get_param("/stt/microphone_device", 1)

class WhisperWrapper:
    def __init__(self) -> None:
        self.transcription_pub = rospy.Publisher("/stt/transcription", WhisperTranscription, queue_size = 1)

        self.record_audio(pause, energy, dynamic_energy, microphone_device)

    def record_audio(self, pause, energy, dynamic_energy, microphone_device):
        recogniser = sr.Recognizer()
        recogniser.energy_threshold = energy
        recogniser.pause_threshold = pause
        recogniser.dynamic_energy_threshold = dynamic_energy

        with sr.Microphone(sample_rate = 10000) as microphone:
            rospy.loginfo("Listening...")
            while True and not rospy.is_shutdown():
                audio = recogniser.listen(microphone)

                with tempfile.NamedTemporaryFile(mode = "wb", suffix = ".wav", delete = False) as f:
                    audio_path = f.name
                    f.write(audio.get_wav_data())

                rospy.loginfo("I heard something... Written to %s" % audio_path)
                req = requests.post(
                    "http://%s/asr?output=json" % whisper_api_url,
                    files = {"audio_file": open(audio_path, "rb")}
                )
                os.remove(audio_path)
                o = req.json()
                rospy.loginfo("Transcribed '%s'" % o["text"])
                if o["text"] != "":
                    self.transcription_pub.publish(
                        text = o["text"],
                        language = o["language"],
                        temperature = o["segments"][0]["temperature"],
                        avg_logprob = o["segments"][0]["avg_logprob"],
                        compression_ratio = o["segments"][0]["compression_ratio"],
                        no_speech_prob = o["segments"][0]["no_speech_prob"]
                    )

                    

if __name__ == "__main__":
    rospy.init_node("whisper_wrapper")
    whisperwrapper = WhisperWrapper()
    


