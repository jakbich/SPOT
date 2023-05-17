import wave
from dataclasses import dataclass, asdict

import pyaudio
import speech_recognition as sr
import warnings
import os
import argparse

import rospy 
from std_msgs.msg import String

@dataclass
class StreamParams:
    format: int = pyaudio.paInt16
    channels: int = 2
    rate: int = 44100
    frames_per_buffer: int = 1024
    input: bool = True
    output: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


class Transcriber:
    """Recorder uses the blocking I/O facility from pyaudio to record sound
    from mic.
    Attributes:
        - stream_params: StreamParams object with values for pyaudio Stream
            object
    """
    def __init__(self, stream_params: StreamParams) -> None:
        
        rospy.init_node('speech_transcriber')
        self.pub = rospy.Publisher("/spot/speech/response", String)
        


    
    # Initializes the Transcriber class with the given stream parameters and
    # creates a ROS node to publish the transcribed speech.
        self.stream_params = stream_params
        self._pyaudio = None
        self._stream = None
        self._wav_file = None
        self.recorder = sr.Recognizer()

        
   

    def _create_recording_resources(self, save_path: str) -> None:
        self._pyaudio = pyaudio.PyAudio()
        self._stream = self._pyaudio.open(**self.stream_params.to_dict())
        self._create_wav_file(save_path)

    def _create_wav_file(self, save_path: str):
        self._wav_file = wave.open(save_path, "wb")
        self._wav_file.setnchannels(self.stream_params.channels)
        self._wav_file.setsampwidth(self._pyaudio.get_sample_size(self.stream_params.format))
        self._wav_file.setframerate(self.stream_params.rate)

    def _write_wav_file_reading_from_stream(self, duration: int) -> None:
        for _ in range(int(self.stream_params.rate * duration / self.stream_params.frames_per_buffer)):
            audio_data = self._stream.read(self.stream_params.frames_per_buffer)
            self._wav_file.writeframes(audio_data)

    def _close_recording_resources(self) -> None:
        self._wav_file.close()
        self._stream.close()
        self._pyaudio.terminate()


    def transcribe(self, save_path:str, duration:int) -> str:
        print("Start speaking...")
        self._create_recording_resources(save_path)
        self._write_wav_file_reading_from_stream(duration)
        self._close_recording_resources()
        print("Stop speaking \n\n")

        r = sr.Recognizer()
        with sr.AudioFile(save_path) as source:
            audio = r.record(source)
        os.remove("audio.wav") 
        return r.recognize_google(audio).lower()
# 
    # def transcribe(self, duration = None):
    #     with sr.Microphone() as source:
    #         if duration is not None:
    #             print(f"Robot: Please speak for {duration} seconds.")
    #         else:
    #             print("Robot: Please speak.")
    #         audio = self.recorder.listen(source, phrase_time_limit=duration)
    #     try:
    #         # Use Google Speech Recognition to transcribe the audio
    #         text = r.recognize_google(audio)
    #         return text.lower()
    #     except sr.UnknownValueError:
    #         # If the speech cannot be transcribed, return None
    #         return None

    
    def conversation(self):

    # Conversation to figure out which item to get
    #  answer = transcriber.transcribe(duration=3, save_path="audio.wav")
        conv_complete = False

        remap = {
            "yeah": "yes",
            "sure": "yes",
            "of course": "yes",
            "absolutely": "yes",
            "definitely": "yes",
            "yah": "yes",    
            "yea": "yes", 
            "yeah": "yes",   
            "yep": "yes",    
            "ok": "yes",     
            "okay": "yes",   

            "nope": "no",    
            "nay": "no",     
            "nah": "no",     
            "negative": "no", 
            "not": "no"      
        }

        print("\n\nSPOT: Hello, do you need anything?")
        answer = transcriber.transcribe(duration=5, save_path="audio.wav")

        while answer is not None:
        # Use remapping to also include the most common ways of saying yes or no
            if answer.lower() in remap:
                answer = remap[answer]

            if "yes" in answer:
                print(f"SPOT: Which item do you need?")
                answer = transcriber.transcribe(duration=5, save_path="audio.wav")
                if answer is not None:
                    print(f"SPOT: You said you need {answer}, is that correct?")
                    confirm = transcriber.transcribe(duration=5, save_path="audio.wav")

                    while confirm is not None:
                    # Use remapping to also include the most common ways of saying yes or no
                        if confirm.lower() in remap:
                            confirm = remap[confirm]

                        if "yes" in confirm:
                            print(f"SPOT: Great, I will start searching {answer} for you.")
                            conv_complete = True
                            break
                        elif "no" in confirm:
                            print(f"SPOT: I'm sorry, let's try again.")
                            break
                        else:
                            print("SPOT: I didn't understand your response. Please say yes or no.")
                            confirm = transcriber.transcribe(duration=5, save_path="audio.wav")
                    if confirm is None:
                        print(f"SPOT: I'm sorry, I didn't hear your response.")
                        break
                else:
                    print(f"SPOT: I'm sorry, I didn't hear your response.")
                    break
            elif "no" in answer:
                print(f"SPOT: Okay, have a good day!")
                break
            else:
                print("SPOT: I didn't understand your response. Please say yes or no.")
                answer = transcriber.transcribe(duration=5, save_path="audio.wav")
            if answer is None:
                print(f"SPOT: I'm sorry, I didn't hear your response.")
                break
            if conv_complete:
                break
            
            
            
        transcriber.pub.publish(answer)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Record audio from microphone")
    parser.add_argument("-d", "--debug", action="store_true", default=False, help="Display debug information")
    args, unknown = parser.parse_known_args()

    debug = rospy.get_param("/debug")

    if not debug:
        # Disable warnings
        import warnings
        warnings.filterwarnings("ignore")
        # Redirect the error output to /dev/null
        os.close(2)
        os.open(os.devnull, os.O_RDWR)
    

    try:
        stream_params = StreamParams()
        transcriber  = Transcriber(stream_params)
        transcriber.conversation()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn("The node speech_recognition could not be launched")
        pass


