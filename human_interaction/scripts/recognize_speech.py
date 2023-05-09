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
    # Initializes the Transcriber class with the given stream parameters and
    # creates a ROS node to publish the transcribed speech.
        self.stream_params = stream_params
        self._pyaudio = None
        self._stream = None
        self._wav_file = None

        
        rospy.init_node('speech_transcriber')
        self.pub = rospy.Publisher("/spot/speech/response", String)



    def record(self, duration: int, save_path: str) -> None:
        """Record sound from mic for a given amount of seconds.
        :param duration: Number of seconds we want to record for
        :param save_path: Where to store recording
        """
        print("Start recording...")
        self._create_recording_resources(save_path)
        self._write_wav_file_reading_from_stream(duration)
        self._close_recording_resources()
        print("Stop recording")

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


    def transcribe(self, audio_file: str) -> str:
        r = sr.Recognizer()
        with sr.AudioFile(audio_file) as source:
            audio = r.record(source)
        #os.remove("audio.wav") 
        return r.recognize_google(audio)

    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Record audio from microphone")
    parser.add_argument("-d", "--debug", action="store_true", default=False, help="Display debug information")
    args = parser.parse_args()

    if not args.debug:
        # Disable warnings
        import warnings
        warnings.filterwarnings("ignore")

        # Redirect the error output to /dev/null
        os.close(2)
        os.open(os.devnull, os.O_RDWR)
    
    try:
        stream_params = StreamParams()
        transcriber  = Transcriber(stream_params)
        transcriber.record(3, "audio.wav")
        text = transcriber.transcribe("audio.wav")
        transcriber.pub.publish(text)


        text = text.split()
        dist, dir, obj = text[0],text[1],text[2]
        print(f"Spot will now walk {dist} steps in the direction {dir} to get {obj}")
        
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launched")
        pass


