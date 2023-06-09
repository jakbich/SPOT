import wave
from dataclasses import dataclass, asdict

import pyaudio
import speech_recognition as sr
import warnings
import os
import argparse
import pyttsx3

import rospy 
import actionlib
from std_msgs.msg import String
from human_interaction.msg import ConversationAction, ConversationFeedback, ConversationResult

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


class ConversationServer:
    """Recorder uses the blocking I/O facility from pyaudio to record sound
    from mic.
    Attributes:
        - stream_params: StreamParams object with values for pyaudio Stream
            object
    """

    def __init__(self, stream_params: StreamParams) -> None:
        """Initializes the Recorder class with the given stream parameters and
        creates a ROS node to publish the recorded speech.
        Args:
            - stream_params: StreamParams object with values for pyaudio Stream
                object
        """
        # Initializes the ROS server for the conversation action
        self.server = actionlib.SimpleActionServer('conversation', ConversationAction, self.run_conv, False)
        self.server.start()
        rospy.loginfo("Action Server Conversation started...")

        # Initializes the feedback 
        self.feedback = ConversationFeedback()
    
        # Initializes the speech recognizer
        self.stream_params = stream_params
        self._pyaudio = None
        self._stream = None
        self._wav_file = None
        self.recorder = sr.Recognizer()
        self.engine = pyttsx3.init(driverName='espeak')
        
        # Remap the most common words that represent gets wrong
        self.remap = {
            "yeah": "yes",
            "correct": "yes",
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
            "certainly": "yes",
            "indeed": "yes",
            "affirmative": "yes",
            "roger": "yes",
            "aye": "yes",
            "correct": "yes",
            "right": "yes",
            "exactly": "yes",
            "positive": "yes",
            "surely": "yes",
            
            "nope": "no",
            "nay": "no",
            "nah": "no",
            "negative": "no",
            "not": "no",
            "no way": "no",
            "absolutely not": "no",
            "definitely not": "no",
            "certainly not": "no",
            "nope": "no",
            "negative": "no",
            "not at all": "no",
            "by no means": "no",
            "never": "no",
            "hardly": "no",
            "scarcely": "no",
            "not really": "no",
            "not sure": "no",
            "not necessarily": "no",
            "unlikely": "no",

            "cold": "old",
            "hold": "old",
            "told": "old",
            "gold": "old",
            "mold": "old",
            "bold": "old",
            "fold": "old",
            "sold": "old",
            "rolled": "old",
            "soul": "old",
            "pole": "old",
            "hole": "old",
            "coal": "old",
            "goal": "old",
            "toad": "old",
            "load": "old",

            "new": "new",
            "knew": "new",
            "grew": "new",
            "drew": "new",
            "who": "new",
            "blue": "new",
            "flue": "new",
            "shoe": "new",
            "stew": "new",
            "crew": "new",
            "few": "new",
            "you": "new",
            "cue": "new",
            "jew": "new",
            "view": "new",
            "pew": "new",
            "due": "new",
            "mu": "new",
            "nu": "new",
            "loo": "new"
            }

        
        


    def _create_recording_resources(self, save_path: str) -> None:
        """Creates the resources needed to record audio from the microphone and
        save it to a wav file.
        Args:
            - save_path: Path to the wav file to be created
        """
        self._pyaudio = pyaudio.PyAudio()
        self._stream = self._pyaudio.open(**self.stream_params.to_dict())
        self._create_wav_file(save_path)



    def _create_wav_file(self, save_path: str):
        """Creates a wav file with the given path and the stream parameters.
        Args:
            - save_path: Path to the wav file to be created
        """
        self._wav_file = wave.open(save_path, "wb")
        self._wav_file.setnchannels(self.stream_params.channels)
        self._wav_file.setsampwidth(self._pyaudio.get_sample_size(self.stream_params.format))
        self._wav_file.setframerate(self.stream_params.rate)



    def _write_wav_file_reading_from_stream(self, duration: int) -> None:
        """Reads audio data from the stream and writes it to the wav file.
        Args:
            - duration: Duration of the recording in seconds
        """
        for _ in range(int(self.stream_params.rate * duration / self.stream_params.frames_per_buffer)):
            audio_data = self._stream.read(self.stream_params.frames_per_buffer)
            self._wav_file.writeframes(audio_data)



    def _close_recording_resources(self) -> None:
        """Closes the resources used to record audio from the microphone and
        save it to a wav file.
        """
        self._wav_file.close()
        self._stream.close()
        self._pyaudio.terminate()



    def transcribe(self, save_path:str, duration:int) -> str:
        """Records audio from the microphone and transcribes it to text.
        Args:
            - save_path: Path to the wav file to be transcribed
            - duration: Duration of the recording in seconds
        Returns:
            - str: Transcribed text
        """
        print("Start speaking...")
        # Create resources to record audio from the microphone and
        # save it to a wav file
        self._create_recording_resources(save_path)
        self._write_wav_file_reading_from_stream(duration)
        self._close_recording_resources()
        print("Stop speaking")
        print("\n")
        print('_' * 50 )
        print("\n")

        # Transcribe the recorded audio
        r = sr.Recognizer()
        with sr.AudioFile(save_path) as source:
            audio = r.record(source)
        os.remove("audio.wav") 
        return r.recognize_google(audio).lower()
    
    

    def print_say(self, text):
        """Prints the text and says it using the text-to-speech engine.
        Args:
            - text: Text to be printed and said
        """
        print("SPOT:", text)
        self.engine.say(text)
        self.engine.runAndWait()



    def remap_answer(self,answer):
        """Remaps the answer to the most common ways of saying yes or no.
        Args:
            - answer: Answer to be remapped
        Returns:
            - str: Remapped answer
        """
        if answer.lower() in self.remap:
            return self.remap[answer]
        else:
            return answer


    
    def conversation_get_mission(self):
        """Conversation to figure out which item to get"""

        conv_complete = False

        # Start the conversation
        print('\n' * 30)
        self.print_say("Hello, I am SPOT you assistance dog, do you need anything?")

        # Transcribe the answer
        answer = self.transcribe(duration=5, save_path="audio.wav")
        self.feedback.current_question = "1/3  done"
        self.server.publish_feedback(self.feedback) 


        while answer is not None:
            # Use remapping to also include the most common ways of saying yes or no
            answer = self.remap_answer(answer)

            # If the answer is yes, ask which item is needed
            if "yes" in answer:
                self.print_say( "Which item do you need?" )
                answer = self.transcribe(duration=5, save_path="audio.wav")
                self.feedback.current_question = "2/3  done"
                self.server.publish_feedback(self.feedback) 

                # If the answer is given, ask for confirmation
                if answer is not None:
                    self.print_say(f"You said you need {answer}, is that correct?")
                    confirm = self.transcribe(duration=5, save_path="audio.wav")
                    self.feedback.current_question = "3/3  done"
                    self.server.publish_feedback(self.feedback)

                    while confirm is not None:
                    # Use self.remapping to also include the most common ways of saying yes or no
                        confirm = self.remap_answer(confirm)

                        # If the answer is yes, start searching
                        if "yes" in confirm:
                            self.print_say( f"Great, I will start searching {answer} for you.")
                            conv_complete = True
                            
                            # Set the result and set the action server to succeeded
                            self.result = ConversationResult()
                            self.result.answer = str(answer)
                            self.server.set_succeeded(self.result) 
                        
                            break
                        # If the answer is no, ask again
                        elif "no" in confirm:
                            self.print_say( f"I'm sorry, let's try again.")
                            
                            break
                        # If the answer is not yes or no, ask again
                        else:
                            self.print_say( "I didn't understand your response.")
                            confirm = self.transcribe(duration=5, save_path="audio.wav")
                    
                    # If the answer is not yes or no, ask again
                    if confirm is None:
                        self.print_say( f"I'm sorry, I didn't hear your response.")
                        break
                # If the answer is not given, ask again
                else:
                    self.print_say(f"I'm sorry, I didn't hear your response.")
                    break

            # If the answer is no, end the conversation
            elif "no" in answer:
                self.print_say( f"Okay, have a good day!")
                break
            # If the answer is not yes or no, ask again
            else:
                self.print_say( "I didn't understand your response.")
                answer = self.transcribe(duration=5, save_path="audio.wav")
            # If the answer is not given, ask again
            if answer is None:
                self.print_say( f"I'm sorry, I didn't hear your response.")
                break
            # If the conversation is complete, end the conversation
            if conv_complete:
                break
        


    def conversation_finish_mission(self):
        """Conversation to figure out if the item is the correct one
        and to deliver the item to the person"""

        # Start the conversation
        self.print_say("Here is the item. Is this the item you wanted?")
        answer = self.transcribe(duration=5, save_path="audio.wav")

        # Analyze confirmation
        while answer is not None:
            # Use remapping to include the most common ways of saying yes or no
            answer = self.remap_answer(answer)

            # If the answer is yes, deliver the item
            if "yes" in answer:
                self.print_say("There you go!")

                self.result = ConversationResult()
                self.server.set_succeeded(self.result) 
                break

            # If the answer is no, ask which item is wanted instead
            elif "no" in answer:
                self.print_say("Which item would you have wanted instead?")
                answer = self.transcribe(duration=5, save_path="audio.wav")

                # If the answer is given, ask for confirmation
                if answer is not None:
                    self.print_say(f"You said you wanted {answer}, is that correct?")
                    confirm = self.transcribe(duration=5, save_path="audio.wav")

                    
                    while confirm is not None:
                        # Use remapping to include the most common ways of saying yes or no
                        confirm = self.remap_answer(confirm)

                        # If the answer is yes, search the new item
                        if "yes" in confirm:
                            self.print_say(f"Great, I will get you {answer} instead.")
                            item_delivered = True
                            break

                        # If the answer is no, ask again
                        elif "no" in confirm:
                            self.print_say("I'm sorry, let's try again.")
                            break
                        # If the answer is not yes or no, ask again
                        else:
                            self.print_say("I didn't understand your response.")
                            confirm = self.transcribe(duration=5, save_path="audio.wav")

                    # If the answer is not yes or no, ask again
                    if confirm is None:
                        self.print_say("I'm sorry, I didn't hear your response.")
                        break
                # If the answer is not given, ask again
                else:
                    self.print_say("I'm sorry, I didn't hear your response.")
                    break
            # If the answer is not yes or no, ask again
            else:
                self.print_say("I didn't understand your response.")
                answer = self.transcribe(duration=5, save_path="audio.wav")

            # If the answer is not given, ask again
            if answer is None:
                self.print_say("I'm sorry, I didn't hear your response.")
                break





    def run_conv(self, goal):
        """Run the conversation server"""

        # If the goal is to give a mission, run the conversation
        if goal.conv_type == "give_mission":
            self.conversation_get_mission()
        
        # If the goal is to finish a mission, run the conversation
        elif goal.conv_type == "finish_mission":
            self.conversation_finish_mission()

        else:
            rospy.loginfo("No proper goal was specified for Conversation server")

        



if __name__ == "__main__":
    """Main function to start the conversation server"""


    # Parse arguments
    parser = argparse.ArgumentParser(description="Record audio from microphone")
    parser.add_argument("-d", "--debug", action="store_true", default=False, help="Display debug information")
    args, unknown = parser.parse_known_args()
    debug = rospy.get_param("/debug")

    # If debug is not set, redirect the error output to /dev/null
    if not debug:
        # Disable warnings
        import warnings
        warnings.filterwarnings("ignore")
        # Redirect the error output to /dev/null
        os.close(2)
        os.open(os.devnull, os.O_RDWR)
    
    # Start the conversation server
    try:
        stream_params = StreamParams()
        rospy.init_node('conversation_server')
        server = ConversationServer(stream_params)

        rospy.spin()

    # If the node is interrupted, warn the user
    except rospy.ROSInterruptException:
        rospy.logwarn("The node speech_recognition could not be launched")
        pass


