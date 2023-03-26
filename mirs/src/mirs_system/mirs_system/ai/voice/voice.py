# Importing neccessary modules
import datetime
import speech_recognition as sr
import pyttsx3 as ts
import os


class Voice:
    def __init__(self, voice_male=True):
        self.engine = ts.init('sapi5')
        self.recognizer = sr.Recognizer()
        self.START = True

        # Initializing voice for Bot
        self.voices = self.engine.getProperty('voices')

        # print(voices[0])
        if (voice_male):
            self.engine.setProperty('voice', self.voices[0].id)
        else:
            self.engine.setProperty('voice', self.voices[1].id)
        self.pause = None

        self.wish()

    def speak(self, audio):
        self.engine.say(audio)
        self.engine.runAndWait()

    def wish(self):
        hour = int(datetime.datetime.now().hour)
        if (hour < 12):
            self.speak("Hi Sir, Good Morning")
        elif (hour >= 12 and hour < 16):
            self.speak("Hi Sir, Good Afternoon")
        elif (hour >= 16 and hour < 19):
            self.speak("Hi Sir, Good Evening")
        else:
            self.speak("Hi Sir, Good Night")

    def listen(self):
        # It takes microphone input from the user and returns string output
        self.speak('Listening sir')
        with sr.Microphone() as source:
            print("Listening...")
            audio = self.recognizer.listen(source, 10)
            print("Recognizing...")

            DIR_NAME = os.path.dirname(os.path.realpath(__file__))
            TENSOR_GRAPH = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            TENSOR_LABELS = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            # command = self.recognizer.recognize_tensorflow(
            #     audio, tensor_graph=TENSOR_GRAPH, tensor_label=TENSOR_LABELS)
            command = self.recognizer.recognize_google(audio)
        return command
