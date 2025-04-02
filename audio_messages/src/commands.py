import speech_recognition as sr
import pyttsx3
import pyaudio
import random

class Commands:

    def __init__(self):
        self.device = pyaudio.PyAudio()
        # initialising a recogniser
        self.r = sr.Recognizer()

    # Convert Text to Speech
    def SpeakText(self, command):
        engine = pyttsx3.init()
        engine.say(command)
        # The say function does not work without a run and wait command
        engine.runAndWait()

    ## SPECIFIC COMMAND FUNCTONS
    
    def modeChange(self, mode):
        match mode:
            case 0: # Greet Guests
                print("Greeting Guests")
                self.SpeakText("going to door to greet guests")
            case 1: # Wander Around
                print("Wander Command Recognised")
            case 2: # Go to Charging Port
                print("Charging Command Recognised")
            case 3: # Dance Mode
                print("Dancey Dancey")

    def sendDrinkOrder(self, drinks, table):
        for i in range(3):
            print("Drink of type " + str(i) + " ordering " + str(drinks[i]))
    
    def artWorkInfo(self):
        # Read closest artwork from publisher
        print("Artwork Info Command Registered")
    
    def tableStatus(self):
        print("Table Status Command Registered")

    def callWaiter(self, location):
        print("Call Waiter Command Recognised")

    def goToTable(self, table):
        print("Go To Table Command Recognised")

    def funFact(self):
        # factList = [""]
        fact = random.randint(0,9)
        print("Fun Fact " + str(fact))
        self.SpeakText("fun fact will be said")


# comms = Commands()