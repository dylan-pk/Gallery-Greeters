import speech_recognition as sr
import pyttsx3
import pyaudio
import random
import time
from PIL import Image, ImageTk
import tkinter as tk

class Commands:

    def __init__(self):
        self.device = pyaudio.PyAudio()
        # initialising a recogniser
        self.r = sr.Recognizer()
        self.engine = pyttsx3.init()

        self.root = tk.Tk()

        ## Data needed to make images halfscreen
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        # Half size
        half_width = screen_width // 2
        half_height = screen_height // 2

        self.root.geometry(f"{half_width}x{half_height}")
        self.root.bind("<Escape>", lambda e: self.root.destroy())

        default_image_path = "src/resources/default.png"
        default_image = Image.open(default_image_path)
        self.tk_default_image = ImageTk.PhotoImage(default_image)
        # image.show()

        self.label = tk.Label(self.root, image=self.tk_default_image)
        # self.label.image = self.default_image
        self.label.pack(expand=True)        

        with open("src/resources/funfacts.txt", "r") as file:
            self.facts = file.readlines()

    def fullScreenImage(self, newImage, duration=4000):
        tk_image = ImageTk.PhotoImage(newImage)
        self.label.configure(image=tk_image)
        # self.label.image = newImage
        self.root.after(duration, self.resetToDefault)

    def resetToDefault(self):
        self.label.configure(image=self.tk_default_image)
        # self.label.image = self.default_image

    # Convert Text to Speech
    def SpeakText(self, command):
        self.engine.say(command)
        # The say function does not work without a run and wait command
        self.engine.runAndWait()

    ## SPECIFIC COMMAND FUNCTONS
    
    def modeChange(self, mode):
        match mode:
            case 0: # Greet Guests
                print("Greeting Guests Command Recognised")
                self.SpeakText("going to door to greet guests")
            case 1: # Wander Around
                self.SpeakText("beginning wander mode")
                print("Wander Command Recognised")
            case 2: # Go to Charging Port
                self.SpeakText("returning to charging port")
                print("Charging Command Recognised")
            case 3: # Dance Mode
                self.SpeakText("Dancey Dancey")
                print("dance mode activated")

    def sendDrinkOrder(self, drinks, table):
        for i in range(3):
            print("Ordering " + str(drinks[i][1]) + " " + str(drinks[i][0]) + "s")
    
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
        factNum = random.randint(0,9)

        # Setting up the image
        image_path = f"src/resources/image{factNum}.png"
        image = Image.open(image_path)

        # Outputting the fact image and audio
        print("Fun Fact " + self.facts[factNum])
        # image.show()
        self.SpeakText(self.facts[factNum].lower())
        


# comms = Commands()