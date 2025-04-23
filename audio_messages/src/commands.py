import speech_recognition as sr
import pyttsx3
import pyaudio
import random
import time
from PIL import Image, ImageTk, ImageDraw, ImageFont
import tkinter as tk
from tableOrganisation import TableDatabase
from screeninfo import get_monitors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

DURATION = 2000
WPM = 160

class Commands:

    def __init__(self, node):
        self.node = node # saved for reference

        self.device = pyaudio.PyAudio()
        # initialising a recogniser
        self.r = sr.Recognizer()
        self.engine = pyttsx3.init()
        # self.testingVoices()
        self.engine.setProperty('rate',WPM)
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[19].id) 

        ## ROS Topics and stuff
        self.subscriber_currentPos = node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.service_artIdentification
        # self.publisher_drinkOrders
        # self.publisher_movementMode
        # self.publisher_location

        self.publisher_location = node.goal_pub

        # monitor = get_monitors()[1] # this will allow us to put it on the second screen
        monitors = get_monitors()
        if len(monitors) > 1:
            monitor = monitors[1]
        else:
            monitor = monitors[0]
        self.root = tk.Tk()

        ## Data needed to make images halfscreen
        # self.screen_width = self.root.winfo_screenwidth()
        # self.screen_height = self.root.winfo_screenheight()
        # # Half size
        # self.half_width = self.screen_width // 2
        # self.half_height = self.screen_height // 2
        # self.root.geometry(f"{self.half_width}x{self.half_height}")
        
        # Establishing the root used
        self.root.geometry(f"{monitor.width}x{monitor.height}+{monitor.x}+{monitor.y}") # attaching the root to the second screen
        self.root.attributes('-fullscreen', True)
        self.root.bind("<Escape>", lambda e: self.root.destroy())
        
        ## Establishing all the images
        self.loadInfo()

        self.tk_default_image = ImageTk.PhotoImage(self.default_image)
        self.label = tk.Label(self.root, image=self.tk_default_image)
        self.label.pack(expand=True)    
        self.fullScreenImage(self.default_image,10) 
        # self.runGUI()   

################################################################### Base Functionality Code #########################################################################
    def getNumofTables(self):
        return self.tables.getNumofTables()
    
    def speakingTimeEst(self, text):
        words = len(text.split())
        wps = WPM / 60
        return int(((words / wps)+0.25) * 1000)
        
    def loadInfo(self):
        # Robot Faces
        self.default_image = Image.open("audio_messages/src/resources/default.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.dancingFace = Image.open("audio_messages/src/resources/danceFace.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.chargeFace = Image.open("audio_messages/src/resources/chargeFace.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.callingFace = Image.open("audio_messages/src/resources/callingWaiter.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)

        # Information about Artwork        
        with open("audio_messages/src/resources/ArtworkInfo/artworkInfo.txt", "r") as artFile:
            self.artworks = artFile.readline().split("/")
            self.artInfo = artFile.readlines()
        self.artImages = {a: Image.open(f"audio_messages/src/resources/ArtworkInfo/{self.artworks[a].strip()}_info.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
                           for a in range(5)}

        # Fun Facts
        with open("audio_messages/src/resources/FunFacts/funfacts.txt", "r") as factFile:
            self.facts = factFile.readlines()
        self.factImages = {i: Image.open(f"audio_messages/src/resources/FunFacts/image{i}.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
                           for i in range(10)}
        
        # Table Database
        self.tables = TableDatabase("audio_messages/src/resources/tableInfo.txt")

    def runGUI(self):
        self.root.mainloop()

    def fullScreenImage(self, newImage, duration=4000, manualReset = False):
        # print(duration)
        tk_image = ImageTk.PhotoImage(newImage)
        self.label.configure(image=tk_image)
        self.label.image = tk_image
        if manualReset == False:
            self.root.after(duration, self.resetToDefault)

    def resetToDefault(self):
        # print("reseting to default")
        self.label.configure(image=self.tk_default_image)
        self.label.image = self.tk_default_image

    # Convert Text to Speech
    def SpeakText(self, command):
        self.engine.say(command)
        # The say function does not work without a run and wait command
        self.engine.runAndWait()
        time.sleep(0.25)

#################################################################### ROS CALLBACKS ################################################################################
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        print(f"Position: x = {position.x}, y = {position.y}, z = {position.z}")
        print(f"Orientation: x = {orientation.x}, y = {orientation.y}, z = {orientation.z}, w = {orientation.w}")
        self.currentPos = [[position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w]]
############################################################## SPECIFIC COMMAND FUNCTONS ##########################################################################
    
    def modeChange(self, mode):
        match mode:
            case 0: # Greet Guests
                print("Greeting Guests Command Recognised")
                self.SpeakText("going to door to greet guests")

            case 1: # Wander Around
                print("Wander Command Recognised")
                self.SpeakText("beginning wander mode")

            case 2: # Go to Charging Port
                print("Charging Command Recognised")
                self.fullScreenImage(self.chargeFace,DURATION)
                self.SpeakText("returning to charging port")

            case 3: # Dance Mode
                print("dance mode activated")
                self.fullScreenImage(self.dancingFace,DURATION)
                self.SpeakText("Dancey Dancey")

    def sendDrinkOrder(self, drinks, table):
        for i in range(3):
            print("Ordering " + str(drinks[i][1]) + " " + str(drinks[i][0]) + "s")
    
    def artWorkInfo(self):
        # Read closest artwork from publisher
        print("Artwork Info Command Registered")
        # print(f"1: {self.artworks[0]}\n2: {self.artworks[1]}\n3: {self.artworks[2]}\n4: {self.artworks[3]}\n5: {self.artworks[4]}")
        artwork = int(input("Artwork: ")) # This will be changed to recieve the name of the painting from the visual subsystem
        if artwork == self.artworks[0] or artwork == 1: # The Ugly Duchess
                self.fullScreenImage(self.artImages[0], (self.speakingTimeEst(self.artInfo[0]) + DURATION))
                self.SpeakText(self.artInfo[0].lower())
        elif artwork == self.artworks[1] or artwork == 2: # Composition of Red Yellow and Blue
                self.fullScreenImage(self.artImages[1], (self.speakingTimeEst(self.artInfo[1]) + DURATION))
                self.SpeakText(self.artInfo[1].lower())
        elif artwork == self.artworks[2] or artwork == 3: # Scene from Moby Dick
                self.fullScreenImage(self.artImages[2], (self.speakingTimeEst(self.artInfo[2]) + DURATION))
                self.SpeakText(self.artInfo[2].lower())
        elif artwork == self.artworks[3] or artwork == 4: # Flowers in Four Seasons
                self.fullScreenImage(self.artImages[3], (self.speakingTimeEst(self.artInfo[3]) + DURATION))
                self.SpeakText(self.artInfo[3].lower())
        elif artwork == self.artworks[4] or artwork == 5: # The Persistence of Memory
                self.fullScreenImage(self.artImages[4], (self.speakingTimeEst(self.artInfo[4]) + DURATION))
                self.SpeakText(self.artInfo[4].lower())
                
    def tableStatus(self):
        print("Table Status Command Registered")
        statusImage = self.tables.generateTableStatusImage(self.half_width, self.half_height)
        self.fullScreenImage(statusImage, DURATION)

    def callWaiter(self, location):
        table = self.tables.getClosestTable(location)
        print(f"Call Waiter Command Recognised, calling to table {table}")
        self.fullScreenImage(self.callingFace)
        self.SpeakText("calling a waiter here")

    def goToTable(self, table):
        tableLocation = self.tables.findTableLocation(table)
        self.SpeakText(f"follow me to table {str(table)}")
        print(f"Go To Table Command Recognised, Table Location is: {tableLocation}")
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg() 
        goal.pose.position.x = tableLocation[0]
        goal.pose.position.y = tableLocation[1]
        goal.pose.orientation.w = 1.0
        self.publisher_location.publish(goal)


    def funFact(self):
        factNum = random.randint(0,9)
        # Outputting the fact image and audio
        print("Fun Fact " + self.facts[factNum])
        self.fullScreenImage(self.factImages[factNum],(self.speakingTimeEst(self.facts[factNum]) + DURATION))
        self.SpeakText(self.facts[factNum])
        
        


# comms = Commands()