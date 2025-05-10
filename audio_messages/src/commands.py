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
import threading
from std_msgs.msg import Int32

DURATION = 2000
WPM = 160
GREETLOCATION = [1.0,1.0]
CHARGINGLOCATION = [2.0,2.0]

class Commands:

    def __init__(self, node, gui_queue):
        self.node = node # saved for reference
        self.queue = gui_queue

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
        self.publisher_movementMode = node.mode_pub
        self.publisher_location = node.goal_pub

        # monitor = get_monitors()[1] # this will allow us to put it on the second screen
        self.display_event = threading.Event()
        monitors = get_monitors()
        self.root = tk.Tk()
        # print(moSnitors)
        if len(monitors) > 1:
            maxx = 0
            for i, mon in enumerate(monitors):
                if mon.x != 0 and mon.x > maxx:
                    maxx = mon.x
                    self.monitor = monitors[i]
            
            self.screenWidth, self.screenHeight = self.monitor.width, self.monitor.height
            self.root.geometry(f"{self.monitor.width}x{self.monitor.height}+{self.monitor.x}+{self.monitor.y}") # attaching the root to the second screen
            self.root.attributes('-fullscreen', True)
            self.root.bind("<Escape>", lambda e: self.root.destroy())
        else:
            # Data needed to make images halfscreen
            self.screen_width = self.root.winfo_screenwidth()
            self.screen_height = self.root.winfo_screenheight()
            # Half size
            self.half_width = self.screen_width // 2
            self.screenWidth= self.half_width
            self.half_height = self.screen_height // 2
            self.screenHeight= self.half_height
            self.root.geometry(f"{self.half_width}x{self.half_height}")
        
        # Establishing the root used
        self.root.after(100, self.process_gui_queue)
        
        ## Establishing all the images
        self.loadInfo()

        self.tk_default_image = ImageTk.PhotoImage(self.default_image)
        self.label = tk.Label(self.root, image=self.tk_default_image)
        self.label.pack(expand=True)    
        self.fullScreenImage(self.default_image,10) 
        # self.runGUI()   

################################################################### Base Functionality Code #########################################################################
    def process_gui_queue(self):
        while not self.queue.empty():
            try:
                func = self.queue.get()
                func()
            except Exception as e:
                print(f"GUI Error: {e}")
        self.root.after(100, self.process_gui_queue)

    def pushToQueue(self, file, duration, manualReset=False):
        self.display_event.clear()
        # print(f"display event cleared so it's {self.display_event.is_set()}")
        self.queue.empty()
        self.queue.put(lambda: self.fullScreenImage(file, duration, manualReset))

    def getNumofTables(self):
        return self.tables.getNumofTables()
    
    def speakingTimeEst(self, text):
        words = len(text.split())
        wps = WPM / 60
        return int(((words / wps)+0.25) * 1000)
        
    def loadInfo(self):
        # Robot Faces
        self.default_image = Image.open("src/resources/default.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.dancingFace = Image.open("src/resources/danceFace.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.chargeFace = Image.open("src/resources/chargeFace.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
        self.callingFace = Image.open("src/resources/callingWaiter.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)

        # Information about Artwork        
        with open("src/resources/ArtworkInfo/artworkInfo.txt", "r") as artFile:
            self.artworks = artFile.readline().split("/")
            self.artInfo = artFile.readlines()
        self.artImages = {a: Image.open(f"src/resources/ArtworkInfo/{self.artworks[a].strip()}_info.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
                           for a in range(5)}

        # Fun Facts
        with open("src/resources/FunFacts/funfacts.txt", "r") as factFile:
            self.facts = factFile.readlines()
        self.factImages = {i: Image.open(f"src/resources/FunFacts/image{i}.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)
                           for i in range(10)}
        
        # Table Database
        self.tables = TableDatabase("src/resources/tableInfo.txt")
    
    def establishLocations(self, location_x, location_y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg() 
        goal.pose.position.x = location_x
        goal.pose.position.y = location_y
        goal.pose.orientation.w = 1.0

        return goal

    def runGUI(self):
        try:
            while True:
                self.root.update_idletasks()
                self.root.update()
                time.sleep(0.01)
        except tk.TclError:
            print("GUI closed.")

    def fullScreenImage(self, newImage, duration=4000, manualReset= False):
        # print(duration)
        def _update_image():
            try:
                # Create the PhotoImage on the main thread
                tk_image = ImageTk.PhotoImage(newImage)

                # Store a reference to prevent garbage collection
                self.label.image = tk_image
                self.label.configure(image=tk_image)
                # print(f"display event before {self.display_event.is_set()}")
                self.display_event.set()
                # print(f"display event after {self.display_event.is_set()}")
                # Optionally reset image after duration
                if not manualReset:
                    self.root.after(duration, self.resetToDefault)
            except Exception as e:
                print(f"[fullScreenImage] Error updating image: {e}")

        # Always call GUI logic via the main thread
        self.queue.put(_update_image)

    def resetToDefault(self):
        # print("reseting to default")
        self.label.configure(image=self.tk_default_image)
        self.label.image = self.tk_default_image

    # Convert Text to Speech
    def SpeakText(self, command, wait_for_image=False):
        if wait_for_image and self.display_event is not None:
            self.display_event.wait()
        else:
            time.sleep(0.5)
        # print("speaking now")
        self.engine.say(command)
        # The say function does not work without a run and wait command
        self.engine.runAndWait()
        # self.display_event.clear()
        time.sleep(0.25)

#################################################################### ROS CALLBACKS ################################################################################
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # print(f"Position: x = {position.x}, y = {position.y}, z = {position.z}")
        # print(f"Orientation: x = {orientation.x}, y = {orientation.y}, z = {orientation.z}, w = {orientation.w}")
        self.currentPos = [[position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w]]
############################################################## SPECIFIC COMMAND FUNCTONS ##########################################################################
    
    def modeChange(self, mode):
        ros_msg = Int32()
        ros_msg.data = mode
        match mode:
            case 0: # Greet Guests
                self.publisher_movementMode.publish(ros_msg)
                print("Greeting Guests Command Recognised")
                self.SpeakText("going to door to greet guests")
                self.publisher_location.publish(self.establishLocations(GREETLOCATION[0], GREETLOCATION[1]))

            case 1: # Wander Around
                self.publisher_movementMode.publish(ros_msg)
                print("Wander Command Recognised")
                self.SpeakText("beginning sentry mode")

            case 2: # Dance Mode
                self.publisher_movementMode.publish(ros_msg)
                print("dance mode activated")
                self.pushToQueue(self.dancingFace,DURATION)
                self.SpeakText("Dancey Dancey")
            
            case 3: # Go to Charging Port
                self.publisher_movementMode.publish(ros_msg)
                print("Charging Command Recognised")
                self.pushToQueue(self.chargeFace,DURATION)
                self.SpeakText("returning to charging port")
                self.publisher_location.publish(self.establishLocations(CHARGINGLOCATION[0], CHARGINGLOCATION[1]))

    def sendDrinkOrder(self, drinks, table):
        for i in range(3):
            print("Ordering " + str(drinks[i][1]) + " " + str(drinks[i][0]) + "s")
    
    def artWorkInfo(self):
        # Read closest artwork from publisher
        print("Artwork Info Command Registered")
        # print(f"1: {self.artworks[0]}\n2: {self.artworks[1]}\n3: {self.artworks[2]}\n4: {self.artworks[3]}\n5: {self.artworks[4]}")
        # response = # Response from the service
        artwork = 1# int(input("Artwork: ")) # response.message # This will be changed to recieve the name of the painting from the visual subsystem
        # if response.success == True:
        if artwork == self.artworks[0] or artwork == 1: # The Ugly Duchess
                self.pushToQueue(self.artImages[0], (self.speakingTimeEst(self.artInfo[0]) + DURATION))
                self.SpeakText(self.artInfo[0].lower())
        elif artwork == self.artworks[1] or artwork == 2: # Composition of Red Yellow and Blue
                self.pushToQueue(self.artImages[1], (self.speakingTimeEst(self.artInfo[1]) + DURATION))
                self.SpeakText(self.artInfo[1].lower())
        elif artwork == self.artworks[2] or artwork == 3: # Scene from Moby Dick
                self.pushToQueue(self.artImages[2], (self.speakingTimeEst(self.artInfo[2]) + DURATION))
                self.SpeakText(self.artInfo[2].lower())
        elif artwork == self.artworks[3] or artwork == 4: # Flowers in Four Seasons
                self.pushToQueue(self.artImages[3], (self.speakingTimeEst(self.artInfo[3]) + DURATION))
                self.SpeakText(self.artInfo[3].lower())
        elif artwork == self.artworks[4] or artwork == 5: # The Persistence of Memory
                self.pushToQueue(self.artImages[4], (self.speakingTimeEst(self.artInfo[4]) + DURATION))
                self.SpeakText(self.artInfo[4].lower())
        # else:
        #     self.SpeakText("what image would you like to know about")



    def tableStatus(self):
        print("Table Status Command Registered")
        statusImage = self.tables.generateTableStatusImage(self.screenWidth, self.screenHeight)
        self.pushToQueue(statusImage, DURATION)

    def callWaiter(self, location):
        table = self.tables.getClosestTable(location)
        print(f"Call Waiter Command Recognised, calling to table {table}")
        self.pushToQueue(self.callingFace, DURATION)
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
        self.publisher_location.publish(self.establishLocations(tableLocation[0], tableLocation[1]))


    def funFact(self):
        factNum = random.randint(0,9)
        # Outputting the fact image and audio
        print("Fun Fact " + self.facts[factNum])
        self.pushToQueue(self.factImages[factNum], (self.speakingTimeEst(self.facts[factNum]) + DURATION))
        # self.fullScreenImage(self.factImages[factNum],(self.speakingTimeEst(self.facts[factNum]) + DURATION))

        self.SpeakText(self.facts[factNum], wait_for_image=True)
        
        


# comms = Commands()