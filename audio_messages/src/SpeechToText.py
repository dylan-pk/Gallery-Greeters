import speech_recognition as sr
import pyttsx3
import pyaudio
from commands import Commands
from std_msgs.msg import Int32, Bool
# from tableOrganisation import TableDatabase
from PIL import Image
import pvporcupine as porcu
import sounddevice as sd
import struct
from example_interfaces.srv import SetBool

import threading
from rclpy.node import Node
import rclpy
import queue
from rclpy.executors import MultiThreadedExecutor
import time
import tkinter as tk

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# spoken = False

audio_to_main_queue = queue.Queue()
main_to_gui_queue = queue.Queue()

TESTING_MODE = True
ACCESS_KEY = "UNuftCmjek2mefFH8OZiwT0LiaeSZJBcFdo1GaqCGcEiKTGQfR7vYQ=="# "8HEM095Qo29k5b/OQ01LPFlr+FfiUHVRi0k1N1rYnUQ2ZvZuig2zdA==" # -Anika
READY_FOR_COMMAND = False

## This class is all about processing the audio for getting the initial commands and then also for any commands that require multiple prompts ##
class SpeechToText(Node):
    numbers_words = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10}

    def __init__(self, audio_queue, gui_queue):
        super().__init__('speech_to_text')
        self.audio_queue = audio_queue
        # initialising a recogniser
        self.r = sr.Recognizer()
        # creating pyaudio object so debugging
        self.device = pyaudio.PyAudio()
        # print(self.device.get_default_input_device_info())
        self.deviceNum = self.device.get_default_input_device_info()["index"]

        self.listening_event = threading.Event()

        self.availabledrinks = Image.open("audio_messages/src/resources/available_drinks.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)

        self.client_artIdentification = self.create_client(SetBool, 'perform_template_matching')
        self.artReq = SetBool.Request()

        self.goal_pub = self.create_publisher(PoseStamped, '/pose_topic', 10)
        self.mode_pub = self.create_publisher(Int32, '/robot_mode', 10)
        self.interrupt_pub = self.create_publisher(Bool,"/interrupt_signal", 10)
        self.publisher_path = self.create_publisher(Path, '/sentry_path', 10)
        self.comms = Commands(self, gui_queue)
        self.numOfTables = self.comms.getNumofTables()
        

    def startVoiceRecognition(self):
        print("voice recognition Thread started")
        audioThread = threading.Thread(target=self.commandAudioRecordingLoop, daemon=True)
        audioThread.start()
    
    def setListenEvent(self):
        self.interrupt_pub.publish(Bool(data=True))
        print("listening event set")
        self.retry_count = 0
        self.listening_event.set()
    
    def commandAudioRecordingLoop(self):
        while True:
            # print(f"loop of audio recording, listening enabled is {self.listeningEnabled}")
            self.listening_event.wait() # using a threading event so if multi prompt stuff is happening then the command audio loop doesn't get confused
            while self.listening_event.is_set():
                if TESTING_MODE == False:
                    recordedText = self.audioRecording(self.deviceNum, "Run", 1)
                    if recordedText != None:
                        threading.Thread(target=self.processText, args=(recordedText,), daemon=True).start()
                else:
                    threading.Thread(target=self.processText, args=("",), kwargs={"testing": True}, daemon=True).start()

    def processText(self, text, testing = False):
        # print(f"processing, listening is disabled")
        if testing == False:
            words = text.split()
            for word in words:
                match word:
                    case "greeting":
                        self.listening_event.clear()
                        self.comms.modeChange(0) # Waiting for how to send data
                    case "drink":
                        self.listening_event.clear()
                        self.comms.pushToQueue(self.availabledrinks, 0, True)
                        self.getDrinkOrder()
                    case "art":
                        self.listening_event.clear()
                        self.artWorkInfo()
                        # self.comms.artWorkInfo() # COMMAND CODE DONE
                    case "walk":
                        self.listening_event.clear()
                        self.comms.modeChange(1) # Waiting for how to send data
                    case "charge":
                        self.listening_event.clear()
                        self.comms.modeChange(3) # Face Change / Waiting for how to send data
                    case "table":
                        self.listening_event.clear()
                        self.tableCommands(words)
                    case "waiter":
                        self.listening_event.clear()
                        currentPosition = [0,0,0] # Get currentPosition value and pass it in
                        self.comms.callWaiter(currentPosition) # Face Change / Waiting for how to send data
                    case "fact":
                        self.listening_event.clear()
                        self.comms.funFact() # COMMAND CODE DONE
                    case "dance":
                        self.listening_event.clear()
                        self.comms.modeChange(2) # Waiting for how to send data
                    case _:
                        # print("Not a command word")
                        pass
        else:
            self.listening_event.clear()
            print("1: Greet Guests\n2: Order a Drink\n3: Tell me about the art\n4: Wander Around\n"
            "5: Do a Dance\n6: Go to a Table\n7: Get Table Status\n8: Call a waiter\n9: Tell me a Fun Fact\n10: Go to Charging Station")
            commandNum = int(input("Which Command: "))
            match commandNum:
                    case 1:
                        self.listening_event.clear()
                        self.comms.modeChange(0) # Waiting for how to send data
                    case 2:
                        self.listening_event.clear()
                        self.comms.pushToQueue(self.availabledrinks, 0, True)
                        self.getDrinkOrder()
                    case 3:
                        self.listening_event.clear()
                        self.comms.artWorkInfo() # COMMAND CODE DONE
                    case 4:
                        self.listening_event.clear()
                        self.comms.modeChange(1) # Waiting for how to send data
                    case 5:
                        self.listening_event.clear()
                        self.comms.modeChange(2) # Face Change / Waiting for how to send data
                    case 6:
                        self.listening_event.clear()
                        table = self.getTable()
                        self.comms.goToTable(table)
                    case 7:
                        self.listening_event.clear()
                        self.comms.tableStatus()
                    case 8:
                        self.listening_event.clear()
                        currentPosition = [0,0,0] # Get currentPosition value and pass it in
                        self.comms.callWaiter(currentPosition) # Face Change / Waiting for how to send data
                    case 9:
                        self.listening_event.clear()
                        self.comms.funFact() # COMMAND CODE DONE
                    case 10:
                        self.listening_event.clear()
                        self.comms.modeChange(3) # Waiting for how to send data
                    case _:
                        # print("Not a command word")
                        pass
        
        # print(f"at the end of process text listening remains disabled until next wake word")

    
    def tableCommands(self, sentence):
        print("In table commands function")
        go_to_table = True
        for word in sentence:
            if word == "status" or word == "tell" or word == "show":
                go_to_table = False
        if go_to_table:
            table = self.getTable()
            self.comms.modeChange(4)
            self.comms.goToTable(table) # Waiting for how to send data
        else:
            self.comms.tableStatus()
    
    def getDrinkOrder(self):
        # print(f"Drink Order Command")
        drinks = [["Carlton",0],["Soda", 0],["Champagne", 0]]
        table = 0
        ordering = True
        self.comms
        while True:
            # self.listeningEnabled = False
            while ordering:
                self.comms.SpeakText("what drink would you like")
                ogText = self.audioRecording(self.deviceNum, "Drink Order", 1)
                drinkRegistered = False
                if ogText != None:
                    textSplit = ogText.split()
                    ordering = False
                
            for text in textSplit:
                match text:
                    case "carlton":
                        drinkIdx = 0
                        drinkRegistered = True
                        break
                    case "soft":
                        drinkIdx = 1
                        drinkRegistered = True
                        break
                    case "champagne":
                        drinkIdx = 2
                        drinkRegistered = True
                        break
                        
                    case _:
                        drinkRegistered = False

            if drinkRegistered == False:
                self.comms.SpeakText("we do not have that drink sorry")
            else:
                # amount = self.drinksAmount()
                self.comms.SpeakText("How many would you like")
                amount = self.audioRecording(self.deviceNum, "Drink Order, Amount", 2)
                # while amountregistered == False:
                for word in amount.split():
                    if word in self.numbers_words:
                        number = self.numbers_words[word]
                    elif word.isdigit():
                        number = word
                    else:
                        pass
                
                drinks[drinkIdx][1] = number

            self.comms.SpeakText("would you like another drink?")
            response = self.audioRecording(self.deviceNum,"Drink Order", 3)
            affirmative = False
            for word in response.split():
                if word == "yes" or word == "yeh":
                    affirmative = True
                    break
            
            if affirmative:
                ordering = True
            else:
                break
                
        
        self.comms.queue.put(lambda: self.comms.resetToDefault())
        self.comms.SpeakText("Drink Order Sent")
        self.comms.sendDrinkOrder(drinks, table)
        print(f"at the end of drink order listening is disabled")
    
    def drinksAmount(self):
        # amountregistered = False
        self.comms.SpeakText("How many would you like")
        amount = self.audioRecording(self.deviceNum, "Drink Order, Carlton", 2)
        # while amountregistered == False:
        for word in amount.split():
            if word in self.numbers_words:
                number = self.numbers_words[word]
            elif word.isdigit():
                number = word
            else:
                pass
        return number
       
    def artWorkInfo(self):
        print("we got in here")
        self.comms.SpeakText("which artwork would you like to know about")
        artwork = self.audioRecording(self.deviceNum, "Art Request", 1)
        actualArt = False
        for word in artwork:
            for art in self.comms.getArtworkNames():
                if word == art:
                    requestedArtwork = artwork
                    actualArt = True
                
        if actualArt:
            # go to the artwork then scan
            pass
        else:
            # spin in a circle till you see artwork
            pass
        
        self.artReq.data = True
        self.future = self.client_artIdentification.call_async(self.artReq)
        rclpy.spin_until_future_complete(self, self.future)
        artResponse = self.future.result()
        print(f"the response was {artResponse}")
        if artResponse.success:
            self.comms.artWorkInfo(artResponse.message)

    def getTable(self):
        self.listening_event.clear()
        self.comms.modeChange(4)
        print("In get Table")
        self.comms.SpeakText("which table should I go to?")
        if TESTING_MODE:
            response = int(input("Enter a table number: "))
            return response
        else:
            response = self.audioRecording(self.deviceNum, "GetTable", 1)
            if response != None:
                response_words = response.split()
                for word in response_words:
                    if word.isdigit():
                        if int(word) < self.numOfTables:
                            return int(word)
                
    def audioRecording(self, device, function, promptNum):            
            with sr.Microphone(device_index=device) as source2:
                self.r.adjust_for_ambient_noise(source2, duration=0.5)  # Adjust to background noise
                print("Listening...")

                # Continuously listen for speech and stop when silence is detected
                while True:
                    if function == "Run" and self.listening_event.is_set() == False:
                        self.listening_event.wait()
                    else:
                        audio2 = self.r.listen(source2)
                        try:
                            MyText = self.r.recognize_google(audio2).lower()
                            # print(f"Detected speech: {MyText}")

                            if MyText != None:  # If text is not empty
                                print(function + " prompt #" + str(promptNum) + " response is: ", MyText)
                                return MyText  # Stop recording and return text


                        except sr.RequestError as e:
                            print("Could not request results; {0}".format(e))

                        except sr.UnknownValueError:
                            # print("unknown error occurred")
                            print("Silence detected, stopping recording...")
                            self.retry_count += 1
                            print(f"amount of retrys is {self.retry_count}")
                            if self.retry_count >= 10:
                                self.listening_event.clear()
                                return None
                            else:
                                retry = self.audioRecording(device,function,promptNum)
                                if retry != None:
                                    return retry
                                else:
                                    return None  # Stop if no speech is recognized

class DebugServiceServer(Node):
    def __init__(self):
        super().__init__('debug_service_server')
        self.create_service(SetBool, 'perform_template_matching', self.handle_template_matching)

    def handle_template_matching(self, request, response):
        self.get_logger().info(f'Received request: {request}')
        response.success = True
        response.message = "Scene from Moby Dick"
        # "Scene from Moby Dick"  # Send dummy response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToText(audio_to_main_queue, main_to_gui_queue)
    # test_node = DebugServiceServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # executor.add_node(test_node)

    node.startVoiceRecognition()
    # rclpy.spin(test_node)

    wakeDetection = porcu.create(access_key=ACCESS_KEY, keywords=['jarvis'])
    wakeAudioStream = sd.RawInputStream(samplerate=wakeDetection.sample_rate, blocksize=wakeDetection.frame_length, dtype='int16', channels=1)
    wakeAudioStream.start()
    READY_FOR_COMMAND = False
    try:
        while True:
            pcm = wakeAudioStream.read(wakeDetection.frame_length)[0]
            pcm = struct.unpack_from("h" * wakeDetection.frame_length, pcm)
            keyword_index = wakeDetection.process(pcm)
            if keyword_index >= 0:
                # Send ROS Message to stop movement
                READY_FOR_COMMAND = True
                print("wake word detected")

            executor.spin_once(timeout_sec=0.01)
            if READY_FOR_COMMAND:
                node.setListenEvent()
                READY_FOR_COMMAND = False

            node.comms.root.update_idletasks()
            node.comms.root.update()

            try:
                text = audio_to_main_queue.get_nowait()
                node.processText(text)
            except queue.Empty:
                pass

            time.sleep(0.01)
    except tk.TclError:
        print("Tkinter GUI Closed")
    finally:
        executor.shutdown()
        # test_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
    # stt = SpeechToText()
    # stt.comms.root.mainloop()
    # stt.run()