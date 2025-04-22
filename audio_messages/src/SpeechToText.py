import speech_recognition as sr
import pyttsx3
import pyaudio
from commands import Commands
# from tableOrganisation import TableDatabase
import threading
from PIL import Image
from rclpy.node import Node
import rclpy
# spoken = False

TESTING_MODE = True

## This class is all about processing the audio for getting the initial commands and then also for any commands that require multiple prompts ##
class SpeechToText(Node):
    numbers_words = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10}

    def __init__(self):
        super().__init__('speech_to_text')
        # initialising a recnogiser
        self.r = sr.Recognizer()
        # creating pyaudio object so debugging
        self.device = pyaudio.PyAudio()
        print(self.device.get_default_input_device_info())
        self.deviceNum = self.device.get_default_input_device_info()["index"]

        self.availabledrinks = Image.open("audio_messages/src/resources/available_drinks.png")#.resize((self.half_width, self.half_height), Image.ANTIALIAS)

        self.comms = Commands(self)
        self.numOfTables = self.comms.getNumofTables()
        # imagesThread = threading.Thread(target=self.comms.runGUI, daemon=True)
        # imagesThread.start()

        self.listeningEnabled = True
        self.startVoiceRecognition()
        

    def startVoiceRecognition(self):
        audioThread = threading.Thread(target=self.commandAudioRecordingLoop, daemon=True)
        audioThread.start()
    
    def commandAudioRecordingLoop(self):
        while True:
            if self.listeningEnabled: # using a flag so if multi prompt stuff is happening then the command audio loop doesn't get confused
                if TESTING_MODE == False:
                    recordedText = self.audioRecording(self.deviceNum, "Run", 1)
                    if recordedText != None:
                        self.processText(recordedText)
                else:
                    self.processText("",testing=True)

    def processText(self, text, testing = False):
        print("processing")
        self.listeningEnabled = False
        if testing == False:
            words = text.split()
            for word in words:
                match word:
                    case "greeting":
                        self.comms.modeChange(0) # Waiting for how to send data
                    case "drink":
                        self.getDrinkOrder()
                    case "art":
                        self.comms.artWorkInfo() # COMMAND CODE DONE
                    case "walk":
                        self.comms.modeChange(1) # Waiting for how to send data
                    case "charge":
                        self.comms.modeChange(2) # Face Change / Waiting for how to send data
                    case "table":
                        table = self.getTable()
                        self.comms.goToTable(table) # Waiting for how to send data
                    case "status":
                        self.comms.tableStatus() # COMMAND CODE DONE
                    case "waiter":
                        currentPosition = [0,0,0] # Get currentPosition value and pass it in
                        self.comms.callWaiter(currentPosition) # Face Change / Waiting for how to send data
                    case "fact":
                        self.comms.funFact() # COMMAND CODE DONE
                    case "dance":
                        self.comms.modeChange(3) # Waiting for how to send data
                    case _:
                        # print("Not a command word")
                        pass
        else:
            print("1: Greet Guests\n2: Order a Drink\n3: Tell me about the art\n4: Wander Around\n"
            "5: Go to Charging Station\n6: Go to a Table\n7: Get Table Status\n8: Call a waiter\n9: Tell me a Fun Fact\n10: Do a Dance")
            commandNum = int(input("Which Command: "))
            match commandNum:
                    case 1:
                        self.comms.modeChange(0)
                    case 2:
                        self.getDrinkOrder()
                    case 3:
                        self.comms.artWorkInfo()
                    case 4:
                        self.comms.modeChange(1)
                    case 5:
                        self.comms.modeChange(2)
                    case 6:
                        table = self.getTable()
                        self.comms.goToTable(table)
                            # self.tableCommands(words)
                    case 7:
                        self.comms.tableStatus()
                    case 8:
                        currentPosition = [0,0,0] # Get currentPosition value and pass it in
                        self.comms.callWaiter(currentPosition)
                    case 9:
                        self.comms.funFact()
                    case 10:
                        self.comms.modeChange(3)
                    case _:
                        # print("Not a command word")
                        pass
        self.listeningEnabled = True

    
    # def tableCommands(self, sentence):
    #     print("In table commands function")
    #     if sentence == "table status":
    #         self.comms.tableStatus()
    #     elif sentence == "go to table":
    #         table = self.getTable()
    #         self.comms.goToTable(table)
    
    def getDrinkOrder(self):
        print("Drink Order Command")
        drinks = [["Carlton",0],["Soda", 0],["Champagne", 0]]
        table = 0
        ordering = True
        self.comms.fullScreenImage(self.availabledrinks, manualReset=True)
        while True:
            while ordering:
                self.listeningEnabled = False
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
            if response != "yes":
                break
            else:
                pass
        
        self.comms.resetToDefault()
        self.listeningEnabled = True
        self.comms.SpeakText("Drink Order Sent")
        self.comms.sendDrinkOrder(drinks, table)
    
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

    def getTable(self):
        self.listeningEnabled = False
        print("In get Table")
        self.comms.SpeakText("which table should I go to?")
        response = self.audioRecording(self.deviceNum, "GetTable", 1)
        if response != None:
            response_words = response.split()
            for word in response_words:
                if word.isdigit():
                    if int(word) < self.numOfTables:
                        self.listeningEnabled = True
                        return int(word)
                
    def audioRecording(self, device, function, promptNum):
        # validResponse = False
        # while(validResponse == False):
        #     try:
        #         with sr.Microphone(device_index=device) as source2:
        #             # wait for a second to let the recognizer
        #             # adjust the energy threshold based on
        #             # the surrounding noise level 
        #             self.r.adjust_for_ambient_noise(source2, duration=0.2)

        #             #listens for the user's input 
        #             audio2 = self.r.listen(source2)

        #             # Using google to recognize audio
        #             MyText = self.r.recognize_google(audio2)
        #             MyText = MyText.lower()

        #             print(function + " prompt #" + str(promptNum) + " response is: ", MyText)
        #             if MyText != None:
        #                 validResponse = True
        #                 return MyText
        #             # SpeakText(MyText)

            with sr.Microphone(device_index=device) as source2:
                self.r.adjust_for_ambient_noise(source2, duration=0.5)  # Adjust to background noise
                print("Listening...")

                # Continuously listen for speech and stop when silence is detected
                while True:
                    audio2 = self.r.listen(source2)
                    try:
                        MyText = self.r.recognize_google(audio2).lower()
                        print(f"Detected speech: {MyText}")

                        if MyText != None:  # If text is not empty
                            print(function + " prompt #" + str(promptNum) + " response is: ", MyText)
                            return MyText  # Stop recording and return text


                    except sr.RequestError as e:
                        print("Could not request results; {0}".format(e))

                    except sr.UnknownValueError:
                        # print("unknown error occurred")
                        print("Silence detected, stopping recording...")
                        retry = self.audioRecording(device,function,promptNum)
                        if retry != None:
                            return retry
                        else:
                            return None  # Stop if no speech is recognized

    # def run(self):
    #     while True:
    #         if TESTING_MODE:
    #             self.processText("",testing=True)
            # else:
            #     while(1):
            #         recordedText = self.audioRecording(self.deviceNum, "Run", 1)
            #         # if recordedText != None:
            #         #     usableText = recordedText
            #         if recordedText != None:
            #             self.processText(recordedText)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    # stt = SpeechToText()
    # stt.comms.root.mainloop()
    # stt.run()