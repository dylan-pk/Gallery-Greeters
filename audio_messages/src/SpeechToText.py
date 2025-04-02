import speech_recognition as sr
import pyttsx3
import pyaudio
from commands import Commands
# spoken = False

class SpeechToText:
    tables = [1,2,3,4,5]

    def __init__(self):
        # initialising a recnogiser
        self.r = sr.Recognizer()
        # creating pyaudio object so debugging
        self.device = pyaudio.PyAudio()
        self.comms = Commands()

    # # Convert Text to Speech
    # def SpeakText(command):
    #     engine = pyttsx3.init()
    #     engine.say(command)
    #     # The say function does not work without a run and wait command
    #     engine.runAndWait()

    def processText(self,text):
        print("processing")
        words = text.split()
        for word in words:
            match word:
                case "greet":
                    self.comms.modeChange(0)
                case "drink":
                    self.getDrinkOrder()
                case "art":
                    self.comms.artWorkInfo()
                case "wander":
                    self.comms.modeChange(1)
                case "charge":
                    self.comms.modeChange(2)
                case "table":
                    table = self.getTable()
                    self.comms.goToTable(table)
                        # self.tableCommands(words)

                case "waiter":
                    currentPosition = [0,0,0] # Get currentPosition value and pass it in
                    self.comms.callWaiter(currentPosition)
                case "fact":
                    self.comms.funFact()
                case "dance":
                    self.comms.modeChange(3)
                case _:
                    # print("Not a command word")
                    pass
    
    def tableCommands(self, sentence):
        print("In table commands function")
        if sentence == "table status":
            self.comms.tableStatus()
        elif sentence == "go to table":
            table = self.getTable()
            self.comms.goToTable(table)
    
    def getDrinkOrder(self):
        print("Drink Order Command")
        drinks = [0,0,0]
        table = 0
        ordering = True
        while ordering:
            self.comms.SpeakText("what drink would you like?")
            text = self.audioRecording(12, "Drink Order", 1)
            match text:
                case "carlton":
                    self.comms.SpeakText("How many would you like?")
                    amount = self.audioRecording(12, "Drink Order, Carlton", 2)
                    drinks[0] = amount
                case "soda":
                    self.comms.SpeakText("How many would you like?")
                    amount = self.audioRecording(12, "Drink Order, Soda", 2)
                    drinks[1] = amount
                case "wine":
                    self.comms.SpeakText("How many would you like?")
                    amount = self.audioRecording(12, "Drink Order, Wine", 2)
                    drinks[1] = amount

            self.comms.SpeakText("would you like another drink?")
            response = self.audioRecording(12,"Drink Order", 3)
            if response != "yes":
                ordering = False
            else:
                pass
        
        self.comms.SpeakText("Drink Order Sent")
        self.comms.sendDrinkOrder(drinks, table)

    def getTable(self):
        print("In get Table")
        self.comms.SpeakText("which table should I go to?")
        response = self.audioRecording(12, "GetTable", 1)
        response_words = response.split()
        for word in response_words:
            if word.isdigit():
                if int(word) < len(self.tables):
                    return int(word)
                
    def audioRecording(self, device, function, promptNum):
        validResponse = False
        while(validResponse == False):
            try:
                with sr.Microphone(device_index=device) as source2:
                    # wait for a second to let the recognizer
                    # adjust the energy threshold based on
                    # the surrounding noise level 
                    self.r.adjust_for_ambient_noise(source2, duration=0.2)

                    #listens for the user's input 
                    audio2 = self.r.listen(source2)

                    # Using google to recognize audio
                    MyText = self.r.recognize_google(audio2)
                    MyText = MyText.lower()

                    print(function + " prompt #" + str(promptNum) + " response is: ", MyText)
                    if MyText != None:
                        validResponse = True
                        return MyText
                    # SpeakText(MyText)

            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))

            except sr.UnknownValueError:
                print("unknown error occurred")

    def run(self):
        print(self.device.get_default_input_device_info())
        while(1):
            recordedText = self.audioRecording(12, "Run", 1)
            # if recordedText != None:
            #     usableText = recordedText
            self.processText(recordedText)

stt = SpeechToText()
stt.run()