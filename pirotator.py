#!/usr/bin/env python
# coding: latin-1

# pirotator
# Ville Jussila (wilho / OH8ETB)
# https://jkry.org/ouluhack/pirotator
# 
# Free as a free beer or something.. 
# use and do what ever you want, i will be pleased if you mention me on credits.
#
#
#

#General TODO:

# Import library functions we need
#import PicoBorgRev
import time
import threading
import Queue
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
#import Adafruit_BBIO.UART as UART
##import RPi.GPIO as GPIO
import binstr
import socket
import sys,getopt
import os.path
import math
import serial
import logging
import datetime
import sys
#setup logging
logFormatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
rootLogger = logging.getLogger()

fileHandler = logging.FileHandler("pirotator.log")
fileHandler.setFormatter(logFormatter)
rootLogger.addHandler(fileHandler)

consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(logFormatter)
rootLogger.addHandler(consoleHandler)

#rootLogger.setLevel(logging.INFO)
#rootLogger.setLevel(logging.DEBUG)
rootLogger.setLevel(logging.INFO)

#parse args (is this needed?)

#failsafe failsafe based on previous experience
EMERGENCYSTOP = False

#things we need to init everything
azi2c = 11
eli2c = 10


#stepperctrl1  = ["P9_12","P9_15","P9_23","P9_41"]
#stepperctrl1_other = ["P9_23","P9_12","P9_15","P9_41"]
#stepperctrl2 = ["P9_25","P9_27","P9_30","P8_26"]
##GPIO.setmode(GPIO.BCM)
##stepperctrl1 = [27,17,22,24]
#stepperctrl1 = [27,22,17,23]
##stepperctrl2 = [10,9,11,18]
pins = [7,5,6,12,13,19,16,26,20,21]

tty = '/dev/ttyAMA0'
#pins = [26,24,7,22,18,16,15,13,12,11]
#pins = ["P8_17","P8_16","P8_15","P8_14","P8_12","P8_11","P8_10","P8_9","P8_8","P8_7"]

## pseudo code planing for rewrite

#class for handling potentiometer, and filter data from it
class Potentiometer:
    def __init__(self,pins):
        import Adafruit_BBIO.ADC as ADC
        ADC.setup()
        self.pins = pins
        self.deg = 0
        self.filt_deg = 0
        self.ratio = (370.0) #turnrate of the potentiometer
        self.offset = 0.0
        #self.initGPIO(self.pins)
        self.start() #start loop

    def start(self):
        loop = threading.Thread(target=self.loop)
        loop.daemon = True
        loop.start()

    def loop(self):
        values = [] #this is for some averaging
        self.filt_deg = self.read_value()
        self.deg = self.read_value()
        while True:
            value = self.read_value()
            #print value
            #print str(self.deg-10) +" "+ str(value)+" "+str(self.deg+10)
            if self.filt_deg-10 <= value <=self.filt_deg+10: # some filtering if RF causes problems
                self.deg = value
                values.append(value)
                self.filt_deg = sum(values) / float(len(values)) # averaging
                if len(values) > 10:
                    values.pop(0)
                #print len(values)
            else:
                logging.info( "value has been changed too much for 0.1sec, RF-spike?")
            time.sleep(0.1)


    def read_value(self):
        return (self.readGPIO(self.pins)-self.offset)*self.ratio

    def get_value(self):
        return self.filt_deg

    #this does actual reading of the pins
    def readGPIO(self,pins):
        potVal=ADC.read(pins[0])
        potVolt=potVal*1.8
        #debug
        #print "The heading  is: ",potVal*self.ratio, "degrees      (",potVolt,"volts)"
        return potVal



#class for handling rotary encoder, and filter data from it
class RotaryEnc:
    def __init__(self,pins):
        self.pins = pins
        self.deg = 0
        self.filt_deg = 0
        self.ratio = (360.0/1024) #1024 (10bit) for 360 degres
        self.initGPIO(self.pins)
        self.start() #start loop

    def start(self):
        loop = threading.Thread(target=self.loop)
        loop.daemon = True
        loop.start()

    def loop(self):
        values = [] #this is for some averaging
        self.filt_deg = self.read_value()
        self.deg = self.read_value()
        while True:
            value = self.read_value()
            #print value
            #print str(self.deg-10) +" "+ str(value)+" "+str(self.deg+10)
            if self.filt_deg-10 <= value <=self.filt_deg+10: # some filtering if RF causes problems
                self.deg = value
                values.append(value)
                self.filt_deg = sum(values) / float(len(values)) # averaging
                if len(values) > 10:
                    values.pop(0)
                #print len(values)
            else:
                logging.info( "value has been changed too much for 0.1sec, RF-spike?")
            time.sleep(0.1)
 

    def read_value(self): 
        return self.readGPIO(self.pins)[0]*self.ratio
    

    def get_value(self):
        return self.filt_deg

    def initGPIO(self,pins):
        #GPIO.setmode(GPIO.BOARD)
        for pin in pins:
            print pin
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            time.sleep(0.1)

    #this does actual reading of the pins
    def readGPIO(self,pins):
        GPIO.setmode(GPIO.BCM)
        #for pin in pins:
        #    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #    time.sleep(0.001)
        raw = ""
        inv_raw = ""
        for pin in pins:
        #    print pin
            raw += str(GPIO.input(pin))
            time.sleep(0.001)
        for bit in raw:
            if bit == "1":
                inv_raw += "0"
            else:
                inv_raw += "1"
        #print raw
        #GPIO.cleanup()
        return int(binstr.b_gray_to_bin(inv_raw, "big"),2),raw

#class for handling SolidState "encode" via tty
class SSE:
    def __init__(self,tty='/dev/ttyAMA0'):
        self.tty = tty
        print tty
        self.deg = 0
        self.filt_deg = 0
        self.ser = serial.Serial(port=self.tty,baudrate=38400,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
        
        self.start() #starts actual loop in own thread
    
    def start(self):
        loop = threading.Thread(target=self.loop)
        loop.daemon = True
        loop.start()

    def loop(self):
        values = [] #this is for making some averaging
        self.filt_deg = self.read_value()
        while True:
            value = self.read_value()

            if self.filt_deg-300 <= value <=self.filt_deg+300: # some filtering if RF causes problems
                self.deg = value
                values.append(value)
                self.filt_deg = sum(values) / float(len(values)) # averaging
                if len(values) > 6: #change this if need more filtering
                    values.pop(0)
            else:
                logging.info( "value has been changed too much for 0.1sec, RF-spike?")
            time.sleep(0.1)

    #this does actual reading from tty
    def read_value(self):
        self.ser.isOpen() 
        self.ser.write("get-360") #+ '\r\n')
        #print "writing get-360"
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(0.5)
        while self.ser.inWaiting() > 0:
            out += self.ser.read(1)

        if out != '':
            #print ">>" + out
            try:
                return float(out)*-1 # hack, if you mount sensor wrongside up, you dummy!
            except:
                return 0.0
        else:
            print "did not manage to get reading from sensor!"    
            return None

    def get_value(self):
        return self.filt_deg

class relayMotor:
    def __init__(self,pins):
        global EMERGENCYSTOP
        self.hasRelay     = True
        self.relayState   = 0
        self.leftPin    = pins[0]
        self.rightPin     = pins[1]
        print self.leftPin
        print self.rightPin
        #self.power = power
        self.power = False
        self.pulseState = 0
        self.prevdir = -1
        self.setup()
    def setup(self):
        #GPIO.setmode(GPIO.BOARD)
        print "Init LeftPin"
        GPIO.setup(self.leftPin,    GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.leftPin, 0)
        print "init RightPin"
        GPIO.setup(self.rightPin,    GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.rightPin, 0)
        #print "disable relay"
        #self.toggle_relay(0)
        time.sleep(1)
        print "enable relay"
        self.toggle_relay(1)
        return

    def toggle_relay(self,state=1):
        print "changing motor state to "+str(state)
        self.relayState = state
        #GPIO.output(self.relayPin, state)
        #time.sleep(0.5)
        if state == 0 :
            #GPIO.output(self.enablePin, 0)
            self.power = False
            GPIO.output(self.leftPin, 0)
            GPIO.output(self.rightPin, 0)
            self.prevdir = -1
            #GPIO.output(self.relayPin, 1)
            #time.sleep(0.1)
            #GPIO.output(self.relayPin, 0)

    def power_off(self):
        GPIO.output(self.leftPin, 0)
        self.power = False
        GPIO.output(self.rightPin, 0)
        print "disable motor"

    def do_step(self,dir):

        if EMERGENCYSTOP:  #failsafe
            print "STOPPED BY EMERGENCYSTOP"
            return False

        if self.power != True:
            if self.relayState == 0:
                self.toggle_relay(1)
            #GPIO.output(self.enablePin, 0)
            self.power = True
            print "enable motor"
        #GPIO.output(self.enablePin, 0)
        #do step forward
        if dir > 0:
            if self.prevdir != dir:
                print "change dir"
                GPIO.output(self.leftPin, 0)
                GPIO.output(self.rightPin, 0)
                time.sleep(0.5)
                #GPIO.output(self.directionPin, 1)
                #time.sleep(0.5)
                #GPIO.output(self.enablePin, 0)
         #   GPIO.output(self.directionPin, 1)
            self.pulseState += 1
            #GPIO.output(self.pulsePin, self.pulseState % 2)
            GPIO.output(self.rightPin, 1)
        #and to another dir
        else:
            if self.prevdir != dir:
                print "change dir"
                GPIO.output(self.leftPin, 0)
                GPIO.output(self.rightPin, 0)
                time.sleep(0.5)
                #GPIO.output(self.directionPin, 0)
                #time.sleep(0.5)
                #GPIO.output(self.enablePin, 0)
            #GPIO.output(self.directionPin, 0)
            self.pulseState += 1
            #GPIO.output(self.pulsePin, self.pulseState % 2)
            GPIO.output(self.leftPin, 1)
        self.pulseState %= 2
        self.prevdir = dir

        return True

class Stepper:
    def __init__(self,pins):
        global EMERGENCYSTOP
#        self.enablePin    = "P9_12" # Note enable is active low!
#        self.pulsePin     = "P9_15"
#        self.directionPin = "P9_23"
#        self.enablePin    = "P9_25" # Note enable is active low!
#        self.pulsePin     = "P9_27"
#        self.directionPin = "P9_30"

#        self.enablePin    = "P9_23" # Note enable is active low!
#        self.pulsePin     = "P9_12"
#        self.directionPin = "P9_15"
        self.hasRelay     = True
        self.relayState   = 0
        self.enablePin    = pins[0] 
        self.pulsePin     = pins[1]
        self.directionPin = pins[2]
        self.relayPin     = pins[3]

        self.pulseState = 0
        #self.power = power
        self.power = False
        self.pulseState = 0
        self.prevdir = -1
        self.setup()

    def setup(self):
        #GPIO.setmode(GPIO.BOARD)
        print "powering up stepper via relay"
        GPIO.setup(self.relayPin,    GPIO.OUT, initial=GPIO.LOW)
        print "init EnablePin"
        GPIO.setup(self.enablePin,    GPIO.OUT, initial=GPIO.LOW)
        print "init DirectionPin"
        GPIO.setup(self.directionPin, GPIO.OUT, initial=GPIO.LOW)
        print "init PulsePin"
        GPIO.setup(self.pulsePin,     GPIO.OUT, initial=GPIO.LOW)
        print "disable relay"
        self.toggle_relay(0)
        time.sleep(1)
        print "enable relay"
        self.toggle_relay(1)
        GPIO.output(self.directionPin, 0)
        GPIO.output(self.enablePin, 1)
        return

    def toggle_relay(self,state=1):
        print "changing relay state to "+str(state)
        self.relayState = state
        GPIO.output(self.relayPin, state)
        time.sleep(0.5)
        if state == 0 :
            #GPIO.output(self.enablePin, 0)
            self.power = False
            GPIO.output(self.enablePin, 0)
            GPIO.output(self.directionPin, 0)
            GPIO.output(self.pulsePin, 0)
            self.prevdir = -1
            #GPIO.output(self.relayPin, 1)
            #time.sleep(0.1)
            #GPIO.output(self.relayPin, 0)

    def power_off(self):
        GPIO.output(self.enablePin, 1)
        self.power = False
        GPIO.output(self.pulsePin, 0)
        print "disable stepper"

    def do_step(self,dir):

        if EMERGENCYSTOP:  #failsafe
            print "STOPPED BY EMERGENCYSTOP"
            return False

        if self.power != True:
            if self.relayState == 0:
                self.toggle_relay(1)
            GPIO.output(self.enablePin, 0)
            self.power = True
            print "enable stepper"
        #GPIO.output(self.enablePin, 0)
        #do step forward
        if dir > 0:
            if self.prevdir != dir:
                print "change dir"
                GPIO.output(self.enablePin, 1)
                GPIO.output(self.pulsePin, 0)
                time.sleep(0.5)
                GPIO.output(self.directionPin, 1)
                time.sleep(0.5)
                GPIO.output(self.enablePin, 0)
         #   GPIO.output(self.directionPin, 1)
            self.pulseState += 1
            GPIO.output(self.pulsePin, self.pulseState % 2)
        #and to another dir
        else:
            if self.prevdir != dir:
                print "change dir"
                GPIO.output(self.enablePin, 1)
                GPIO.output(self.pulsePin, 0)
                time.sleep(0.5)
                GPIO.output(self.directionPin, 0)
                time.sleep(0.5)
                GPIO.output(self.enablePin, 0)
            #GPIO.output(self.directionPin, 0)
            self.pulseState += 1
            GPIO.output(self.pulsePin, self.pulseState % 2)

        self.pulseState %= 2
        self.prevdir = dir

        return True

#yes, after all we do need class for stepper (it is easier to implement to use of other controllers)
# there is no fancy loops or nothing, only .step(direction) for moving stepper one step by step
# should be quite easy to implement on "real" controllers

class Stepper_pico:
    def __init__(self,i2c,power=0.29):
        global EMERGENCYSTOP
        self.i2c = i2c
        self.power = power
        self.sequence = [[self.power, self.power], [self.power, -1*self.power], [-1*self.power, -1*self.power], [-1*self.power, self.power]]
        self.PBR = PicoBorgRev.PicoBorgRev()
        self.PBR.i2cAddress = i2c
        self.PBR.Init()
        self.PBR.ResetEpo()
        self.PBR.MotorsOff()
        self.power = False
        self.step = -1
        self.hasRelay     = False

    def power_off(self):
        self.PBR.MotorsOff()
        self.power = False
        self.step = -1

    def reset(self):
        self.step = -1

    def do_step(self,dir):

        if EMERGENCYSTOP:  #failsafe
            print "STOPPED BY EMERGENCYSTOP"
            return False

        self.power = True
        if self.step == -1:
            drive = self.sequence[-1]
            self.PBR.SetMotor1(drive[0])
            self.PBR.SetMotor2(drive[1])
            self.step = 0
        else:
            self.step += dir

        # Wrap step when we reach the end of the sequence
        if self.step < 0:
            self.step = len(self.sequence) - 1
        elif self.step >= len(self.sequence):
            self.step = 0

        # For this step set the required drive values
        if self.step < len(self.sequence):
            drive = self.sequence[self.step]
            self.PBR.SetMotor1(drive[0])
            self.PBR.SetMotor2(drive[1])
        return True
        


#class for hanling each axis 
class Axis:
    def __init__(self,speed,sensor,stepper,min,max,radio=None,reverse=False,type=1,deg_per_step=0.007):
        self.radio = radio
        self.wdir = sensor.get_value() 
        self.olddir = sensor.get_value()  #starting point where we start to find position to wdir
        self.sensor = sensor
        self.stepper = stepper
        self.speed = speed #this is actually time to wait between steps, so less is more speed
        self.reverse = reverse #if need to reverse movement
        self.min = min #min degree where monotrs are force off (failsafe)
        self.max = max #max degree when motors are forced off (failsafe)
        self.type = type #could support different kinds or motors etc
        self.last_received = datetime.datetime.now()
        self.alive = datetime.datetime.now()
        self.flip = True
        self.status = 'idle'
        self.deg_per_step = deg_per_step #how many degrees per stepper step, used for calculating maxsteps for floop (failsafe)
        self.maxsteps = 1000 #max steps used in floop, have to instance global as you could update wdir on the fly
        global EMERGENCYSTOP  # if anything else fails

        self.start_loop()
    #starts actual loop to own thread
    def start_loop(self):
        loop = threading.Thread(target=self.main_loop)
        loop.daemon = True
        loop.start()
    
    #idle loop, if we have find our destination return to this loop, checks if wdir == sensordata if not starts floop()
    def main_loop(self):
        self.errors = []
        count = 0
        while True:
            self.alive = datetime.datetime.now()
            reading = self.sensor.get_value()
            towdir = reading - self.wdir

            if self.radio !=None:
                #sleep while Radio TX
                while self.radio.get_tx():
                    print "Waiting radio TX, from mainloop"
                    time.sleep(1)
                    self.alive = datetime.datetime.now()

            if towdir <= 0:
                towdir = towdir*-1

            if towdir > 2.3:
                #self.maxsteps = self.positive(int(self.deg_per_step*towdir*1.2))
                #self.maxsteps = self.deg_per_step*towdir*1.2 # gives maxsteps for finding (failsafe)
                self.maxsteps = int(towdir/self.deg_per_step*1.3)
                if self.maxsteps < 50:
                    self.maxsteps += 100
                count = 0
                success = self.floop()
                if success == True:
                    logging.info("Succesfully found our way to wanted dir")
                else:
                    logging.info("Failed to find our way to wanted dir")
                    self.errors.append(datetime.datetime.now())
                    #self.errors += 1

            if len(self.errors) > 3:
                logging.error("Failed to found dir too many times, exiting whole program for failsafe")
                print("Failed to found dir too many times, exiting whole program for failsafe")
                sys.exit(2)
            #reset errors if latest error older than 5min
            if len(self.errors) >= 1:
                if datetime.datetime.now() - self.errors[-1] >= datetime.timedelta(seconds=60):
                    logging.info("cleared errors, as older than 5min")
                    self.errors = []  

            time.sleep(1)
            #print count
            if count > 5 and self.stepper.hasRelay and self.stepper.relayState == 1:
                self.stepper.toggle_relay(0)
            count += 1
    def get_errors(self):
        return len(self.errors)
    def get_wdir(self):
        return self.wdir

    #finding loop, basically closed loop for finding wanted position based on sensor data, ends when found, or error, or used max amount of steps(failsafe)
    def floop(self):
        print "started floop"
        found_times = 0
        reading_start = self.sensor.get_value()
        while True:
            self.alive = datetime.datetime.now()
            #TODO set wdir to current if wdir changes
            reading = self.sensor.get_value()
            towdir = reading - self.wdir
            toolddir = reading - self.olddir 
            
            if self.radio != None:
                #sleep while Radio TX
                while self.radio.get_tx():
                    print "Waiting radio TX, from floop"
                    time.sleep(1)
                    self.alive = datetime.datetime.now()

            if towdir <= 0:
                dir = -1
            else:
                dir = 1

            if self.reverse:
                dir = dir * -1
            logging.debug( str(self.wdir)+" "+str(reading) + " " + str(towdir) + " "+ str(dir))
            #print str(self.wdir+0.5) + " " + str(reading) + " " +str(self.wdir-0.5)
            #we have reach our destination


            if self.wdir+0.3 >= reading >= self.wdir-0.3 and found_times > 5: #stop if 0.1deg away from wanted
                self.stepper.power_off()
                logging.info("I am so close, I could sleep now, I think.. wdir: "+str(self.wdir)+" steps left: "+str(self.maxsteps))
                return True

            if self.wdir+0.1 >= reading >= self.wdir-0.1: #stop if 0.1deg away from wanted
                #self.stepper.power_off()
                logging.debug("close adding one to fount_times times: "+str(found_times))
                found_times +=1


            #we have user all of our steps for finding direction (failsafe)
            if self.maxsteps == 0:
                logging.error("we have used all of our steps for finding direction. STOPPED! "+str(self.wdir))
                self.stepper.power_off()
                return False

            # min/max boundaries failsafe
            if self.min > reading or self.max < reading: # min/max check failsafe
                logging.error("position past min/max boundaries. STOPPED! "+str(self.wdir))
                self.stepper.power_off()
                return False

            # does actual step
            #print str(dir) + " " + str(self.maxsteps)
            if self.stepper.do_step(dir) == False:
                logging.error("Steppers have been disabled by EMERGENCYSTOP "+str(self.wdir))
                return False
            #wait between steps
            if self.type == 2:
                time.sleep(self.speed)
            
            else:
                if 0 <= self.positive(towdir) <=0.5 or 0 <= self.positive(toolddir) <=0.5: # first step of soft start/stop
                    time.sleep(self.speed*12)
                elif 0.5 <= self.positive(towdir) <=2 or 0.5 <= self.positive(toolddir) <=0.5: # second step of soft start/stop
                    time.sleep(self.speed*4)
                elif 2 <= self.positive(towdir) <=4 or 2 <= self.positive(toolddir) <=4: # second step of soft start/stop
                    time.sleep(self.speed*2)
                elif 4 <= self.positive(towdir) or 4 <= self.positive(toolddir): # full speed of soft start/stop
                    time.sleep(self.speed)
                else:
                    logging.error("WTF? Stopping motors via EMERGENCYSTOP")
                    EMERGENCYSTOP = True
            #print self.maxsteps
            if self.maxsteps == 1:
                logging.info("Last step, giving some time for sensors to read "+str(self.wdir))
                time.sleep(1)
                if reading_start == self.sensor.get_value(): #another failsafe for sensor problem
                    logging.error("We have used our steps and sensor reading is same than on start, CHECK SENSOR! "+str(self.wdir))
                    sys.exit(2)
                    global EMERGENCYSTOP
                    EMERGENCYSTOP = True   
            self.maxsteps -= 1



    def positive(self,deg):
        if deg < 0:
            return deg*-1
        else:
            return deg            

    def set_dir(self,bearing):
        self.olddir = self.sensor.get_value() #updates "starting point"
        self.wdir = bearing #updates wanted direction
        self.last_received = datetime.datetime.now() #updates last received command
        #calculating self.maxsteps (dublicate if in idle loop, but so what)
        towdir = self.sensor.get_value() - self.wdir
        if towdir <= 0:
            towdir = towdir*-1
        self.maxsteps = int(towdir/self.deg_per_step*1.3) # gives maxsteps for finding (failsafe)
        if self.maxsteps < 30:
            self.maxsteps += 100
    def get_value(self):

        # "flick" reading if set.alive older than 1min (probably thread died or something)
        if datetime.datetime.now() - self.alive >= datetime.timedelta(seconds=120):
            if self.flip:
                self.flip = False
                return self.sensor.get_value()
            else:
                self.flip = True
                return 45
        else:         
            return self.sensor.get_value()

class FlexRadio:
    def __init__(self,ip='10.90.10.198',port=4992):
        self.port = port
        self.tx = False
        self.ip = ip #'10.90.10.100' # howto get selfip?
        self.buffer_size = 20
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.connect((self.ip, self.port))
        #self.socket.listen(1)
        #self.conn, self.addr = self.socket.accept()
        #self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.start_loop()

    def start_loop(self):
        loop = threading.Thread(target=self.loop)
        loop.daemon = True
        loop.start()
    def translate_command(self,command):
         if command == 'S0|interlock state=TRANSMITTING source=SW tx_allowed=1':
             print("Translated TX")
             return "tx"
         elif command == 'S0|interlock state=READY tx_allowed=1':
             return "rx"
         else:
             return "none"

    def loop(self):
        data = ""
        while True:
            packet = self.socket.recv(self.buffer_size)
            if not packet: break

            data += packet

            if '\n' in data:
                line, data = data.split('\n', 1)
                print line

                ctype = self.translate_command(line)
                if ctype == "tx":
                    self.tx = True
                elif ctype == "rx":
                    self.tx = False

    def get_tx(self):
        return self.tx

class FakeAxis:
    def __init__(self):
        self.wdir = 0
        self.olddir = 0  #starting point where we start to find position to wdir
        self.status = 'idle'

    def get_value(self):
        return self.wdir

    def set_dir(self,value):
        self.wdir=value

#listens on some tcp socket, and emulates some known az/el rotator
# OR, you could spawn own socket for each axis (atleas pstrotator support this)
class SerialEmulator:
    def __init__(self,az,el,type="Radant",port=5005,ip='10.90.10.167'):
        self.az = az
        self.el = el
        self.type = type
        self.port = port
        self.ip = ip #'10.90.10.100' # howto get selfip?
        self.buffer_size = 20
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip, self.port))
        self.socket.listen(1)
        self.conn, self.addr = self.socket.accept()
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.start_loop()
    def translate_command(self,command):
        if self.type == "RAS":
             if command[-2] == chr(15):  # hex 0F
                 print("stop")
                 return "stop"

             elif command[-2] == chr(31): # hex 1F 
                 print("status")
                 return "status"
             elif command[-2] == chr(47): # hex 2F
                 print("move")
                 return "move"
        if self.type == "Radant":
             if command[0] == 'S':  
                 print("stop")
                 return "stop"
             elif command[0] == 'Y':  
                 return "status"
             elif command[0] == 'Q':
                 print("move")
                 return "move"


    def send_status(self):
        #az = format(self.az.get_value()), '.1f')
        az ='%.1f' % round(self.az.get_value(), 1)
        el ='%.1f' % round(self.el.get_value(), 1)
        if self.type == "RAS":
            logging.error("Not implemented status for RAS")
            return 0            
        elif self.type == "Radant":
            return "OK"+az.zfill(5)+" "+el.zfill(5)+'\r'

    def move(self,command):
        if self.type == "RAS":
            logging.error("Not implemented moving for RAS")
            return 0
        elif self.type == "Radant":
            az = float(command.strip('Q').split()[0])
            self.az.set_dir(az)
            el = float(command.strip('Q').split()[1])
            self.el.set_dir(el)
            logging.debug(command)
            logging.debug("Set to AZ:"+str(az)+" EL:"+str(el))
            print "Set to AZ:"+str(az)+" EL:"+str(el)
            return True

    def stop(self):
        if self.type == "RAS":
            logging.error("Not implemented stopping RAS")
            return 0
        elif self.type == "Radant":
            print "STOP THE PRESS, NO, STOP THE ROTATOR!"
            global EMERGENCYSTOP 
            EMERGENCYSTOP = True
            return True
    def start_loop(self):
        loop = threading.Thread(target=self.loop)
        loop.daemon = True
        loop.start()


    def loop(self):
        while True:
            command = self.conn.recv(self.buffer_size)
            if not command: break

            ctype = self.translate_command(command)
            if ctype == "stop":
                self.stop()
            elif ctype == "status":
                self.conn.send(self.send_status())
            elif ctype == "move":
                self.move(command)
            else:
                logging.info("received something from socket, but could not handle it "+ command)
        

    #spawns socket serial emulator



# class for handling restapi traffic
class RestAPi:
    def __init__(self,az,el):
        self.az = az
        self.el = el

class Cli:
    def __init__(self,az,el=None):
        self.az = az
        self.el = el
        self.start_loop()

    def start_loop(self):
        loop = threading.Thread(target=self.main_loop)
        loop.daemon = True
        loop.start()

    def main_loop(self):
        try:
            # Loop forever
            while True:
                # Ask the user where to move
                dir = input("dir to move (-ve for reverse, 0 to quit): ")
                if dir == 0:
                    break
                else:
                    # Move to
                    self.az.set_dir(dir)
        except KeyboardInterrupt:
            print 'Terminated'


##main thread
#GPIO.cleanup()

#enableAZ    = "P9_41" # <- ylempi
#enableAZ    = "P8_26" # <- alempi
#print "enable az controller power relay"
#GPIO.setup(enableAZ,    GPIO.OUT)
#GPIO.output(enableAZ, 0)
#time.sleep(5)
#GPIO.output(enableAZ, 1)
#time.sleep(5)
#GPIO.output(enableAZ, 0)
#time.sleep(5)
#GPIO.output(enableAZ, 1)

#powers up the ttl adapter
#enableRS = "P8_18"
#GPIO.setup(enableRS,    GPIO.OUT)
#GPIO.output(enableRS, 1)
#UART.setup("UART4")


##az_stepper = Stepper(stepperctrl2)
##time.sleep(2)
##el_stepper = Stepper(stepperctrl1)

##az_sensor = RotaryEnc(pins)
#el_sensor = SSE("/dev/ttyO4")
##el_sensor = SSE(tty)

#radio = FlexRadio()

#####Sensors#######
az_pot_pins = ["P9_33","harhar"]
az_sensor = Potentiometer(az_pot_pins)

time.sleep(1) # give time to sensors to create some averaging

#####Motors########
az_relay_motor_pins = ["P9_12","P9_15"]
az_motor = relayMotor(az_relay_motor_pins)

#####Axis#########
az = Axis(0.004,az_sensor,az_motor,5,355,None,False,2)
el = FakeAxis()

#emulator = SerialEmulator(az_sensor,el_sensor)
#el = Axis(0.020,el_sensor,el_stepper,-10,60,None,True)

##el = Axis(0.010,el_sensor,el_stepper,-15,85,None,True)
#el = Axis(0.040,el_sensor,el_stepper,-10,85,None,True)
##az = Axis(0.004,az_sensor,az_stepper,25,350,None,False)
#az = Axis(0.032,az_sensor,az_stepper,30,320,None,False)
#el = Axis(0.020,el_sensor,el_stepper,-10,60,None,True)
#az = Axis(0.006,az_sensor,az_stepper,10,350,None,False)
#time.sleep(1)
#cli = Cli(az)
#az.set_dir(100)
#el = Axis(0.022,el_sensor,el_stepper,radio,-90,100,False,1,0.02)
#el = Axis(0.022,el_sensor,el_stepper,radio,-90,100,False,1,0.02)
emulator = SerialEmulator(az,el)

#serial = SerialEmulator(az,el)

#cli = Cli(az,el)

#mainloop
while True:
#    print "status: AZ: W"+str(az.get_wdir())+" S%.2f E"% round(az.get_value(), 1)+ str(az.get_errors())+ " R"+str(az.stepper.relayState)+"   EL: W"+str(el.get_wdir())+" S%.2f E"% round(el.get_value(), 1)+ str(el.get_errors())+ " R"+str(el.stepper.relayState)
    try:
        print "status: AZ: W"+str(az.get_wdir())+" S%.2f E"% round(az.get_value(), 1)+ str(az.get_errors())
        time.sleep(1)
    except KeyboardInterrupt:
        print 'You pressed Ctrl+C! I am trying to stop all the things properly'
        az.stepper.power_off()
        print 'to make sure i am tripping emergency off too, and wait for a bit'
        EMERGENCYSTOP = True
        time.sleep(3)
        print 'bye'
        sys.exit(0)






 
