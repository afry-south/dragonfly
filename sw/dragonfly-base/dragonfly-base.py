import json 
import time
import sys
from random import randint
import RPi.GPIO as GPIO
import urllib2
import pprint

#This python script controls the dragonfly charger base station.
#It make url requests to thingspeak.com where it reads if the charger should be turned on or off.
#It runs on a raspberrypi with a GPIO board. 
#The script should be started during startup. Place the script eg here: /home/pi/dragonfly-base.py
#Add to this to /etc/rc.local "/usr/bin/python /home/pi/dragonfly-base.py &" to automatcally start at boot.

#Uses ThingSpeak.com to set and read charger variables. 
#Two channels must be used if read and write are asyncronous.

#Channel "Dragonfly Set Charger" is used to send orders to charger.
#Keys to write/read Dragonfly Set Charger.
writeSetAPIKey = "HSSSXOE5P90V0P28" #use this to write to thingspeak
readSetAPIKey =  "JW7PD9LICE10WVK3" #use this to read from thingspeak

#Channel "Dragonfly Get Charger" is used to confirm orders/read status from charger.
#Keys to write/read Dragonfly Get Charger.
writeGetAPIKey = "P07ZXG60Y1CTJGB4" #use this to write to thingspeak
readGetAPIKey =  "IV82A3N7MDFNP5H5" #use this to read from thingspeak

#TODO Add check off time so that if Dragonfly Set Channel hasn't been updated for a while then turn charger off.
#TODO Not more than one pair dragonfly-base/dragonfly-fms should be able to use one channel. 

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)

while 1:         
	ChargerOnOffURL = 'https://api.thingspeak.com/channels/117072/fields/1/last?key=%s' % readSetAPIKey	
	try:
		respons = urllib2.urlopen(ChargerOnOffURL)
		chargerOnOff = respons.read()
	except:
		print "Could'n open url"
		chargerOnOff = -1                
            
        if chargerOnOff == '1':
                print "Charger On"
 		GPIO.output(7, True)
	        GPIO.output(11, False)
	        GPIO.output(40, True)

	elif chargerOnOff == '0':
                print "Charger Off"
		GPIO.output(7, False)
	        GPIO.output(11, True)
	        GPIO.output(40, False)

	else:
		print "No valid charger value, set charger to off"
		GPIO.output(7, False)
                GPIO.output(11, True)
                GPIO.output(40, False)
	
	ConfirmOnOffURL = 'https://api.thingspeak.com/update?key=%s&field1=%s' % (writeGetAPIKey, chargerOnOff) 
	try:
		respons = urllib2.urlopen(ConfirmOnOffURL)
	except:
		print "Could'n confirm"
	
	time.sleep(15)
           
respons.close()
chargerOnOff.close()
GPIO.cleanup()

        
