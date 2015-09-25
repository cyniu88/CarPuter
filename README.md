# CarPuter
Development of an Automatic Speed Control for Vehicles

This project consists of developing a speed controller to a car, based on an Arduino.
-	The Arduino is connected into the cars diagnostic port: OBD2
-	The throttle cable is pulled using a servo motor
-	The speed of the car is obtained using a Bluetooth adapter module (ELM 327) plugged to the OBD2 car interface
-	The Arduino is connected to the ELM 327 Bluetooth module using a HM-10 Bluetooth module


***> How to connect Arduino or RaspberyPi to a HM-10 Bluetooh Module

	We normally use the software serial  library (#include <SoftwareSerial.h>) to connect to the HM-10 Module UART. So include the library as follow:
	#include <SoftwareSerial.h>
	SoftwareSerial mySerial(7, 8); // RX, TX
	mySerial.begin(9600);
	//Useful Commands:
		  mySerial.flush();
		  mySerial.write("AT");  
		  while (mySerial.available())       Serial.write(mySerial.read());

WARNINGs: 
1)	There are many translation mistakes in the reference PDF (Bluetooth4_en.pdf). --Always make sure to have the same version of the HM-10 software and the pdf manual!!
2)	Note that the module will not give any reply if it does not understand a given command.
3)	Once connected to a second HM-10 module, the first Module will always attempt to connect to the second again every time it is in ROLE1 mode, even if the second is turned off and on again. It may cause a lot of confusion.
4)	Once connected to a second module, the module will refuse all commands except the “AT” command, that will break the connection. It will treat every message as data and transmit it to the second module.
5)	Once it have been connected to a second module, it you turn power on and change ROLE to 1 (ROLE1 = Master) it will immediately try to connect to the second module again and then ignore all your commands, treating them as data and sending them as data to the second module.
6)	The only way to solve this (2-4), is to turn off the second module; then reset the first one and give the command AT+IMME1 -> this will make the module stop doing stupid things and wait for your commands!!! IMME1 will remain even after switching on and of again.
7)	The proper way to manage two modules is:
	a)	Turn on both modules and make sure they are in slave mode: AT+ROLE0 is totally redundant as the modules start in slave mode when powered on.
	b)	Stop first module from doing stupid things: AT+IMME1
	c)	Make first module Master: AT+ROLE1
	d)	Scan available modules: AT+DISC?      -> if you want to know the names of the available modules, you should give the command AT+SHOW1 before.
	e)	Choose one of the presented Bluetooth to connect: AT+CONN0, AT+CONN1…
	f)	If you want to connect direct to a given module, you need the command: AT+CON 0017EA090909, where 0017EA090909 is the MAC address.
		It will respond: 
		OK+CONNA    =========    Accept request, connecting
		OK+CONNE    =========    Connect error
		OK+CONN     =========    Connected, if AT+NOTI1 is setup
		OK+CONNF    =========    Connect Failed, After 10 seconds
	g)	You can now send data between the modules.
	h)	To break the connection, send command AT   -> it will respond: OK+LOST


***> How the HM-10 Bluetooh Module to the ELM327 Bluetooth Module

