# CarPuter
Development of an Automatic Speed Control for Vehicles

This project consists of developing a speed controller for a Mitsubishi Pagero TR4, based on an Arduino.
-	The Arduino Speed Control system gets the current speed, and the state of the throttle and break pedals from the car diagnostic port (OBD2).
-	The speed of the car is controlled by pulling the throttle cable with a servo motor.
-	The speed of the car and pedal state are obtained using a Bluetooth adapter module (ELM 327) plugged to the OBD2 car interface
-	The Arduino Speed Control system is connected to the ELM 327 Bluetooth module using a HC-05 Bluetooth module (BTH07 MODULE)
-	Warning: The HM-10 Bluetooh Module is a Bluetooth 4 and cannot connect to Bluetoth 2 devices such as the ELM 327 Bluetooth adapter!!!


***> How to connect Arduino or RaspberyPi to a HC-05 Bluetooh Module

-	We normally use the software serial  library (SoftwareSerial.h) to connect to the Bluetooth Module UART.

//Useful Commands:
-	SoftwareSerial mySerial(7, 8); // RX, TX
-	mySerial.begin(9600);
-	mySerial.flush();
-	mySerial.write("AT");  
-	while (mySerial.available())       Serial.write(mySerial.read());

WARNINGs: 

1)	There are many translation mistakes in the reference PDFs (HC-0305_serial_module_AT_commamd_set_201104_revised.PDF and Bluetooth4_en.pdf).
-	Always make sure to have the same version of the HC-05 software and the pdf manual!!

2)	In order to communicate to the HC-05 Module, it has to be set to AT Mode:
-	the KEY pin has to be set to HIGH (3,3V) and the default baund rate to 38400

2)	Note that the module will not give any reply if it does not understand a given command.

3)	If the module is turnned off in Master mode (Role=1), it will remain in Master mode when turnned on again.

4)	Warning: Differently from the HM-10 Bluetooth Module, the HC-05 Module commands have to end with both NL and CR ("\r\n").

5)	The proper way to manage the connection to the ELM327 Bluetooth Module:

 
-	Enter HC-05 AT mode		// the KEY pin has to be set to HIGH (3,3V) and the default baund rate to 38400
-	delay(500);
-	AT+RESET				//send to Reset HC-05
-	delay(1000);
-	AT+ROLE=1               //send ROLE=1, set role to master
-	AT+CMODE=0              //send CMODE=0, set connection mode to specific address
-	AT+BIND=1122,33,DDEEFF  //send BIND=??, bind HC-05 to OBD bluetooth address ("1122,33,DDEEFF" is the ELM327 MAC address)
-	AT+INIT                 //send INIT, cannot connect without this cmd. Initialize the SPP lib
-	delay(1000); 
-	AT+PAIR=1122,33,DDEEFF,20 //send PAIR, pair with OBD address - Try it for 20 secconds
-	delay(1000);  
-	LINK=1122,33,DDEEFF     //send LINK, link with OBD address
-	delay(1000); 
-	Enter HC-05 comunication mode - Basically, KEY pin is set to LOW to allow comands to be sent to the ELM327. Keep baund rate set to 38400
-	delay(500);

-	** Begin to send and receive commands to the ELM327 Module **


6)	Exemple of a communication attempt via terminal that acctually worked (sent commands and ELM327 reply): 

-	AT+ROLE1		// Master Mode

OK
-	AT+INIT			// Init SPP library

OK
-	AT+INQ			// Discover other Bluetooth MAC addresses

+INQ:8818:56:6898EB,1F00,7FFF
OK
-	AT+LINK=8818,56,6898EB		// Connect to ELM327 Bluetooth

OK
OK
-	0111			// Commands sent to ELM327 OBD2 Module

SEARCHING...
UNABLE TO CONNECT	// Did not work ...

-	AT+BIND=8818,56,6898EB	// Do not know why it is important... Just tryied...

AT+BIND=8818,56,6898EB
?

-	0111		// And finnaly got a response!!!

0111
SEARCHING...
41 11 00 

-	0111

0111
41 11 1A 

-	011C

011C
41 1C 06 

-	011D

011D
7F 01 12 


