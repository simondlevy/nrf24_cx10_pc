 This is a fork of the Multi-Protocol nRF24L01 Tx project
 from goebish on RCgroups / github
 This version accepts serial port strings and converts
 them to ppm commands which are then transmitted via
 the nRF24L01. 

 The purpose of this code is to enable control over the Cheerson CX-10 
 drone via code running on a PC.  In my case, I am developing Python code to
 fly the drone. 
 
 This code can be easily adapted to the other mini-drones 
 that the Multi-protocol board supports. 

 The format for the serial command is:
 ch1value,ch2value,ch3value,...
 e.g.:  1500,1800,1200,1100, ...

 Up to 12 channel commands can be submitted. The channel order is defined
 by chan_order. The serial port here is running at 115200bps. 

 Python code in serial_test.py was written to generate the serial strings.  

 Hardware used:
 This code was tested on the Arduino Uno and nRF24L01 module. 
 Wiring diagrams and more info on this project at www.makehardware.com/pc-mini-drone-controller.html
 
 I believe this code will remain compatible with goebish's 
 nRF24L01 Multi-Protocol board.  A way to 
 connect to the serial port will be needed (such as the FTDI). 
 
 Perry Tsao 29 Feb 2016
 perrytsao on github.com
 *********************************************************************************

 
 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


