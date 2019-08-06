# cosmicpi-arduino_V1.6
STM32 Arduino Core firmware for the Version 1.6

## Features
*	Set up the detector with default values, if it hasn't been calibrated
*	Send events from the detector via serial to Raspberry Pi
*	Send data from all sensors on the board via serial
*	Configure the on board GPS and pipe it's data to serial

### Meaning of the LEDs
Note that the LED's aren't very bright!
*	Lower LED (blue): Power and GPS
	*	Solid: Power, but no GPS lock
	*	Blinking: Power and GPS lock
*	Upper LED (blue): Event
	*	Flash: An Event has been registered

	
## ToDo before release
*	The timer needs a rework, its current implementation is very imprecise (uses micros() rather than a proper timer)
*	The detector should be able to calibrate it's parameters automatically
	*	High voltage
	*	Thresholds
  * Add a mode for this from the Pi.

	
## Installation
For regular users of the CosmicPi V1.6 this should be taken care of automatically by the software on the RaspberryPi.
If you want to modify the firmware, you will need an ST-LINK programmer (we used the Nucleo64 STM32F401RE board). You can find out how to do the programming here:
https://www.st.com/content/ccc/resource/technical/document/user_manual/98/2e/fa/4b/e0/82/43/b7/DM00105823.pdf/files/DM00105823.pdf/jcr:content/translations/en.DM00105823.pdf

For everybody interested in looking into developing this a bit more:
1.	Download the most recent Arduino IDE: https://www.arduino.cc/en/main/software
2. 	Install the STM32 core for Arduino: https://github.com/stm32duino
3.  Install the COSMICPI variant https://github.com/CosmicPi/STM32_Arduino_Core
4. 	Clone this repository: `git clone https://github.com/CosmicPi/cosmicpi-arduino_V1.6.git`
(note you can also download the .zip file from the menu above to the right, expand and remember to re-name the directory so cosmpicpi-arduino_v1.6 otherwise Arduino will give you errors)
5.	Open the file `cosmicpi-arduino_V1.6.ino` with the Arduino IDE
6.	Connect your CosmicPi to your computer via the ST-LINK pins of your programmer, you may find it helpful to remove the Raspberry Pi, we use the +5V pin on the raspberry pi header to power the device, and the UART pins for debugging.
7. 	Select the newly appearing port in the Arduino IDE (Tools -> Port)
8.  Select the correct board and variant (Board = Nucleo-64, Board Part Number = COSMICPI)
9.  Select the following 'standard' options - smallest OS default, newlib nano default, upload method ST-Link, 
10.	Compile and upload the firmware (Sketch -> Upload)


## Usage
As soon as the CosmicPi is connected you can open the software of your choice for monitoring serial data. Open the Arduinos serial port with a baudrate of 19200.
This firmware is designed to be a plug and play, if you get stuck drop us a line! 
It will only send data. It will not accept inputs. What data is sent is defined [here](https://github.com/CosmicPi/cosmicpi-rpi_V1.6/blob/master/documentation/CosmicPi_V16_serial_comm.txt).

Note that the V1.6 firmware has been designed to be identical to the V1.5 firmware on the outside, but there are some major changes on the inside. 
