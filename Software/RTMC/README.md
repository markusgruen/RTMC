# RTMC - a truly universal DAQ, Data Logger, and real-time I/O system
| - [What is RTMC?](#What-is-RTMC)<br> - [Hardware](#Hardware)<br> - [Software](#Software)<br> - [Example](#Example)<br> - [Bugs and known errors](#Bugs-and-known-errors)<br> - [Further ideas](#Further-ideas)<br> - [License and Legal](#License-and-Legal) | <img src="doc/RTMC_1.jpg" width="500px"> |
|:--|--|

## What is RTMC?

### Introduction
RTMC is an open source data acquisition [(DAQ)](https://en.wikipedia.org/wiki/Data_acquisition) platform which can measure signals from multiple sensors or other inputs and save the data for later analysis. [Up to eight](#number-of-connected-sensors) sensors can be read simultaneously and synchronously, with precise timing, and at sampling rates of up to 4.4 kHz.

This task itself is nothing new or special and countless commercial and open source devices are available. However, almost all of these devices only accept analog or simple digital signals, which severly limits the user's choice of sensors. Nowadays, many sensors are available with an I2C or SPI interface, but those cannot be connected with these conventional DAQs or data logging systems. RTMC, however, works with ***any*** sensor:
- analog
- I2C
- SPI
- UART
- RS232
- RS422/485
- CAN
- Profibus
- and many more, even proprietary interfaces and protocolls

For example, simultaneous reading of multiple I2C sensors, all with the *same* address, is absolutely no problem.

Additionally to the simple data logging task, RTMC can also act as a universal I/O system: You can connect actuators (motor, pump, servo, heater,...) to the RTMC, which can be programmed to act on the measured sensor signals. This expands RTMC's functionality to a real-time I/O system, e.g. for rapid control prototyping, which enables closed loop control systems with ***any*** sensor and ***any*** actuator to be built easily.

### Overview
<img src="doc/RTMC_2.jpg" width="500px">

The RTMC system consists of two elements: the [RTMC-nodes](#RTMC-node) and [RTMC-hub](#RTMC-hub). Each sensor is connected to an RTMC-node, which reads the sensor's output and transmits the data to the RTMC-hub. The RTMC-hub acts as the master-device, that generates a trigger signal at a user-defined, precise intervall, that triggers the nodes to read their sensor and transmit the data. The RTMC-hub then collects the data from the nodes, formattes it, and streams it to the PC via USB or to SD-card (not yet implemented). A [terminal program](#Terminal) on the user PC then receives the formatted data and can save it to a .txt or .csv file for analysis with Excel, Matlab, or other programs. 

### Is it difficult to set up?
There are two aspects to this question: Building the hardware, and programming the nodes. Because you are able to connect *any* sensor, The RTMC-nodes are ***not*** plug-and-play. But the programming part has been designed to be as beginner-friendly as possible and the process of adding a sensor only involves a few lines of code. Some examples are provided via the Arduino IDE to demonstrate how easy it is to set up. The RTMC-hub does not require any user defined program code.

For the hardware part, you should have experience in soldering SMD parts in 0603 size. Furtheremore, some basic electronic knowledge is required to connect your selected sensor to the RMTC-node. For instance, you should know when and how to use a logic level shifter. Unfortunately, no hardware is currently available for purchase. But all files and information are available to order the PCBs and build the hardware yourself. There are no special parts required. For people living in Germany, a Reichelt Warenkorb is provided.

For the software part, you should have some first experience within the Arduino world. The actual code to initialize the sensor and read the data from your sensor must be provided by you. However, for almost any sensor there are well documented libraries available, so all you need to do is copy and paste a few lines of code from the library's example file. All other aspects of RTMC, like handling the trigger signal, formatting the data and transmitting the data to the RTMC-hub are already implemented.

## Hardware
### RTMC-node
<img src="doc/RTMC-node_4.jpg" height="260px"> <img src="doc/RTMC-node_2.jpg" height="260px">

The RTMC-node is the interface between your sensors and the RTMC-hub. It is based on a standard [Raspberry Pi Pico](https://en.wikipedia.org/wiki/RP2040), that provides all neccessary connectivity, such as I2C, SPI, UART, and analog. You can simply connect the RTMC-node via USB to your computer and program the microcontroller using the provided example files in the Arduino IDE. Once programmed, disconnect the RTMC-node from the PC and connect it to the RTMC-hub using a standard ethernet cable. 

All of RTMC-node's inputs have a 3.3 V logic level. Use the breadboard area to add a standard logic level shifter if your sensor requires a different voltage level. You can also add a custom voltage regulator, or provide external voltage through the terminals, if the board voltages of 3.3V or 5V are not suitable.

The RTMC-node is designed to be as cheap as possible with component costs of less than 10 Euro. The idea is to have a node for each of your sensors and just store them together until you need it again.

The Schematics, Gerber files, and the BOM can be found in `Hardware/RTMC-node`-folder. I am currently working on providing KiCad files.

### RTMC-hub
<img src="doc/RTMC-hub_1.jpg" height="290px">

The RTMC-hub is the master device, that provides the trigger signal to the RTMC-nodes and collects and processes the sensor data. It is based on a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html). The hub has a dedicated START button to start and stop the data acquisition. Using the rotary knob, you can select between german and english csv-format (decimal point and separator). Power can be provided either by USB or from an external power source. Each RJ45 connector has a green LED, indicating a connected node and an orange LED, that lights up if there is a short on the connected node. With the RESET-button, the hub can be restarted. The real time clock is optional and can be used to create time based file names, when logging in stand-alone mode.

The Schematics, Gerber files, and the BOM can be found in `Hardware/RTMC-hub`-folder. I am currently working on providing KiCad files.

The V0.1 has some minor [hardware bugs](#Bugs-and-known-errors). The RTMC's functionality and performance are not affected by those bugs.

## Software

### Installation
1. Install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Install Earl F. Philhower's [RPi pico extension](https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json) as described [here](https://github.com/earlephilhower/arduino-pico#installing-via-arduino-boards-manager)
3. Install the [Teensy extension](https://www.pjrc.com/teensy/td_download.html)
4. Download the "RTMC.zip" library from the `Software`-folder, and install it in the Arduino IDE via "Sketch / Include Library / Add .ZIP library".

### Usage

#### RTMC-node
Check out the provided examples to get some impressions on how to  write code for the nodes. To get an "empty" arduino sketch, open `_basicSensor`. Below are some hints and remarks:

1. via "Files / Examples / RTMC / RTMCnode", open the "_basicSensor" example.
2. make sure to select `Raspberry Pi pico` via "Tools / Board".
3. change the name of your node via `Node node("mySensorName");` (e.g. call it "Accelerometer" if you have an accelerometer connected)
4. add an output for each value you want to transmit: e.g. `node.addOutput("travel [mm]");`
   you can also specify the decimals of each value: `node.addOutput("flow [ml/min", 3);` will print the flow with three decimals: `42.123`. The default is 2.
   Up to eight outputs can be added.
6. after `node.begin();`, add the sensor specific code for initializing the sensor.
   This code **must not** take longer than 4 seconds to execute.
7. in the function `void readSensorData()`, add the sensor specific code to read the values. Then, transmit the values via `node.output[0].write(sensor_value);`. "`sensor_value`" must either be a `float` or an `int`. Transmit your other sensor values (if any) by setting the number in the brackets (e.g. `node.output[1].write(...)`.

#### RTMC-hub
If you only want to use RTMC as a DAQ or data-logger, no user specific code is required. Simply flash the file `basic` found in "Files / Examples / RTMC / RTMChub" to your RTMC-hub. Make sure to select `Teensy 4.1` via "Tools / Board".

If you want to use RTMC as a real-time I/O-System, use the `advanced`-Example file. Use `node[0].output[0].readInt();` to read integers from the node or `node[0].output[0].readFloat();` if your node sends floats. Make sure to select the correct port.

#### Capturing data
You can directly use the Arduino's Serial monitor to capture data from the RTMC-hub. However, the serial monitor cannot save the received data to a file. If you want to save the data, close Arduino and connect to the RTMC-hub via a [terminal program](#Terminal). Once connected, you should see the RTMC-hub looking for connected nodes. If not, press the RESET-button. The hub will display the connected nodes (their names, outputs, and inputs, as specified in the node's code). It has also measured the node's response time, which is the minimum time required to read the sensor's value. The "slowest" node will define the minimum sample time.

#### How to start and stop the measurement
To start the measurement, simply press the START-button or send an empty message to the RTMC-hub (i.e. place the cursor in the transmit-field and just hit "Enter"). You can change the sampling time by sending `T`, followed by the sampling time in seconds. E.g. send `T0.1` to select a sampling time of 100 ms. You can even select faster sampling times than set by the RTMC by default. This is useful, if you have a "fast" and a "slow" sensor connected and want to capture the "fast" sensor with its maximum frequency. Then, the "slow" sensor will only transmit its value only every n-th sampling intervall. 

To stop the measurement, press the START-button, or send an empty message to the RTMC-hub. 

You can restart the measurement by pressing the START-button (or sending an empty message). The previous sampling time will be used.

## Example
Simultaneaous reading of ten Sensirion flow sensors via i2c (all have the same address!) 
<img src="doc/FlowSensors_Setup.jpg" width="800px">
<img src="doc/FlowSensors_Output.jpg" width="800px">

What, 10 sensors? I thought only up to 8 sensors can be used? Yes, you are right, but the Raspberry Pi offers two hardware i2c interfaces. Therfore, two i2c sensors (even both with the same address) can be connected to one RTMC-node. 

## Hints, tips and tricks
### Number of connected sensors
Up to eight RTMC-nodes can be connected to the RTMC-hub. The number of connected nodes does not affect sampling time. To each RTMC-node, more than one sensor can be connected. However, with every additional value to be transmitted, the maximum sampling frequency is reduced. 

## Bugs and known errors

### Terminal
The well-known "putty"-client is not a good fit for RTMC. At high data rates, putty's receive buffer overflows, which results in a crash of RTMC after a few seconds, as the Teensy's USB output buffer cannot be emptied. I recommend using [Coolterm](https://freeware.the-meiers.org/).

### RTMC-hub
1. The Teensy must be connected to the host PC directly via its USB port. Make sure to use a micro-USB cable with a small connector, as there is not much room between the rotary knob and the Teensy. 
2. The gate resistor for the FETs driving the port's green LEDs (Q11, Q21,...) are missing. Some of the FETs at my board are already dead, the LEDs don't light up anymore

I am currently working on a new layout in KiCad to resolve those issues. None of those issues, however, affect RTMC's functionality or performance.

## Further ideas
I am more than happy to hear your ideas. Here are mine so far:

- implement output to SD-card
- implement stand-alone usage of RTMC-nodes (to capture data without connecting it to the RTMC-hub)
- GUI for easy control and plotting of sensor data
- RTMC-nodes with
  - ... dedicated AD-converter for better analog performance
  - ... DAC for analog output
  - ... other standard interfaces, such as 4...20 mA, RS232, Profibus, CAN,...
- RTMC-hub V2.0 with fixed bugs and display interface to improve stand-alone usage (without a PC connected)

   
## License and Legal
Copyright 2024 by Markus Grün

Software licensed under MIT-license, see license file in `Software`-folder

Hardware (Schematics, Layout, etc.) licensed under CERN-OHL-P v2 or later, see license file in `Hardware`-folder

THE SOFTWARE AND SOURCES ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR SOURCES OR THE USE OR OTHER DEALINGS IN THE SOFTWARE OR SOURCES.






