# Arduino audio spectrum visualizer / analyzer

![Main Image](Images/Main_photo.JPG)

This project allows you to make your own 1 band audio/music frequency spectrum analyzer/visualizer using an Arduino and other basic electronic components.
In this case I placed the audio analyzer in my half transparent closet and gave it a cool look with RGB LEDs which responds to the music I am currently playing using a microphone.
The expected target audience for this project is any music enthusiast with a basic understanding of electronics, Arduino and C programming. The components used in this project are fairly cheap to obtain and easy to assemble.

### Main features of the project

- Uses only the IRRemote.h library (for infrared control of the project) which is easy to install.
- Can display up to 7 colors that can be switched using a push button or a remote control.
- Mixes all the frequencies in your hearing range to one band as opposed to other similar projects that display the full spectrum.
- Prototype uses the IC 74HC595 to expand the output pins which also allows to connect higher current LEDs.
- Feeding audio to this audio analyzer is done using a MAX9814 which includes a microphone.
- The program uses dynamic average to ensure that the display will not get "stuck" on the minimum/maximum level of the LEDs with different sound volumes.

### Components required

1. 1 Arduino Nano or Uno (in this case I used Arduino Nano)
2. 3 Shift registers 74HC595 (You can probably use a different shift register with some minor changes to the code)
3. 1 MAX9814 - An integrated circuit with a microphone and a built in amplifier
4. 1 Infrared receiver module
5. 1 Push button
6. 1 10 kilo ohm resistor

### Schematic diagram

<p align="center">
<img alt="Schematic Diagram" src="Images/Schematics_diagram.png">
</p>
                           
To supply power to the circuit you can use the Arduino USB connector or the Power supply connector (on Arduino Uno).

### Description of the system

The Arduino board (ATmega328P) has a built in analog to digital converter (ADC) which is being used to convert the samples of the analog volume level to information that can be handled by the Arduino.

The digital samples are than mapped according to a dynamic average stored by the system to the corresponding LEDs.

The mapped data is than being transferred to the shift registers and then to the LEDs.

The loop is constantly reading the push button and infrared receiver and changes the color (up to 7 different colors) of the LEDs when the user decides.

Using the infrared receiver, you can turn the system on or off.

## Code

Code is available [here](Code/RGBSpectrumAnalyzer.ino)

### Watch this project in action

Examine this demo video


 [![Demo Video](http://img.youtube.com/vi/J6Y9mcG-ZyQ/0.jpg)](https://www.youtube.com/shorts/J6Y9mcG-ZyQ)
