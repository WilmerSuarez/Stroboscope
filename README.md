## Stroboscope

### Table of Contents

- [Introduction](#introduction)
- [Schematics](#schematics)
- [Demo Video](#demo-video)

## Introduction 

AVR Assembler was used to program a microcontroller to build a Stroboscope with the 
following features: 

- Plus and Minus 1 Frequency Adjustments
- Fine and Course Frequency Adjustments
- Double and Half Frequency Adjustments 

*A Stroboscope is an instrument used to study periodic motion or to determine the
speeds of rotation (rpm) of an object by using flashes of light so that the object
appears to be stationary.*

The system uses an ET MINI DC Motor (The object being measured) and 4 LEDs. 

[*Motor Connections: Figure1*](#figure1-et-mini-dc-motor-connections)

[*LED Connections: Figure2*](#figure2-led-connections)

The system is controlled using an Atmega324A microncontorller. The input to the MCU is
done using 8 pushbuttons and a 74HC148 8-Line to 3-Line Priority Encoder.
The MCU reads reads the pushbutton and executes the corresponding function. It uses
its built in 16-bit timer counter to control the flash frequency of the LEDs. 
The frequency of the motor was controlled by a Function Generator supplying a 1Khz
frequency pulse wave.

[*Final System Schematic: Figure3*](#figure3-final-system-schematic)

## Schematics

###### Figure1 ET MINI DC Motor Connections
![alt text](https://i.imgur.com/uynAaBQ.png "ET MINI DC Motor Connections")

###### Figure2 LED Connections
![alt text](https://i.imgur.com/3bPOhXl.png "LED Connections")

###### Figure3 Final System Schematic 
![alt text](https://i.imgur.com/kQRujtq.jpg "Final System Schematic")

## Demo Video

[![Project Demo](https://i.imgur.com/KZBAdsw.jpg)](https://www.youtube.com/watch?v=334mv07YeCo "Project Demo - Click to Watch")

The video shows the built system. The LCD screen shows the initial starting 
frequency of 1000Hz (60,000 RPM). The video then shows the motor and the LEDs.
The fundamental frequency is found and the white dot appears to be stationanry. To 
ensure that the frequency found is the fundamental frequency, the frequency at which 
the dot is stationary is doubled. If the doubled frequency shows two stationary dots
then the frequency found is the fundamental frequency and not a subharmonic. This 
can also be ensured by comparing the frequency of the LED flashes and the frequency at 
the optical outputs of the sensors on the motor board. The fundamental frequency would
show two pulses at the optical outputs for every LED pulse. This is shown in the shown
in the image below. 

![alt text](https://i.imgur.com/nj2mans.png "Oscilloscope Fundamental Frequency")