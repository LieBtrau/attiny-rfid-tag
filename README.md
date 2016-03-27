# attiny-rfid-tag
[Original implementation](http://scanlime.org/2008/09/using-an-avr-as-an-rfid-tag/) 
* Memory usage: 580 bytes flash, 0 bytes RAM.
* No possibility to choose which ID to send.
* Runs on the 125KHz clock from the RFID-reader
* Written in assembly. 

This implementation:
* Memory usage: 1634 bytes of flash and 0 bytes of RAM.  The above program has been implemented actually three times.  A C implementation where the 125KHz clock was used to clock the internal timer0 and the AVR ran on an internal 300KHz clock yielded disappointed reading range and performance.
* Three buttons to choose which ID-code to send.  More buttons could be added.

# Info
* [SimAvr](https://github.com/buserror/simavr)
	Simulation command: ~/git/simavr/simavr/run_avr -m attiny13a -t -v avrfid.elf
	The simulation doesn't notice changing clock dividers at run-time.
* [Atmel AVR4027: Tips and Tricks to Optimize Your C Code for 8-bit AVR Microcontrollers](http://www.atmel.com/Images/doc8453.pdf)
* [Engbedded Atmel AVR® Fuse Calculator](http://www.engbedded.com/fusecalc)

# RDM630
Clocks at 127KHz, one manchester halfbit takes 249-251us.

# Hardware
View it on [Circuitmaker](https://workspace.circuitmaker.com/Projects/4B27023E-4BFD-408E-A568-20982BFBAB32)

