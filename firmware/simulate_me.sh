#/bin/bash
~/git/simavr/simavr/run_avr -m attiny13a -t -v avrfid.elf
gtkwave gtkwave_trace.vcd
