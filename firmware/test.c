#ifdef F_CPU
#undef F_CPU
#endif
#define F_CPU 125000 //clock speed after CLKPR settings

//#define SIMULATION

#include <avr/io.h>         //needed for port definitions of MCU
#include <avr/sleep.h>      //needed for sleep_cpu(), which in turn is needed for simulation
#include <avr/interrupt.h>  //needed for sei()
//#include <util/delay.h>
#include <util/parity.h>    //needed for parity calculation
#include <avr/pgmspace.h>   //needed for reading arrays stored in flash
#include "../../simavr/simavr/sim/avr/avr_mcu_section.h"    //needed for simulation only

AVR_MCU(F_CPU, "attiny13a");
extern void emulateid0(void);
extern void emulateid1(void);
extern void emulateid2(void);

//Struct for logging values during simulation
const struct avr_mmcu_vcd_trace_t _mytrace [] _MMCU_ =
{
{ AVR_MCU_VCD_SYMBOL ( " DDRB_ " ) , . what = ( void *) & DDRB  }
};

void setup(void)
{
    //Configure GPIOs connected to switches as inputs with internal pull-ups
    PORTB |= _BV(DDB2) | _BV(DDB1) | _BV(DDB0);
}


void loop(void)
{
    if(!(PINB & _BV(PINB0)))
    {
        emulateid0();
    }
    if(!(PINB & _BV(PINB1)))
    {
        emulateid1();
    }
    if(!(PINB & _BV(PINB2)))
    {
        emulateid2();
    }
}

int main(void)
{
    setup();
#ifndef SIMULATION
    while(1)
    {
        loop();
    }
#else
    emulateid0();
    // this quits the simulator, since interupts are off
    // this is a "feature" that allows running test cases and exit
    cli();
    sleep_cpu();
#endif
    return 0;
}


