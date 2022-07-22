/*-----------------------------------------------------------------------------
    example_app.c (loaded via bootloader) - 
      blink an led, allow resetting back into bootloader via switch or via
      rx falling edge (from a pc)
    
    build with ld option-
        -Wl,-section-start=.text=0x400
        (byte address 0x800, word address 0x400)
        this value will match what the bootloader BL_SIZE value is set to
        in terms of a byte address (in this case 2048 -> 0x800 -> 0x400 word)

    the following commands assume the program arguments are in the current
    directory, or in the path environment variable, and the file arguments
    are in the current directory, change as needed for your own needs
    
    convert elf to bin format for xmodem use-
    $ avr-objcopy -binary my_project.elf -O my_project.bin

    Linux command line-
    $ stty -F /dev/ttyACM1 230400
    $ sx my_project_bin < /dev/ttyACM1 > /dev/ttyACM1

    this example also allows using the pc to reset the mcu via the rx pin
    if any falling edge seen on the rx pin or the sw pin, the port irq will
    fire and either software reset (sw) or first erase the last byte in eeprom (rx)

    if the sw pin is triggered, it will remain pressed long enough for the
    bootloader to see, and will remain in the bootloader

    if the rx pin is triggered (via pc), we wil need to erase the last byte in
    eeprom so the bootloader does not jump to the app
    this method will require the bootloader to load an app as the eeprom byte will
    need to be set before the bootloader will jump to an app again, where the sw pin
    does not require programming (just power up again without sw pressed)

    Linux command line (first trigger rx pin by sending 0xFF)-
    $ stty -F /dev/ttyACM1 230400
    $ printf "\xFF" > /dev/ttyACM1
    $ sx my_project_bin < /dev/ttyACM1 > /dev/ttyACM1
-----------------------------------------------------------------------------*/

#include <avr/io.h>
#include <stdbool.h>

#define F_CPU 3333333ul
#include <util/delay.h>

                //pins type
                typedef struct {
                    PORT_t* port;
                    uint8_t pin;
                    bool onVal;
                    }
pin_t           ;

                //our pins- Led and Sw
                //Sw and UartRx falling edge will trigger a reset
                static const pin_t
Led             = { &PORTA, 3, 0 };
                static const pin_t
Sw              = { &PORTB, 7, 0 };
                static const pin_t
UartRx          = { &PORTB, 3, 0 };


                static void
softReset       () { CCP = 0xD8; RSTCTRL.SWRR = 1; } //software reset

                static void
init            ()
                {
                (&Sw.port->PIN0CTRL)[Sw.pin] = 0x08 | 0x03; //pullup on, falling edge sense
                (&UartRx.port->PIN0CTRL)[UartRx.pin] = 0x08 | 0x03; //pullup on, falling edge sense
                asm("sei");
                }

                static void
writePage       () { CCP = 0x9D; NVMCTRL.CTRLA = 3; } //ERWP, erase page, write page buffer

                static void
eeBLsignal      ()
                {
                *(volatile uint8_t*)EEPROM_END = 0xFF; //write to eeprom page buffer, last eeprom byte
                writePage(); //write eeprom (0xFF signifies to bootloader that flash is unprogrammed)
                while( NVMCTRL.STATUS & 2 ){} //ee is busy
                }

                //both pin irq's are on same port, so only PORTB isr
                __attribute(( signal, used )) void
PORTB_PORT_vect()
                {
                uint8_t flags = PORTB.INTFLAGS; //both Sw and UartRx flags
                PORTB.INTFLAGS = flags; //clear in case not our pin
                if( 0 == (flags & ((1<<Sw.pin)|(1<<UartRx.pin))) ) return; //not our pins
                //if sw pin, then just reset- bootloader will also see pin pressed and remain in bootloader
                //if UartRx pin, then need to erase last eeprom byte so bootloader does not jump to this app again
                if( flags & (1<<UartRx.pin) ) eeBLsignal();
                softReset();
                }

                static void
ledTog          ()
                {
                Led.port->DIRSET = 1<<Led.pin;
                Led.port->OUTTGL = 1<<Led.pin;
                }

                int
main            (void)
                {

                init(); //setup irq pins to reset mcu

                while(1){
                    ledTog();
                    _delay_ms(100);
                    }

                }
