/*-----------------------------------------------------------------------------
    blink_app (loaded via bootloader)
    build with ld option-
        -Wl,-section-start=.text=0x400
        (byte address 0x800, word address 0x400)

    convert elf to bin format for xmodem use-
    avr-objcopy -binary my_project.elf -O my_project.bin

    Linux command line-
    $ stty -F /dev/ttyACM1 230400
    $ sx my_project_bin < /dev/ttyACM1 > /dev/ttyACM1

    this example also allows using the pc to reset the mcu via the rx pin
    if any falling edge seen on the rx pin or the sw pin, the port irq will
    fire and either software reset (sw) or first erase the last byte in 
    eeprom (rx)

    if the sw pin is triggered, it will remain pressed long enough for the 
    bootloader to see, and will remain in the bootloader

    if the rx pin is triggered (via pc), we wil need to erase the last byte in 
    eeprom so the bootloader does not jump to the app
    this method will require the bootloader to load an app (or simply have the
    xmodem send a null file so an EOT is seen by the mcu) as the eeprom byte 
    will need to be set before the bootloader will jump to an app again, where 
    the sw pin method does not require programming (just power up again without 
    sw pressed)

    Linux command line (first trigger rx pin by sending 0xFF)-
    $ stty -F /dev/ttyACM1 230400
    $ printf "\xFF" > /dev/ttyACM1
    $ sx my_project_bin < /dev/ttyACM1 > /dev/ttyACM1

    the mcu will dump out sigrow, fuse, flash, eeprom data after programming
    and you can do whatever you wish with this data
    if you want to just get the data dump, have the xmodem software send a
    blank file-
    $ sx /dev/null < /dev/ttyACM1 > /dev/ttyACM1
    and then capture the dump data (not shown here how to do that)

    the format of the binary data dump is-
        addressL addressH lengthL lengthH data[0] ... data[length-1]
        where the addresses are memory mapped addresses
-----------------------------------------------------------------------------*/

#include <avr/io.h>
#include <stdbool.h>

#define F_CPU 3333333ul
#include <util/delay.h>

                //pins type
                typedef struct {
                    PORT_t* port;
                    uint8_t pin;
                    uint8_t pinbm;
                    bool onVal;
                    }
pin_t           ;

                //our pins- Led and Sw
                //Sw and UartRx falling edge will trigger a reset
                static const pin_t
Led             = { &PORTA, 3, 1<<3, 0 };
                static const pin_t
Sw              = { &PORTB, 7, 1<<7, 0 };
                static const pin_t
UartRx          = { &PORTB, 3, 1<<3, 0 };

                static volatile uint8_t* const
eeLastBytePtr   = (volatile uint8_t*)EEPROM_END;


                static void
softReset       () { CCP = 0xD8; RSTCTRL.SWRR = 1; } //software reset

                static void
init            ()
                {
                (&Sw.port->PIN0CTRL)[Sw.pin] = 0x08 | 0x03; //pullup on, falling edge sense        
                (&UartRx.port->PIN0CTRL)[UartRx.pin] = 0x08 | 0x03; //pullup on, falling edge sense
                asm("sei");
                }

                static void //ERWP, erase page, write page buffer
nvmWrite        () { CCP = 0x9D; NVMCTRL.CTRLA = 3; } //ERWP


                static void
eeAppOK         ()
                {
                *eeLastBytePtr = 0xFF; //write to eeprom page buffer, last eeprom byte
                nvmWrite(); //write eeprom (0xFF signifies to bootloader that flash is unprogrammed)
                while( NVMCTRL.STATUS & 2 ){} //ee is busy
                }

                //both pin irq's are on same port, so only PORTB isr
                __attribute(( signal, used )) void 
PORTB_PORT_vect()
                {
                uint8_t flags = PORTB.INTFLAGS; //both Sw and UartRx flags
                PORTB.INTFLAGS = flags; //clear in case not our pin
                if( 0 == (flags & (Sw.pinbm|UartRx.pinbm)) ) return; //not our pins
                //if sw pin, then just reset- bootloader will also see pin pressed and remain in bootloader
                //if UartRx pin, then need to erase last eeprom byte so bootloader does not jump to this app again
                if( flags & UartRx.pinbm ) eeAppOK();
                softReset();
                }

                static void
ledTog          () { Led.port->DIRSET = Led.pinbm; Led.port->OUTTGL = Led.pinbm; }

                int
main            (void)
                {

                init(); //setup irq pins to reset mcu

                while(1){
                    ledTog();
                    _delay_ms(100);
                    }

                }
