/*-----------------------------------------------------------------------------
    xmodem / xmodem-crc bootloader for avr0/1

    ATtiny3217 Curiosity Nano in use here

    --- [1] ---
    set user specified defines-
        cpu speed- 16 or 20MHz
        bootloader size
        xmodem-crc or normal xmodem
        uart baud rate

    --- [2] ---
    set fuse values as needed
        BOOTEND is calculated using define set above
        other fuses set as needed
        (the example is using mega4809 fuses, so change to fuses fr=or your mcu)

    --- [3] ---
    set led and sw pin values
        this bootloader uses a switch as one method to signal bootloader, so
        a switch is needed, and an led is used to notify bootloader status
        if some other scheme is desired, simply change the code to suite

    --- [4] ---
    set uart info
        if alternate pins are used, also fill in the UartAltPins function body
        to handle setting portmux to use the alternate pins
        a table is provided that maps out pins to uarts for all avr0/1

    --- [5] ---

-----------------------------------------------------------------------------*/



// --- [1] ---
//=============================================================================
#define FREQSEL     2           // OSC20M speed- 1==16MHz, 2=20MHz
#define BL_SIZE     2048        // value divisible by 256, used in FUSES
#define XMODEM_CRC  1           // 1=xmodem-crc, 0=standard xmodem
#define UART_BAUD   230400      // will be checked to see if possible
//=============================================================================


//=============================================================================
// will be using clock PDIV of 2, so will get either 10MHz or 8MHz which
// allows higher speeds for uart but still within speed limits for 3.3v power
#define F_CPU       (FREQSEL==2 ? 10000000ul : 8000000ul)
//=============================================================================
// check for valid define values
#if FREQSEL != 2 && FREQSEL != 1
#error "FREQSEL required to be 1 or 2"
#endif
#if BL_SIZE % 256
#error "BL_SIZE needs to be divisible by 256"
#endif
#if (F_CPU*4/UART_BAUD) < 64
#error "UART_BAUD value is too high for cpu speed"
#endif
//=============================================================================

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/delay.h>


// --- [2] ---
                //for an atmega4809, but close enough for the timy3217
FUSES           = {
                .WDTCFG = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
                .BODCFG = 0x00, // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHZ, LVL=BODLEVEL0}
                .OSCCFG = FREQSEL, // OSCCFG {FREQSEL=20MHZ(2)/16MHZ(1), OSCLOCK=CLEAR}
                .SYSCFG0 = 0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSRC=NOCRC}
                .SYSCFG1 = 0xFF, // SYSCFG1 {SUT=64MS}
                .APPEND = 0, // APPEND
                //boot starts at 0, goes up to bootend, app starts at bootend
                .BOOTEND = BL_SIZE/256, // BOOTEND
                };


                //pins type
                typedef struct {
                    PORT_t* port;
                    uint8_t pin;
                    bool onVal;
                    }
pin_t           ;

// --- [3] ---
                //our pins- Led and Sw

                static const pin_t
Led             = { &PORTA, 3, 0 };
                static const pin_t
Sw              = { &PORTB, 7, 0 };

                /*-----------------------------------------------------------
                    reference for uart pins, any avr0/1

                    -avr0 mega-
                             usart | 0   1   2   3
                              port | A   C   F   B
                    default pin tx | A0  C0  F0  B0
                    default pin rx | A1  C1  F1  B1
                  alternate pin tx | A4  C4  F4  B4
                  alternate pin rx | A5  C5  F5  B5

                    -avr0/1 tiny (usart0 only)-
                    default pin tx | B2
                    default pin rx | B3
                  alternate pin tx | A1
                  alternate pin rx | A2

                    -avr0/1 tiny 8pin (usart0 only)-
                    default pin tx | A6
                    default pin rx | A7
                  alternate pin tx | A1
                  alternate pin rx | A2
                -----------------------------------------------------------*/

// --- [4] ---
                //uart info

                static USART_t* const
Uart            = &USART0;
                static const pin_t
UartTx          = { &PORTB, 2, 0 }; //onVal value unimportant
                static const pin_t
UartRx          = { &PORTB, 3, 0 }; //onVal value unimportant

                //set function to handle enabling the alternate pins if needed
                //else leave as a blank function
                static void
UartAltPins     (){} // { PORTMUX.USARTROUTEA |= 1<<0; /*mega0 USART0 alt pins*/ }



                //constants

                static const uint8_t
NACK            = 0x15;
                static const uint8_t
ACK             = 0x06;
                static const uint8_t
SOH             = 0x01;
                static const uint8_t
EOT             = 0x04;
                static const uint8_t
PING            = XMODEM_CRC ? 'C' : 0x15; // C or NACK

                //vars

                uint8_t
xmodemData      [128]; //storage for an xmodem data packet (always 128 in size)


                //functions

                static bool
swIsOn          ()
                {
                (&Sw.port->PIN0CTRL)[Sw.pin] = 0x08; //pullup on
                _delay_ms(1); //give pullup time before we check switch
                return (Sw.port->IN & (1<<Sw.pin)) == Sw.onVal<<Sw.pin;
                }

                static void
ledOn           ()
                {
                Led.port->DIRSET = 1<<Led.pin;
                if(Led.onVal) Led.port->OUTSET = 1<<Led.pin;
                else Led.port->OUTCLR = 1<<Led.pin;
                }
                static void
ledTog          ()
                {
                Led.port->DIRSET = 1<<Led.pin;
                Led.port->OUTTGL = 1<<Led.pin;
                }

                static void
softReset       () { CCP = 0xD8; RSTCTRL.SWRR = 1; } //software reset
                static void
writePage       () { CCP = 0x9D; NVMCTRL.CTRLA = 3; } //ERWP, erase page, write page buffer
                static bool
isRxActive      () { return UartRx.port->INTFLAGS & (1<<UartRx.pin); } //we enabled falling edge sense, so any rx will set the rx intflag

                static bool
entryCheck      ()
                { //return true if we want to stay in bootloader
                //if last eeprom byte is erased, or if sw is pressed, return true
                return *(volatile uint8_t*)EEPROM_END == 0xFF || swIsOn();
                }

                static void
init            ()
                {
                CCP = 0xD8; CLKCTRL.MCLKCTRLB = 1; //prescale enable, div2
                Uart->BAUD = F_CPU*4/UART_BAUD;
                Uart->CTRLB = 0xC0; //RXEN,TXEN
                UartTx.port->DIRSET = 1<<UartTx.pin; //output
                (&UartRx.port->PIN0CTRL)[UartRx.pin] = 0x08|0x03; //pullup, falling edge sense
                UartAltPins(); //function to handle alternate pins if needed
                }

                static void
write           (const char c)
                {
                while( (Uart->STATUS & 0x20) == 0 ){} //DREIF
                Uart->TXDATAL = c;
                }
                static uint8_t
read            ()
                {
                while ( (Uart->STATUS & 0x80) == 0 ){}
                return Uart->RXDATAL;
                }

                //=================================================================
                #if !XMODEM_CRC //simple checksum

                static bool
xmodem          ()
                {
                while(1){
                    uint8_t c;
                    while( c = read(), c != SOH && c != EOT ){} //wait for SOH or EOT
                    if( c == EOT ) return false;
                    uint8_t blockSum = read() + read(); //block#,block#inv, sum should be 255
                    uint8_t checksum = 0;
                    for( uint8_t i = 0; i < 128; i++ ){
                        c = read();
                        packetData[i] = c;
                        checksum += c;
                        }
                    if( read() == checksum && blockSum == 255 ) break;
                    write(NACK); //bad checksum or block# pair not a match
                    }
                //we let caller ack when its ready for more data
                return true;
                }

                //=================================================================
                #else //xmodem-crc, checksum is now 2 bytes
                //=================================================================

                static uint16_t
crc16           () //crc the packetData array
                {
                uint16_t crc = 0;
                for( uint8_t i = 0; i < sizeof xmodemData; i++ ){
                    crc = crc ^ (xmodemData[i] << 8);
                    for( uint8_t j = 0; j < 8; j++ ){
                        bool b15 = crc & 0x8000;
                        crc <<= 1;
                        if (b15) crc ^= 0x1021;
                        }
                    }
                return crc;
                }

                static bool
xmodem          ()
                {
                while(1){
                    uint8_t c;
                    while( c = read(), c != SOH && c != EOT ){} //wait for SOH or EOT
                    if( c == EOT ) return false;
                    uint8_t blockSum = read() + read(); //block#,block#inv, sum should be 255
                    for( uint8_t i = 0; i < 128; i++ ) xmodemData[i] = read();
                    uint16_t crc = (read()<<8) + read(); //2 bytes, H,L
                    if( crc == crc16() && blockSum == 255 ) break;
                    write(NACK); //bad checksum or block# pair not a match
                    }
                //we let caller ack when its ready for more data
                return true;
                }

                #endif
                //=================================================================

                static void
programApp      ()
                {
                //the other end expects a NACK or 'C' (xmodem crc) when we are ready
                //we do not know if sender is ready yet (may not see our C/NACK),
                //so to get things started send out a C/NACK (PING) every second or so until
                //we see the first rx start bit
                while(1){
                    ledTog(); //blink when waiting for sender
                    write(PING);
                    uint32_t t = F_CPU/10; //count to wait (while loop about 10 clocks)
                    while( t-- && !isRxActive() ){}
                    if( isRxActive() ) break;
                    }
                ledOn(); //on when xmodem active (probably will not see for very long)
                volatile uint8_t* flashPtr = (volatile uint8_t*)(MAPPED_PROGMEM_START|BL_SIZE); //start of app (data mapped address)
                while( xmodem() ){
                    uint8_t i = 0;
                    uint8_t pg = 0;
                    //also handle avr0/1 with page size < 128 (64 is the only other lower value)
                    while( i < sizeof xmodemData ){ //128
                        flashPtr[i] = xmodemData[i]; //write to page buffer
                        i++;
                        if( ++pg < MAPPED_PROGMEM_PAGE_SIZE ) continue;
                        writePage();
                        pg = 0;
                        }
                    i = 0;
                    while( (flashPtr[i] == xmodemData[i]) && (++i < 128) ){} //verify
                    if( i == 128 ){ write( ACK ); flashPtr += 128; }
                    else write( NACK );
                    //if flash write failure- instead of retrying flash write on our own (we have the data),
                    //let the sender know there is an error so it is informed
                    //(it will send the data again, the sender will decide when its time to give up)
                    }
                write(ACK); //ack the EOT
                }

                static void
eeAppOK         ()
                {
                *(volatile uint8_t*)EEPROM_END = 0; //write to eeprom page buffer, last eeprom byte
                writePage(); //write eeprom (!0xFF signifies to bootloader that flash is programmed)
                while( NVMCTRL.STATUS & 2 ){} //ee is busy
                }

                int
main            (void)
                {

                //check if bootloader needs to run, true=run, false=jump to app
                //convert BL_SIZE to flash address mapped into data space
                //goto will result in a jmp instruction, so can use BL_SIZE as-is (byte address)
                if( entryCheck() == false ) goto *( (void*)BL_SIZE );

                //we are now officially a bootloader
                init();
                programApp();
                eeAppOK();              //mark that app is programmed
                while( swIsOn() ){}     //in case sw still pressed, wait for release
                softReset();
                while(1){}

                }





#if 0

/*-----------------------------------------------------------------------------
    blink_app (loaded via bootloader)
    build with ld option-
        -Wl,-section-start=.text=0x400
        (byte address 0x800, word address 0x400)

    convert elf to bin format for xmodem use-
    avr-objcopy -binary my_project.elf -O my_project.bin

    Linux command line-
    $ stty -F /dev/ttyACM1 115200
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
    $ stty -F /dev/ttyACM1 115200
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

#endif
