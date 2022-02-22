//
// FILE     main.c
// DATE     140128
// WRITTEN  RAC
// PURPOSE  Utilities to communicate with FM receiver IC.   $
// LANGUAGE MPLAB C18
// KEYWORDS USB/I2C/SPARKFUN/FM_MODULE/RECEIVER
// PROJECT  FM TxRx Lab experiment
// CATEGORY UTILITY
// TARGET   Darwin
//
//


// This is skeletal code which won't run until 'fleshed out'.
// It does give an idea of the setup and initialization required
// of the PIC and the FM tuner module.  The basics of communication
// over the I2C bus are also covered.
//
// This version contains eevisions needed to compile under MPLABX.
//
// PIC is highly configurable to meet various needs. For this project
// we need to enable the LCD segments and I2C bus. The following
// code and also in Init() configure PIC for our need in this project.
// You should not change the configuration unless you modify the
// hardware design.


#pragma config OSC = INTIO7  // Internal osc, RA6=CLKO, RA7=I/O
#pragma config FCMEN = OFF   // Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF    // Oscillator Switchover mode disabled
#pragma config WDT = OFF     // WDT disabled (control through SWDTEN bit)
#pragma config PWRT = OFF    // racmod  -> PWRT disabled
#pragma config MCLRE = ON    // MCLR pin enabled; RG5 input pin disabled
#pragma config XINST = OFF   // Instruction set extension disabled
#pragma config BOREN = OFF   // Brown-out controlled by software
#pragma config BORV = 3      // Brown-out voltage set for 2.0V, nominal
#pragma config STVREN = OFF  // Stack full/underflow will not cause Reset
#pragma config CP = OFF      // Program memory block not code-protected



#include <plib/i2c.h> // If compiler could not find this header file
                      // check if the legacy peripheral library
                      // is installed properly

#include "fm.h"
#include "LCD.h"
#include <stdio.h>

// FM register bank defaults -
const unsigned int regDflt[18] = {
        0xFFFF, // R0 -- the first writable register .  (disable xo_en)
        0x5B15, // R1.
        0xD0B9,     // R2.
        0xA010,         // R3   seekTHD = 16
        0x0780,         // R4
        0x28AB,         // R5
        0x6400,         // R6
        0x1EE7,         // R7
        0x7141,         // R8
        0x007D,         // R9
        0x82C6,         // R10  disable wrap
        0x4F55,         // R11. <--- (disable xo_output)
        0x970C,         // R12.
        0xB845,         // R13
        0xFC2D,         // R14
        0x8097,         // R15
        0x04A1,         // R16
        0xDF6A         // R17
};


// We keep a copy of AR1010 register data in regImg[].
// When writing to AR1010, we take the value from regImg
// and write to AR1010 via I2C.
unsigned int regImg[18]; // FM register bank images


/*
 * Obtain latest change in state for the pushbutton set.
 *
 * @param butn Which button changed.  See fm.h.
 *
 * @return  0 if no button has changed state,
 *			1 if button is pushed,
 *			2 if button is released.
 *
 */
unsigned char butnEvent(unsigned char *butn) {

        unsigned int but, i;
        unsigned int ports[8];

        ports[0] = PORTBbits.RB0;
        ports[1] = PORTBbits.RB5;
        ports[2] = PORTAbits.RA0;
        ports[3] = PORTAbits.RA1;
        ports[4] = PORTGbits.RG0;
        ports[5] = PORTGbits.RG1;
        ports[6] = PORTGbits.RG2;
        ports[7] = PORTGbits.RG3;

        for(i=0; i<8; i++) {
                but = ports[i];
                if (but==0) {
                        dly(100);
                        if (but==0) {
                                *butn = i;
                                return 1;
                        }


                }
        }

        return 0;         // No changes
}
//
// end butnEvent ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void dly(int d) {
        int i = 0;

        for ( ; d; --d)
                for (i = 100; i; --i);
}
//
// end dly ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

/*
 * Set all LCD segments to 0 (off, clear).
 *
 */
void clrscn() {

        int i = 0;
        unsigned char *CLEARptr;         // Pointer used to clear all LCDDATA


        for ( i = 0,
              CLEARptr = (unsigned char *) &LCDDATA0;               // Point to first segment
              i < 28;
              i++)               // Turn off all segments
                *CLEARptr++ = 0x00;
}
//
// end clrscn ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//

void Init() {

        int i;

        OSCCON = 0b01110010;          // Select 8 MHz internal oscillator
        LCDSE0 = 0b11111111;          // Enable  LCD segments 07-00
        LCDSE1 = 0b11111111;          // Enable  LCD segments 15-08
        LCDSE2 = 0b11111111;          // Enable  LCD segments 23-16
        LCDSE3 = 0b00000000;          // Disable LCD segments 31-24
        LCDCON = 0b10001000;           // Enab LC controller. Static mode. INTRC clock
        LCDPS  = 0b00110110;           // 37 Hz frame frequency
        ADCON1 = 0b00111111;          // Make all ADC/IO pins digital
        TRISA = 0b00000011;              // RA0 and RA1 pbutton
        TRISB = 0b00100001;              // RB0 and RB5 pbutton
        TRISC = 0b00011000;              // RC3 and RC4 do the I2C bus
        TRISG = 0b11111111;              // RG0, RG1 & RG3 pbutton
        PORTA = 0;
        PORTB = 0;
        PORTC = 0;
        INTCONbits.TMR0IF = 0;           // Clear timer flag
        //T0CON = 0b00000011;           // Prescale by 16
        T0CON = 0b00001000;              // No prescale
        TMR0H = 0;                       // Clear timer count
        TMR0L = 0;
        T0CONbits.TMR0ON = 1;            // Start timer
        OpenI2C( MASTER, SLEW_OFF);
        SSPADD = 0x3F;
}
//
// end Init ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//





void lcd_send_data (unsigned char data)
{
        unsigned char data_l, data_u;
        data_l = (data<<4)&0xf0;         //select lower nibble by moving it to the upper nibble position
        data_u = data&0xf0;         //select upper nibble

        StartI2C();
        WriteI2C (SLAVE_ADDRESS_LCD);
        WriteI2C (data_u|0x0D);         //enable=1 and rs =1
        WriteI2C (data_u|0x09);         //enable=0 and rs =1

        WriteI2C (data_l|0x0D);         //enable =1 and rs =1
        WriteI2C (data_l|0x09);         //enable=0 and rs =1

        StopI2C();
}

void lcd_send_cmd (unsigned char data)
{
        unsigned char data_l, data_u;
        data_l = (data<<4)&0xf0;         //select lower nibble by moving it to the upper nibble position
        data_u = data&0xf0;         //select upper nibble

        StartI2C();
        WriteI2C (SLAVE_ADDRESS_LCD);
        WriteI2C (data_u|0x0C);         //enable=1 and rs =0
        WriteI2C (data_u|0x08);         //enable=0 and rs =0

        WriteI2C (data_l|0x0C);         //enable =1 and rs =0
        WriteI2C (data_l|0x08);         //enable=0 and rs =0

        StopI2C();
}

lcd_clear (void)
{
        lcd_send_cmd (LINE_OFFSET);
        for (int i=0; i<60; i++)
        {
                lcd_send_data (' ');
        }
        lcd_send_cmd (SECOND_LINE);
        for (int i=0; i<60; i++)
        {
                lcd_send_data (' ');
        }

}

// initialise LCD

void lcd_init (void)
{
        lcd_clear();        // clear lcd contents and DRAM as well 0x02 returns home but doesn't clear DRAM
        lcd_send_cmd (RETURN_HOME);
        lcd_send_cmd (FOUR_BIT_TWO_LINE);
        lcd_send_cmd (DISPLAY_ON_CSR_OFF);
        lcd_send_cmd (LINE_OFFSET);
}

// send string to LCD

void lcd_send_string (char *str)
{
        while (*str) lcd_send_data (*str++);
}


/*
 * Write an individual LCD segment.
 *
 * @param segOrd The segment ordinal.  Between 0 and 22.
 *
 * @param state Whether to turn the segment dark (true) or clear (false).
 *
 */
void segWrt(unsigned char segOrd,  unsigned char state) {

        unsigned char bitSelect;
        unsigned char *LCReg;

        if (segOrd > 23) return;
        LCReg = (unsigned char *)&LCDDATA0 + (segOrd >> 3);
        bitSelect = 1 << (segOrd & 0x07);
        if (state) *LCReg  |=  bitSelect;        // Segment on
        else *LCReg &= ~bitSelect;          // Segment off

        // The above is an optimized implementation of concisely writing
        // a segment to the appropriate bit onto the right LCDDATA register
        // which can be LCDDATA0, LCDDATA1 (accessed by LCDDATA0+1), or
        // LCDDATA2 (accessed by LCDDATA0+2).
        // More explicit (and readable, but slower) logic is:
        /*
           if (segOrd==0)      { LCReg=&LCDDATA0; bitSelect=0b00000001; }
           else if (segOrd==1) { LCReg=&LCDDATA0; bitSelect=0b00000010; }
           else if (segOrd==2) { LCReg=&LCDDATA0; bitSelect=0b00000100; }
           ...
           else if (segOrd==7) { LCReg=&LCDDATA0; bitSelect=0b10000000; }
           else if (segOrd==8)  { LCReg=&LCDDATA0+1; bitSelect=0b00000001; }
           else if (segOrd==9)  { LCReg=&LCDDATA0+1; bitSelect=0b00000010; }
           else if (segOrd==10) { LCReg=&LCDDATA0+1; bitSelect=0b00000100; }
           ...
           else if (segOrd==15) { LCReg=&LCDDATA0+1; bitSelect=0b10000000; }
           else if (segOrd==16)  { LCReg=&LCDDATA0+2; bitSelect=0b00000001; }
           else if (segOrd==17)  { LCReg=&LCDDATA0+2; bitSelect=0b00000010; }
           ...
           else if (segOrd==22)  { LCReg=&LCDDATA0+2; bitSelect=0b01000000; }

           if (state) *LCReg |= bitSelect; // Paste 1 to the selected bit in LCReg
           else *LCReg &= ~bitSelect;      // Mask out the selected bit from LCReg
         */
}
//
// end segWrt ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMwrite() -  Write a two byte word to the FM module.  The new
 * register contents are obtained from the image bank.
 * The content must be ready in regImg[adr] before calling FMwrite(adr).
 *
 * @param adr The address of the register in the FM module that needs
 * to be written.
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMwrite(unsigned char adr) {

        unsigned int regstr;
        unsigned char firstByt;
        unsigned char secndByt;
        unsigned char rpy;

        firstByt = regImg[adr] >> 8;
        secndByt = regImg[adr];

        StartI2C();         // Begin I2C communication
        IdleI2C();

        // Send slave address of the chip onto the bus
        if (WriteI2C(FMI2CADR)) return XF;
        IdleI2C();
        WriteI2C(adr);         // Adress the internal register
        IdleI2C();
        WriteI2C(firstByt);         // Ask for write to FM chip
        IdleI2C();
        WriteI2C(secndByt);
        IdleI2C();
        StopI2C();
        IdleI2C();
        return XS;
}
//
// end FMwrite ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//



/*
 * FMread - Read a two byte register from the FM module.
 *
 * @param regAddr The address of the register in the module that needs
 *        to be read.
 *
 * @param data Where to store the reading.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMread(unsigned char regAddr, unsigned int *data) {

        unsigned char firstByt;
        unsigned char secndByt;

        StartI2C();         // Begin I2C communication
        IdleI2C();         // Allow the bus to settle

        // Send address of the chip onto the bus
        if (WriteI2C(FMI2CADR)) return XF;
        IdleI2C();
        WriteI2C(regAddr);         // Adress the internal register
        IdleI2C();
        RestartI2C();         // Initiate a RESTART command
        IdleI2C();
        WriteI2C(FMI2CADR + DEVRD);         // Ask for read from FM chip
        IdleI2C();
        firstByt = ReadI2C();         // Returns the MSB byte
        IdleI2C();
        AckI2C();         // Send back Acknowledge
        IdleI2C();
        secndByt = ReadI2C();         // Returns the LSB of the temperature
        IdleI2C();
        NotAckI2C();
        IdleI2C();
        StopI2C();
        IdleI2C();
        *data = firstByt;
        *data <<= 8;
        *data = *data | secndByt;

        return XS;
}
//
// end FMread ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 * FMready - See if the FM module is ready.
 *
 * @param rdy Where to store the busy/ready status.  Will become
 * non-zero if the chip is ready, zero if busy.
 *
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMready(unsigned int *rdy) {

        unsigned int sts;

        if (FMread(FMCHIPSTSADR, &sts)  != XS) return XF;
        sts &= FMASKSTATUS;
        *rdy = sts ? TRUE : FALSE;
        return XS;
}
//
// end FMready ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMinit() -  Initialise the FM module.
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMinit() {

        unsigned char ad;
        unsigned int dat;

        // Copy default FM register values to the image set -
        for(ad = 0; ad < 18; ad++) regImg[ad] = regDflt[ad];

        dat = regImg[0];
        regImg[0] &= ~1;
        if (FMwrite(0) != XS) return XF;
        for(ad = 1; ad < 18; ad++) {
                if (FMwrite(ad) != XS) return XF;
        }

        regImg[0] = dat | 1;
        if (FMwrite(0) != XS) return XF;
        dly(20);
        while (FMready(&dat), !dat) dly(2);
        return XS;
}
//
// end FMinit ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//

unsigned int VolChange(unsigned int vol, unsigned char dir){ //should be correct
        switch (dir) {
        case TRUE:
                if(vol < 18) {
                        vol++;
                        regImg[3] &= ~FMASKVOL1;
                        regImg[14] &= ~FMASKVOL2;
                        regImg[3] |= (Vol_data[0][vol] << 7);
                        regImg[14] |= (Vol_data[1][vol] << 12);
                        if(FMwrite(3) != XS) return XF;
                        if(FMwrite(14) != XS) return XF;
                        showFreq_vol(vol,FALSE);
                }
                else
                        vol = 18;
                break;

        case FALSE:
                if(vol > 0) {
                        vol--;
                        regImg[3] &= ~FMASKVOL1;
                        regImg[14] &= ~FMASKVOL2;
                        regImg[3] |= (Vol_data[0][vol] << 7);
                        regImg[14] |= (Vol_data[1][vol] << 12);
                        if(FMwrite(3) != XS) return XF;
                        if(FMwrite(14) != XS) return XF;
                        showFreq_vol(vol,FALSE);
                }
                else
                        vol = 0;
                break;

        default: break;
        }
        return vol;
}
//
// end VolChange ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//



/*
 * FMfrequenc(f) -  Tune the FM module to new frequency.
 *
 *
 * @param f The new frequency as a multiple of 100 kHz.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMfrequenc(unsigned int f) {

        unsigned int dat;
        unsigned int cn;         // AR1010 channel number

        cn = f - 690;

        // NB AR1010 retunes on 0 to 1 transition of TUNE bit -
        regImg[2] &= ~FMASKTUNE;            // Mask out bit D9 in R2 (see FMASKTUNE)
        if (FMwrite(2) != XS) return XF;         // Write R2 to AR1010
        regImg[2] &= 0xfe00;                // Clear bits D8-D0 in R2
        regImg[2] |= (cn | FMASKTUNE);         // Paste 1 to bit D9 (see FMASKTUNE)
                                               //   and also paste 'cn' onto D8-D0 in R2
        if (FMwrite(2) != XS) return XF;         // Write R2 to AR1010
        do {
                dly(2);
                if (FMready(&dat) != XS) return XF;
        } while (!dat);
}
//
// end FMfrequenc ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//


/*
 * FMvers - Obtain the FM chip version.
 *
 * @param vsn Where to store the version number.  Will become
 * 0x65B1 for vintage 2009 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMvers(unsigned int *vsn) {
        if (FMread(FMCHIPVERSADR, vsn)  != XS) return XF;
        return XS;
}
//
// end FMvers ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


/*
 * FMid - Obtain the FM chip ID.
 * * @param id Where to store the ID number.  Will become
 * 0x1010 for AR1010 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMid(unsigned int *id) {

        if (FMread(FMCHIPIDADR, id)  != XS) return XF;
        return XS;
}
//
// end FMid ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


unsigned int ManualChan(unsigned int channel, unsigned char dir) {
        if(dir == TRUE) {
                if(channel < 1080) {
                        channel++;
                        FMfrequenc(channel);
                        showFreq_vol(channel,TRUE);
                }
        }
        else if(dir == FALSE) {
                if(channel > 875) {
                        channel--;
                        FMfrequenc(channel);
                        showFreq_vol(channel,TRUE);
                }
        }
        return channel;
}



unsigned int StationPresets(unsigned int channel, unsigned int dir){
        int counter;

        switch(dir) {
        case TRUE:
                for(counter = 0; counter <= 3; counter++) {
                        if(channel < presetFreq[counter] || channel == presetFreq[3]) {
                                FMfrequenc(presetFreq[counter]);
                                showFreq_vol(presetFreq[counter],TRUE);
                                return presetFreq[counter];
                        }
                }
                break;

        case FALSE:
                for(counter = 3; counter >= 0; counter--) {
                        if(channel > presetFreq[counter] || channel == presetFreq[0]) {
                                FMfrequenc(presetFreq[counter]);
                                showFreq_vol(presetFreq[counter],TRUE);
                                return presetFreq[counter];
                        }
                }
                break;
        }
        return channel;
}

unsigned int SeekChannel(unsigned int channel, unsigned int dir) {

        unsigned int newchannel;
        dir <<= 15;
        unsigned int dat;

        regImg[1] |= FMASKMUTE; //Set hmute bit
        if(FMwrite(1) != XS) return XF;

        regImg[2] &= ~FMASKTUNE; //clear tune bit
        if(FMwrite(2) != XS) return XF;

        regImg[3] &= ~FMASKSEEK; //clear seek bit
        if(FMwrite(3) != XS) return XF;

        regImg[3] |= dir;       //set SEEKUP bit
        regImg[3] |= 0x2000;
        regImg[3] |= 0x0008;
        if(FMwrite(3) != XS) return XF;

        regImg[3] |= FMASKSEEK; //enable SEEK bit
        if(FMwrite(3) != XS) return XF;

        do {
                dly(2);
                if (FMready(&dat) != XS) return XF; //Ask if AR1010 is ready or not
        } while (!dat);

        regImg[1] &= ~FMASKMUTE; //clear hmute bit
        if(FMwrite(1) != XS) return XF;

        if(FMread(FMCHIPSTSADR, &newchannel) != XS) return XF;
        regImg[3] &= ~(0x8000);
        if(FMwrite(3) != XS) return XF;
        newchannel = newchannel >> 7;
        newchannel += 690;

        if(newchannel <= 1080 && newchannel >= 875) {
                FMfrequenc(newchannel);
                showFreq_vol(newchannel,TRUE);
                return newchannel;
        }
        else{
                return channel;
        }
}
//
// end nextChan ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//




/*
 * Display the frequency that the receiver chip is set to.
 *
 * @return XS if successful, else XF on failure.
 *
 */
void showFreq_vol(unsigned int freq_or_vol, unsigned char dir) {
        int i;
        int name = FALSE;
        int position;
        char str[5];
        char volume[2];


        lcd_clear();
        lcd_send_cmd(RETURN_HOME);
        if (dir == TRUE) {

                for(i=0; i<4; i++) {
                        if (freq_or_vol == presetFreq[i]) {
                                name = TRUE;
                                position = i;
                                break;
                        }
                }
                if(name == TRUE) {
                        lcd_send_string(presets[position]);
                        sprintf(str,"%.1f",(float)freq_or_vol/10);
                        lcd_send_cmd(SECOND_LINE);
                        lcd_send_string (str);
                }
                else{
                        lcd_send_string("Frequency");
                        sprintf(str,"%.1f",(float)freq_or_vol/10);
                        lcd_send_cmd(SECOND_LINE);
                        lcd_send_string (str);
                }
        }
        else if(dir == FALSE) {
                lcd_send_string("Volume");
                sprintf(volume,"%d",freq_or_vol);
                lcd_send_cmd(SECOND_LINE);
                lcd_send_string (volume);
        }

        else
                lcd_send_string("Display error");

}

//
// end showFreq ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//


void main(void) {
        unsigned char evt;
         unsigned int ui;
        unsigned char btn;
        unsigned int vol = 18;
        unsigned int channel = 1024;
        dly(20);
    
    Init();
    FMvers(&ui);                                    // Check we have comms with FM chip
    FMinit();
    //dly(20);
    lcd_init ();
    //vol = VolChange(vol-1, TRUE);
    showFreq_vol(channel,TRUE);
    FMfrequenc(channel);
      
        for (;;) {
                evt = butnEvent(&btn);
                if (evt == 1) switch (btn) {
                        case 0: channel = StationPresets(channel, TRUE); break;
                        case 1: channel = StationPresets(channel, FALSE); break;
                        case 2: channel = ManualChan(channel, TRUE); break;
                        case 3: channel = ManualChan(channel, FALSE); break;
                        case 4: vol = VolChange(vol, TRUE); dly(2000); showFreq_vol(channel,TRUE); break;
                        case 5: vol = VolChange(vol, FALSE); dly(5000); showFreq_vol(channel,TRUE); break;
                        case 6: channel = SeekChannel(channel, TRUE); break;
                        case 7: channel = SeekChannel(channel, FALSE); break;
                        default: break;
                        }
        }

}
//
// end main ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
