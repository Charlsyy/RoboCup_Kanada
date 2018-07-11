/* 
 * File:   Init.h
 */

#ifndef INIT_H
#define	INIT_H

void init(void) {
    //Benutzte Analogchannel: 0..4 PORTA,5..7 PORTE,26 PORTC,(24..27 PORTD)
    PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0;
    LATA = 0; LATB = 0; LATC = 0; LATD = 0; LATE = 0;
    //TRIS 1=Input, 0=Output, ANSEL 1=ana, 0=digi
    TRISA = 0b11101111;//7..6unimpl.,5Lin/AbSe,4LED,3..0Lin/AbSe 
    ANSELA= 0b00101111;//7..6unimpl.,5Lin/AbSe,4LED,3..0Lin/AbSe
    //POTRB
    TRISB = 0b11100101;//7..6ICSP,5..4US-Echo,3..2n.b.,1Encoder,0IR/Servo
    ANSELB= 0b00000000;//Alle digital, ausser evtl. RB0 als IR
    WPUB  = 0b00100001; //Pull Up für Silbersensor, Encoder(ausserdem OPTION_REG Bit 7 = 0, s.u.)
    IOCBP = 0b00000001; //INTERRUPT-ON-CHANGE PORTB POSITIVE EDGE REGISTER
    IOCBN = 0b00100000;//INTERRUPT-ON-CHANGE PORTB NEGATIVE EDGE REGISTER(2..0US-Echo))
    IOCBF = 0b00000000;//INTERRUPT-ON-CHANGE PORTB FLAG REGISTER
    //
    TRISC = 0b11111001;//7Farb/Abst-Sensor,6..5Servo,4..3I2C,2..1PWM,0Taster
    ANSELC= 0b11100000;//7Farb/Abst-Sensor,6..5Servo,4..3I2C,2..1PWM,0Taster
    TRISD = 0b11010000;//5US-Trig,3..0Motor 
    ANSELD = 0b11000000;//7..6,4Ana,5US-Trig,3..0Motor
    TRISE = 0b00001111;//7..4unimp.,3MCLR,2..0Lin/AbSe
    ANSELE = 0b00000111;//7..4unimp.,3MCLR,2..0Lin/AbSe
    //
    ADCON0 = 0b00000001; //7unimp.,CHS4 CHS3 CHS2 CHS1 CHS0(00000)=Channel 0(0-27), GO/DONE, ?, ADON Ein
    ADCON1 = 0b00100000; //7Left justified,6..4FOSC/32,3..2unimp.,1..0VREF is connected to VDD
    OPTION_REG = 0b00000011; // WPU enabled,Prescaler 1:16
    INTCON = 0b01101000; //GIE PEIE TMR0IE INTE(RB0) IOCIE(RB7..0) TMR0IF INTF IOCIF
    PIE1 = 0b00000000; //TMR1GIE ADIE RCIE TXIE SSPIE CCP1IE TMR2IE TMR1IE
    PIR1 = 0b00000000; //TMR1GIF ADIF RCIF TXIF SSPIF CCP1IF TMR2IF TMR1IF
    PIE2 = 0b00000000; //OSFIE ? ? ? BCLIE ? ? CCP2IE (wird wohl nicht gebraucht)
    PIR2 = 0b00000000; //OSFIF ? ? ? BCLIF ? ? CCP2IF (wird wohl nicht gebraucht)
    T1GCON = 0b00000000; //Des brauchts net
    //6.12 Determining the Cause of a Reset S.67
    //PCON = 0b00000000;//Zum resetten RESET (asm)
    T1CON = 0b00000001; //TMR1ON = T1CON,0 ,TMR1 einschalten, Prescaler 1:1
    TMR0 = 0; //Muss, weiß nicht warum
    SSPSTAT = 0b10000000; //Slew rate control disabled for
			 //standard speed mode (100 kHz, 1MHz)    
    SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz    
    SSPCON1 = 0b00101000; // 5 Enable serial port(SSPEN),I2C Master-Mode
    SSPIF = 0; 		// Clear the serial port interrupt flag
    BCLIF = 0;        	// Clear the bus coll interrupt flag
    //PWM-Mode s.S166
    PR2 = 255; //Faktor für PWM-Periode
    CCPR1L = 255; //  1/2 Duty, diese 8Bit und LSB1,LSB0 von CCP1CON * Presc. TMR2
    CCP1CON = 0b00001100; //7..6 unim.,LSB1,LSB0(vom Duty),11xx PWM-Mode
    CCPR2L = 255; //  1/2 Duty, diese 8Bit und LSB1,LSB0 von CCP1CON * Presc. TMR2
    CCP2CON = 0b00001100; //7..6 unim.,LSB1,LSB0(vom Duty),11xx PWM-Mode
    T2CON = 0b00000111; //Prescaler 1:64, TMR2 disable, TMR2ON = 1 enables TMR2
}

#endif	/* INIT_H */

