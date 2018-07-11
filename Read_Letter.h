/* 
 * File:   Read_Letter.h
 * Author: a
 *
 * Created on 28. Juli 2017, 10:15
 */

extern char I2C_Add;
extern char I2C_Count;
extern bit I2C_Read;
extern char Datum_S[8];
extern char Datum_R[2];
extern const char PIC_Letter_1;
extern char Register_R[2];

bit Letter_Slave_toggel; //Schaltet zwischen 0x15 und 0x16 um

extern void warten_auf_SSP_flag (void);

#ifndef READ_LETTER_H
#define	READ_LETTER_H

void Read_Letter(void) {
    SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz (PICTime)
//    SSPADD = 11; //PIC16F1827_Zeit: 400kHz I²C-Frequenz,20MHz/(4*(11+1))~400kHz (PIC16F1827 Time)
    Letter_Slave_toggel = !Letter_Slave_toggel; //rechter, linker Sensor           
    I2C_Add = PIC_Letter_1 + Letter_Slave_toggel; //I2C-Adresse vom Letter Sensor
/***TEST***********************************************************************/    
    I2C_Add = PIC_Letter_1; //I2C-Adresse vom Letter Sensor
/******************************************************************************/
    SSPIF = 0; //SSP InteruptFlag löschen
    SEN = 1; //Master übernimmt Bus, wird vom Chip gelöscht
        warten_auf_SSP_flag ();
    SSPBUF = (I2C_Add << 1) & 0b11111110; //Slaveadresse = 104[1101000X],
                                //LSB=0-> Master schreibt
        warten_auf_SSP_flag ();
//        SSPBUF = 0;
    SSPBUF = Register_R[1]; //Registeradresse die ausgelesen werden soll
        warten_auf_SSP_flag ();
    RSEN =1; //Repeat Start Condition
        warten_auf_SSP_flag ();
    SSPBUF = (I2C_Add << 1) | 0b00000001; //LSB=1-> Master liest
        warten_auf_SSP_flag ();
    RCEN = 1; //receive enable
        warten_auf_SSP_flag ();
    Datum_R[1] = SSPBUF; //Hier ist RCEN wieder 0
/***************************************************************************/
    Datum_S[5] = Datum_R[1]; //Auf LCD anzeigen
/***************************************************************************/
    ACKDT = 1; //Not Acknowledge, beendet die Übertragung
    ACKEN = 1; //Acknowledge-Sequenz enable
        warten_auf_SSP_flag ();
    PEN = 1; //Master ist fertig
        warten_auf_SSP_flag ();
}

#endif	/* READ_LETTER_H */

