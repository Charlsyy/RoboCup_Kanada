/* 
 * File:   MLX_read.h
 * Author: a
 *
 * Created on 14. Februar 2018, 10:03
 */
//#define _XTAL_FREQ 20000000

extern char I2C_Add;
extern const char MLX_Slave;
extern char Register_R_MLX;
extern char Temperatur_LO_rechts;
extern char Temperatur_LO_links;
extern char Temperatur_HI_rechts;
extern char Temperatur_HI_links;
extern char Datum_S[20];

bit MLX_Slave_toggel; //Schaltet zwischen 0x5A und 0x5B um
char junk;

void warten_auf_SSP_flag (void){
    while(!SSPIF);
    SSPIF = 0;
}


#ifndef MLX_READ_H
#define	MLX_READ_H
void MLX_read (void) {
    SSPADD = 49; //MLX_Zeit: 100kHz I²C-Frequenz,20MHz/(4*(49+1))=100kHz (MLX Time)
    MLX_Slave_toggel = !MLX_Slave_toggel; //rechter, linker Sensor           
    I2C_Add = MLX_Slave + MLX_Slave_toggel; //I2C-Adresse vom Temp.Sensor   
    SSPIF = 0; //SSP InteruptFlag löschen
    SEN = 1; //Master übernimmt Bus, wird vom Chip gelöscht
        warten_auf_SSP_flag ();
    SSPBUF = (I2C_Add<<1) & 0b11111110; //Slaveadresse = 20[0010100X],
                                    //LSB=0-> Master schreibt
        warten_auf_SSP_flag ();
    SSPBUF = Register_R_MLX; //Zu beschreibendes Registeradresse übertragen
        warten_auf_SSP_flag ();
    RSEN =1;//Repeat start
        warten_auf_SSP_flag ();
    SSPBUF = (I2C_Add << 1) | 0b00000001;
        warten_auf_SSP_flag ();
    RCEN = 1; //Receive enable
        warten_auf_SSP_flag ();
    if (!MLX_Slave_toggel) {
        Temperatur_LO_rechts = SSPBUF; //Datum lesen
    } else {
        Temperatur_LO_links = SSPBUF; //Datum lesen
    }
    ACKEN = 1;
        warten_auf_SSP_flag ();
    RCEN = 1; //Receive enable
        warten_auf_SSP_flag ();
    if (!MLX_Slave_toggel) {
        Temperatur_HI_rechts = SSPBUF;
    } else {
        Temperatur_HI_links = SSPBUF;
        //Für die Auswertung
    }
    ACKEN = 1;
        warten_auf_SSP_flag ();
    RCEN = 1; //Receive enable
        warten_auf_SSP_flag ();
        junk = SSPBUF;
/***Ausprobieren***************************************************************/
    //ACKDT = 1; //Not Acknowledge, beendet die read-Sequenz
    ACKEN = 1; //Wert von ACKDT wird gesendet, 9. Flanke
        warten_auf_SSP_flag ();
/******************************************************************************/
    PEN = 1;
        warten_auf_SSP_flag ();
}

#endif	/* MLX_READ_H */

