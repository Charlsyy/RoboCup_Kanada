/* 
 * File:   MLX_Neu.h
 * Author: a
 *
 * Created on 15. Februar 2018, 08:03
 */

#define _XTAL_FREQ 20000000
void I2C_an_MLX (char, char, char, char);

#ifndef MLX_NEU_H
#define	MLX_NEU_H
void MLX_Neu (void) {
    __delay_ms(100); //Spannung Zeit geben zum anliegen
    SSPADD = 49; //MLX_Zeit: 100kHz I²C-Frequenz,20MHz/(4*(49+1))=100kHz (MLX Time)
    I2C_an_MLX(0x2E, 0x00, 0x00, 0x6f);//MLX-Adresse 0x00, Genaral Call
    //EEPROM mit 0x0000 beschreiben, PEC = 0x67 Testwert
    __delay_ms(5);//min. 5ms warten, bis EEPROM beschrieben ist
    I2C_an_MLX (0x2E, 0x5A, 0x00, 0xE1);//MLX-Adresse 0x00, Genaral Call
    //EEPROM mit 0x0058 beschreiben, PEC = 0xCB Prüfwert
    //EEPROM mit 0x0059 beschreiben, PEC = 0xDE Prüfwert
    //EEPROM mit 0x005A beschreiben, PEC = 0xE1 Prüftwert
    //EEPROM mit 0x005B beschreiben, PEC = 0xF4 Prüfwert
    //Für Slave-Adresse 5A ist PEC = E1!!!!
    __delay_ms(10000); //10 s, genug Zeit, den Roboter wieder auszuschalten
}

/*Um ins EEPROM vom MLX zu schreiben muss das Register erst gelöscht werden
 EEPROM Register schreiben oder lesen 0b001xxxxx
 Ram Register 0b000xxxxx wobei xxxxx die Adresse des Registers ist
 */
//Datum an MLX übertragen
void I2C_an_MLX (char Kommando, char LO_Byte, char HI_Byte, char PEC){
    SEN = 1; //Master übernimmt Bus, wird vom Chip gelöscht
        warten_auf_SSP_flag ();
    SSPBUF = (0x00<<1) & 0b11111110; //Slaveadresse = 0x00[0000000X],
                                            //LSB=0-> Master schreibt
        warten_auf_SSP_flag ();
    SSPBUF = Kommando; //Zu beschreibende EEPROM- oder RAM-Adresse übertragen
        warten_auf_SSP_flag ();
    SSPBUF = LO_Byte; //LO-Byte übertragen
        warten_auf_SSP_flag ();
    SSPBUF = HI_Byte; //HI-Byte übertragen
        warten_auf_SSP_flag ();
    SSPBUF = PEC; //PEC
        warten_auf_SSP_flag ();
    PEN = 1; //Master ist fertig
        warten_auf_SSP_flag ();
}//

#endif	/* MLX_NEU_H */

