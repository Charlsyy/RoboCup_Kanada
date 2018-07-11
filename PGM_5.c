#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "Init.h"
#include "MLX_read.h"
//#include "MLX_Neu.h" //Muss aktiviert werden, wenn der T-Sensor eine neue Adresse bekommen soll (s.u.))

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ 20000000
//LATX ist ein Register, das die Ausgabe des zugehörigen PORTX regelt.
//PORTX ist der Zustand an den Pins
//LATX kann also anders sein als PORTX
#define Status LATA4 //Bit 4 des Registers LATA(=PORTA), 0 LED ein, 1 LED aus
#define Taster RD4 //!!!!!!!!!!war davor RC0
//#define vorwaerts LATD = LATD & 0b11110000 | 0b00001010 //Beim Inhalt des Registers
//LATD werden zunächst die unteren 4 Bits gelöscht und dann Bit 1 und 3 gesetzt.
//An PORTD liegt also dann an Pin 22 und 24 fünf Volt an, an Pin RD7..4 liegt der
//Pegel an, der auch vorher angelegen hat
//#define stopp LATD = LATD & 0b11110000
#define vorwaerts   LATD = (LATD & 0b11110000)|0b00001010
#define nachhinten  LATD = (LATD & 0b11110000)|0b00000101
#define rechts      LATD = (LATD & 0b11110000)|0b00000110
#define links       LATD = (LATD & 0b11110000)|0b00001001
#define stopp       LATD = (LATD & 0b11110000)
#define bissl       LATD = (LATD & 0b11110000)|0b00001000
#define bissr       LATD = (LATD & 0b11110000)|0b00000010
#define bisslrueck  LATD = (LATD & 0b11110000)|0b00000100
#define bissrrueck  LATD = (LATD & 0b11110000)|0b00000001

//Definitionen sinnvoll ergänzen: rueckwaerts, rechts_drehen....
#define US_Trigger  LATD5 //US-Trigger Pin an RD5, Pin 28
#define pwm_links   CCPR1L 
#define pwm_rechts  CCPR2L
#define servo       RB1

#define vornelinks A_D_Wert [0]
#define vornerechts A_D_Wert [3]
#define rechtsvorne A_D_Wert [2]
#define rechtshinten A_D_Wert [5]
#define hintenrechts A_D_Wert [4]
#define hintenlinks A_D_Wert [7]
#define linkshinten A_D_Wert [6]
#define linksvorne A_D_Wert [1]
#define schwarzlinks A_D_Wert [8] 
#define schwarzrechts A_D_Wert [9]  
#define vornelang A_D_Wert [10]

//#define IR_unten A_D_Wert[9]

//Deklaration der Variabeln, mit Wertzuweisung
char grad_90_1 = 69; //90° Drehung, Anzahl der Ticks am Encoder: einstellen!! // zusammen 84
char grad_90_2 = 12; // 2. langsame teil der Drehung
char grad_90links_1 = 65;// 90 grad drehung links, brauch andere ticks als rechts! //zusammen 77
char grad_90links_2 = 12; // 2. Teil von Links (langsam)
char cm_30 = 134; //30cm fahren, Anzahl der Ticks am Encoder: einstellen!!
const char PIC_Slave = 0x14; //20 PIC-Slave Adresse vom 16F876A
const char PIC_Letter_1 = 0x15; //21 0x15 und 0x16 sind PIC-Slave Adressen von den 16F1827
const char MLX_Slave = 0x5A; //Slave Adresse von einem T-Sensor, der andere hat
        //0x5B als Adresse, wird in der MLX-I2C Funktion aufgerufen
//Es stehen 512 Speicherplätze zur Verfügung
//Jeweils 80 Bytes pro Bank, 31 Bänke
//Diese sind in den Speicherplätzen 0x2000 - 0x29AF gespiegelt
//Liegen also hintereinander, was eine indirekte Adressierung vereinfacht

//Deklaration der Variabeln, ohne Wertzuweisung
char A_D_Wert [28]; //28 A/D-Wandler-Pins am 1519
char Channel [11] = {0, 1, 2, 3, 4, 5, 6, 7, 17, 18, 19}; //links vom Master ADKanal 0 - 7, rechts ADKanal 17 - 19
char Channel_Index;

char I2C_Add; //aktuell zu verarbeitende I2C-Adresse
char I2C_Index_w; //Write Index, aktuell zu verarbeitendes Register/Datum-Paar
char I2C_Index_R; //Read Index, aktuell zu verarbeitendes Register/Datum-Paar
bit  I2C_Write; //Ist der I2C-Bus gerade mit Schreiben beschäftigt?
bit  I2C_Read; //Ist der I2C-Bus gerade mit Lesen beschäftigt?
char I2C_Count; //Zähler der Schritte bei der I2C-Kommunikation
char Count; //Zähler für I2C_Index_w


char Datum_S[20]; //ans LCD zu übertragene Werte
char Datum1_an_Letter;
char Datum2_an_Letter;
bit  Flag_kalibrieren;
char Register_S; //LCD Tabulatorstelle
char Datum_R[3]; //vom 16F876A[0] und 16F1827[1] u [2] gelesene Werte
char Register_R_MLX = 0x07; // in 0x07 und 0x08 stehen LO- und HI-Byte der Temperatur

int Tick; //Zähler der TMR0 Überläufe im Interrupt
char m, n; //Zähler im Interrupt
char n_US; //Zähler für US-Trigger, 40ms
char Zeit_US; //Laufzeit des US-Signals
char Abstand_US; //Abstand vom US in cm (ca. 10% ungenau)
bit Flag_LED; //Soll die LED blinken (1) oder an sein (0)

char VAR_ausrichten_durchgaenge;
bit  Flag_Zeit1000ms;
bit  VAR_von_mitte_bis_wieder_wand ;
bit  VAR_von_sensor_VR_bis_mitte ;
bit  VAR_schwarz ;
char VAR_Schwarz_ausrichten;
char VAR_bis_wand_zeit ;
int  VAR_aus_zeit;
int  VAR_bis_mitte_zeit ;
int  VAR_zeit_opfer;
bit  VAR_schwarz_rechts ;
bit  VAR_opfer_aus;
int  VAR_rechts_zeit;
int  VAR_links_zeit ;
char VAR_omni = 60;
char Var_lang_Ausrichten;
char Ausrichten_Fahren = 5;
char Ausrichten_Stopp = 15;
int  Zeit_power_Ausrichten = 100;
char Schwellwert_Ausrichten = 36;
char Schwellwert_Omni = 61 ;
char VAR_im_Kreis_faren = 0 ;
char VAR_links_ist_nichts = 0 ;
char H = 'H';
char VAR_schwarz_ausrichten_merken;

char wand = 38;
char zuweit = 85; 
char zunah  = 75;
int  zeit;
char VAR_cm_f;


char I2C_Add; //aktuell benutzte I2C-Slave-Adresse
char Register_R[2];// = {0x07, 0xFF}; //in 0x07 und 0x08 stehen LO- und HI-Byte der Temperatur
        //0x00  oder 0xFF wird an "Letter" übertragen, 0 normal, 255 kalibrieren
char Temperatur_LO_rechts; //Den Wert brauchen wir nicht unbedingt
char Temperatur_LO_links; //Den Wert brauchen wir nicht unbedingt
char Temperatur_HI_rechts; //HI-Teil der Temperatur
char Temperatur_HI_links; //HI-Teil der Temperatur
char Waerme_links = 58;
char Waerme_rechts = 58;
char schwarz = 235;

bit Flag_Hindernis;
bit Flag_T_links;
bit Flag_T_rechts;
bit nicht_rechts = 0 ;

char Encoder; //Anzahl der Motordrehungen, Raddurchmesser 50mm -> ca. 6 Umdrehungen = 1cm
char encodersave;
int  Strecke; //Länge des zu fahrenden Wegs in Radumdrehungen
bit  Flag_Strecke_erreicht = 0; //die vorgegebene Strecke ist erreicht -> Interrupt
bit  Flag_Strecke_messen = 0; //Es wird gerade eine Strecke gemessen -> Interrupt

char z;



void init(void); //Chip initialisieren
void Strecke_fahren(char, char); //(Länge, Richtung)
void Letter_kal(void); //Buchstabensensoren kalibrieren
void Werte_auf_LCD_anzeigen (void);
void interrupt Interrupt(void); //sic


void anderwandlang(void); 
void rechts90grad (void);
void links90grad (void);
void ausrichten (void) ;
void vorne_ausrichten (void) ;
void hinten_ausrichten (void) ;
void links_ausrichten (void) ;
void rechts_ausrichten (void) ;
void von_sensor_VR_bis_mitte (void);
void von_mitte_bis_wieder_wand (void);
void von_mitte_bis_wieder_wand_in_schwarzeplatte(void);
void schwarzeplatte (void);
void cm_fahren (void) ;
void opfer (void) ;
void hindernis (void);
void kit_abwerfen (void);
void Power_Ausrichten(void);
void im_Kreis_faren (void);
void hintenausrichtenopfer (void);
void omnifakelinks (void);
void omnifakerechts (void);


void interrupt Interrupt(void); //sic
/*
 * Jetzt gehts los
 */

/*Hauptfunktion*/
void main(void) {
    init(); //Chip initialisieren, incl. I2C
//    while(1) {
//        MLX_Neu();
//    }
    servo=0;
    stopp; //Erstmal alle Motoren aus
    pwm_links = 255; //werte rumprobieren! 
    pwm_rechts = 155;
    
//    MLX_Neu(); //Dem Temperatursensor eine neue Adresse geben
    //Dazu müssen die Zeile 7 (#include "MLX_Neu.h")  und die Zeile hier drüber
    //aktiviert werden. Kurz, 1s, den Roboter einschalten und dann alles wieder
    //auskommentieren. Es darf dabei natürlich nur ein T-Sensor angeschlossen sein.
    
    //Zum Debuggen können Interrupts ein und aus geschaltet werden:
    TMR1IE = 0; //En/Disable TMR1-Interrupt
    IOCIE = 1; //En/Disable Interupt on Change, RB7..1/0
    TMR0IE = 1; //En/Disable TMR0 Interrupt
    SSPIE = 1; //En/Disable SSP Interrupt
    INTE = 0; //En/Disable Interrupt on RB0
    ADIE = 0; //En/Disable Interrupt on AD-Module
    //Flags zurück setzen
    I2C_Write = 0; //Ist der I2C gerade mit Schreiben beschäftigt?
    I2C_Read = 0; //Ist der I2C gerade mit Lesen beschäftigt?
    Flag_LED = 0; //Normales LED-Blinken aus
    
    Register_R[0] = 0x07; //Adresse im T-Sensor in der der T-Wert gespeichert ist
    Register_R[1] = 127; //Eigentlich würde ich ja 0 nehmen, aber dann bricht die Übertragung zusammen, k.A. warum
    
    __delay_ms(600); //Allen Sensoren Zeit lassen auf den Anfangswert einzupegeln
    GIE = 1; //GeneralInterruptEnable(/disable)
    __delay_ms(400); //die ersten Werte holen (dauert natürlich nicht 400ms)
    
    Status = 1;
    
    if(Taster){                         //wenn Taster am master gedrückt
        __delay_ms(2500);               // Pause damit er nicht direkt wieder raus springt
        while(!Taster){                 //solange Taster NICHT gedrückt wird
            Werte_auf_LCD_anzeigen();
            kit_abwerfen();
            __delay_ms(1000);
        }
        }
            
        
    
   
    
    //Für Tests:
    //char Z = 0;
    //Datum_S[0] = 123;
//    while(0) {
//        stopp;
//    }
/*Hier gehts los, nachdem der Taster am Master gedrückt wurde*/
    //Letter_kal();    
    while (1) { //Endlosschleife
        Flag_LED = 0; // LED aus, damit bei opfer an kann
        schwarzeplatte();
        anderwandlang() ;
        Werte_auf_LCD_anzeigen();
        opfer();
        hindernis();
        
        if (rechtsvorne < wand ) { //ist rechts keine Wand?
            Strecke_fahren(3,vorwaerts); //kontrollabfrage
                while (!Flag_Strecke_erreicht){
            }
            stopp;
            __delay_ms(50);
            if (rechtsvorne < wand ) {//ist da wirklich keine?
                VAR_im_Kreis_faren ++ ; 
                von_sensor_VR_bis_mitte(); //wenn ja -> keine Wand
                stopp;
                ausrichten();
                if (nicht_rechts == 0){ //wird gesetzt nach schwarzer platte
                    rechts90grad();
                    while (!Flag_Strecke_erreicht){
                        //opfer();
                    }
                    stopp;
                    if ((vornelinks < wand)&&(vornerechts < wand)){ //falls er sich dreht und vor eine Wand fahren würde.
                    ausrichten();
                    im_Kreis_faren(); //guckt ob er über 6 mal rechts ist und reagiert drauf
                    von_mitte_bis_wieder_wand(); //fährt bis rechts wand oder ticks zu Ende
                    }
                    else{
                        links90grad();
                        ausrichten();
                    }
                }
                nicht_rechts = 0 ;
            }
        }
        else{
            if((vornelinks > wand)&&(vornerechts > wand)){ //ist vorne eine Wand?
                stopp;
                __delay_ms(50);
                if((vornelinks > wand)&&(vornerechts > wand)){//Kontrollabfrage
                    if (linksvorne > wand){
                        ausrichten();
                        stopp;
                        links90grad();
                        while (!Flag_Strecke_erreicht);
                        stopp;
                        links90grad();
                        while (!Flag_Strecke_erreicht);
                        stopp;
                        ausrichten();
                    }
                    else{
                        VAR_im_Kreis_faren = 0 ; //wennn links, dann fährst du nicht mehr im KReis
                        ausrichten();
                        links90grad();
                        while (!Flag_Strecke_erreicht){
                            //opfer();
                        }
                        stopp;
                        ausrichten();
                    }
                }
                
            }
        }
    }
}

/*Kalibrieren der Buchstabensensoren, falls notwendig
 Taster am 16F876A einmal drücken -> kalibrieren, nochmal drücken -> es kann los gehen*/


void von_sensor_VR_bis_mitte(void) {
    stopp;
    Strecke_fahren(77, vorwaerts); //die strecke die er in ticks nacg vorne fährt (idealerweise auf die mitte der platte))
        while (!Flag_Strecke_erreicht){
            schwarzeplatte();
            opfer();
            hindernis();
        }
    stopp;
}
void von_mitte_bis_wieder_wand (void){
    VAR_von_mitte_bis_wieder_wand = 1;
    Encoder = 0 ;
    while ( VAR_von_mitte_bis_wieder_wand == 1){ // == ist Abfrage! 
        vorwaerts;
        if (rechtsvorne > wand){ // ist da eine wand rechts von dir?
            stopp;
             if (rechtsvorne > wand){ // ist da eine wand?
                VAR_von_mitte_bis_wieder_wand = 0 ; //Variable um aus der While zu gehen
                stopp ;  
            }
        }
        if (vornerechts > 80){ //Ist da eine Wand vor dir?
            stopp;
           __delay_ms (50);
           if (vornerechts > 80){ // ist da eine wand?
                VAR_von_mitte_bis_wieder_wand = 0 ;
                stopp ;
                __delay_ms(500);
            }
        }
        
        if (Encoder > 40 ){
            VAR_von_mitte_bis_wieder_wand = 0 ;
            VAR_bis_wand_zeit = 0;
            stopp ;
        }
        
        schwarzeplatte();
        opfer();
        hindernis();
    }
    stopp ;
}
void von_mitte_bis_wieder_wand_in_schwarzeplatte (void){
    VAR_von_mitte_bis_wieder_wand = 1;
    Encoder = 0 ;
    while ( VAR_von_mitte_bis_wieder_wand == 1){ // == ist Abfrage! 
        if (rechtsvorne > wand){ // ist da eine wand?
            Strecke_fahren(2,vorwaerts);
            while(!Flag_Strecke_erreicht);
            stopp;
            __delay_ms (50);
             if (rechtsvorne > wand){ // ist da eine wand?
                VAR_von_mitte_bis_wieder_wand = 0 ;     
                stopp ;  
            }
        }
        if (vornerechts > wand){
           __delay_ms (20);
           if (vornerechts > wand){ // ist da eine wand?
                VAR_von_mitte_bis_wieder_wand = 0 ;
                stopp ;  
            }
        }
        
        if (Encoder > 40 ){
            VAR_von_mitte_bis_wieder_wand = 0 ;
            VAR_bis_wand_zeit = 0;
            stopp ;
        }
        
        opfer();
        hindernis();
        vorwaerts ;
        
        
    }
    stopp ;
}

void rechts90grad (void){
Strecke_fahren(grad_90_1, rechts);
while (!Flag_Strecke_erreicht){
    //opfer();
}
stopp;
Strecke_fahren(grad_90_2, rechts);
while (!Flag_Strecke_erreicht){
    stopp;
    __delay_ms(10);
    rechts;
    __delay_ms(10);
    //opfer();
}
}
void links90grad (void){
    Strecke_fahren(grad_90links_1, links);
    while (!Flag_Strecke_erreicht){
        //opfer();
    }
    stopp;
    Strecke_fahren(grad_90links_2, links);
    while (!Flag_Strecke_erreicht){
        stopp;
        __delay_ms(10);
        links;
        __delay_ms(10);
        //opfer();
    }
}
void omnifakelinks(void){
    if((linksvorne > Schwellwert_Omni)&&(linkshinten > Schwellwert_Omni)){
        VAR_omni = 0;
        while((((linksvorne + linkshinten )/2)<=78)&&(!VAR_omni < 6)){ //solange durchschnitt aus lv und lh nicht größer als 80...
             bissl;                                                    // und die schleife nicht öfter als 6x durch (1x 195ms) mach omni
            __delay_ms(50);
            stopp;
            bissr;
            __delay_ms(55);
            stopp;
            bisslrueck;
            __delay_ms(50);
            stopp;
            bissrrueck;
            __delay_ms(50);
            stopp;
            VAR_omni++;
        }
        stopp;
        VAR_omni = 0;
        while((((linksvorne + linkshinten )/2)>=80)&&(!VAR_omni < 6)){
            bissr;
            __delay_ms(52);
            bissl;
            __delay_ms(38);
            bissrrueck;
            __delay_ms(42);
            bisslrueck;
            __delay_ms(42);
            VAR_omni++;
        }
        VAR_omni = 0;
        stopp;
    }
    
    
}
void omnifakerechts (void){
    if((rechtsvorne > Schwellwert_Omni)&&(rechtshinten > Schwellwert_Omni)){
        VAR_omni = 0;
        while((((rechtsvorne + rechtshinten )/2)<=78)&&(!VAR_omni < 6)){ //solange durchschnitt aus lv und lh nicht größer als 80...
            bissr;                                                       // und die schleife nicht öfter als 6x durch (1x 195ms) mach omni
            __delay_ms(52);
            bissl;
            __delay_ms(38);
            bissrrueck;
            __delay_ms(42);
            bisslrueck;
            __delay_ms(42);
            VAR_omni++;
        }
        stopp;
        VAR_omni = 0;
        while((((rechtsvorne + rechtshinten )/2)>=80)&&(!VAR_omni < 6)){
           bissl;
            __delay_ms(50);
            bissr;
            __delay_ms(55);
            bisslrueck;
            __delay_ms(45);
            bissrrueck;
            __delay_ms(45);
            VAR_omni++; 
        }
        VAR_omni = 0;
        stopp;
    }
}

void Letter_kal(void) {
    while(!Taster) { //Taster am Master. Wenn gedrückt geht die Arbeitsschleife los
        Status = 1; //LED dauernd an
        if (!Datum_R[0]){ //Warten auf Taster am 16F876A, ungedrückt HI
        
            while (!Datum_R[0]); //Warten auf Taster loslassen
            Datum1_an_Letter = 255; //Letter wird kalibriert
            Datum2_an_Letter = 255;
            //Zeit, dass sich der Letter-Baustein kalibrieren kann
            while (Datum_R[0]) { //Warten auf Taster
               Status = 0;; // LED dauernd aus
               //Roboter langsam am Strich vorbei fahren lassen
               vorwaerts;
               __delay_ms(3);
               stopp;
               __delay_ms(17);
               Werte_auf_LCD_anzeigen();
            }
            while (!Datum_R[0]); //Warten auf Taster loslassen
            Datum2_an_Letter = 254; //Warum auch immer: 0 als gesendeter Wert geht nicht
        }
        Werte_auf_LCD_anzeigen();
    }
    Datum1_an_Letter = rechtsvorne;
    Datum2_an_Letter = rechtshinten;
    Flag_kalibrieren = 1;
}
void Werte_auf_LCD_anzeigen (void) {
    for(char i = 0; i < 11; i++) { //die ersten 11 sind die AD-Werte 
        Datum_S[i] = A_D_Wert[i]; //HI-Teil des AD-Registers, s. Interrupt   
    }
    Datum_S[12] = Encoder; 
    Datum_S[13] = 13; //Datum_R[1] ; //Wert von Letter_1-Sensor TESTWERT GERADE DRIN!
    Datum_S[14] = 14; //Datum_R[2]; //Wert von Letter_2-Sensor TESTWERT GERADE DRIN!
    Datum_S[15] = Temperatur_HI_rechts; 
    Datum_S[16] = Temperatur_LO_rechts; 
    Datum_S[17] = Temperatur_HI_links;
    Datum_S[18] = Temperatur_LO_links;
    Datum_S[19] = 33; //auch
}
void opfer (void) {
    while(0){
        if(Flag_T_links == 1){
            if (linksvorne > wand){
                if (VAR_zeit_opfer > 300){
                    Flag_LED = 0; // wenn LED = 0 dann blinkt sie, wenn 1 dann aus.
                        stopp;
                        Flag_T_links = 0;
                        stopp;
                        rechts90grad();
                        while(!Flag_Strecke_erreicht);
                        stopp;
                        kit_abwerfen();
                        stopp;
                        links90grad();
                        while(!Flag_Strecke_erreicht);
                        stopp;
                        __delay_ms(5000);
                        Flag_LED = 0;
                        VAR_zeit_opfer = 0;
                        Strecke_fahren(15, vorwaerts);
                        while(!Flag_Strecke_erreicht);
                        stopp;
                        
                }
                else{
                    Flag_T_links = 0;
                }
            }
            else{
//                stopp;
//                __delay_ms(1000);
//                links90grad();
//                while(!Flag_Strecke_erreicht);
//                stopp;
//                hintenausrichtenopfer();
//                while(vornerechts < wand){
//                    vorwaerts;
//                }
//                stopp ;
//                links90grad();
//                while(!Flag_Strecke_erreicht);
//                stopp;
//                links90grad();
//                while(!Flag_Strecke_erreicht);
//                stopp;
//                kit_abwerfen();
//                stopp ;
//                Flag_LED =1;
//                __delay_ms(5000);
//                Flag_LED =0;
//                hintenausrichtenopfer();
//                while(vornerechts < wand){
//                    vorwaerts;
//                }
//                stopp ;
//                links90grad();
//                while(!Flag_Strecke_erreicht);
//                stopp;
                Flag_T_links = 0;
            }
        }
        if(Flag_T_rechts == 1){
            if(rechtsvorne > wand){
                if (VAR_zeit_opfer > 300){
                    Flag_LED =0; // wenn LED = 0 dann blinkt sie, wenn 1 dann aus.
                    stopp;
                    Flag_T_rechts = 0;
                    stopp;
                    links90grad();
                    while(!Flag_Strecke_erreicht);
                    stopp;
                    kit_abwerfen();
                    stopp;
                    rechts90grad();
                    while(!Flag_Strecke_erreicht);
                    stopp;
                    __delay_ms(5000);
                    Flag_LED = 0;
                    VAR_zeit_opfer = 0;
                }
                else{
                    Flag_T_rechts = 0;
                }
            }
            else{
                Flag_T_rechts = 0;
            }
        }
}
}
void kit_abwerfen(void) {
    GIE = 0; //Interrupt aus, damit delays vong servo nicht kaputt.
    for(char i=0; i<6; i++){
        servo = 1;
        __delay_us(2390);
        servo = 0;
        __delay_ms(20);
    }
    __delay_ms(200);
    for(char i=0; i<6; i++){
        servo = 1;
        __delay_us(2110);
        servo = 0;
        __delay_ms(20);
    }
    for(char i=0; i<6; i++){
        servo = 1;
        __delay_us(2390);
        servo = 0;
        __delay_ms(20);
    }
    GIE = 1; //Interrupt wieder an!!!!!!!!!!!!
}

void anderwandlang(void){ 
    // mit der Funktion soll er an der Wand lang fahren und richtet sich rechts aus.
    //wenn rechts keine Wand ist fährter bis zur mitte der nächsten Platte

         if (rechtsvorne >= zunah) { //Sensor rechts vorne zu nah an der Wand?
        bissl; //linke Motoren aus, rechte vorwärts
        __delay_ms(7); //Diesen Wert anpassen, muss zusammen mit nächstem delay 20ms ergeben
        vorwaerts; //Beide Motoren wieder an
        __delay_ms(13); //Wir produzieren uns ein einfaches PWM, delta T = 1ms
        opfer();
    }
    if (rechtsvorne <= zuweit) { //Sensor rechts vorne zu weit weg von der Wand?
        bissr; //rechte Motoren aus, linke vorwärts
        __delay_ms(7); //leichten Bogen nach rechts fahren
        vorwaerts; //
        __delay_ms(13); //
        opfer();
    }
    if (rechtsvorne < zunah) { //NICHT zu nah an Wand
        if (rechtsvorne > zuweit) { //NICHT zu weit von Wand, also genau in dem
            //Intervall in dem sich der Roboter bewegen soll
            if (rechtsvorne > (rechtshinten -2)) { //rechtsvorne näher, als hinten, Hysterese = 5, evtl. Offset
                bissl; //"PWM"
                __delay_ms(7); //Werte anpassen
                vorwaerts; //
                __delay_ms(13); //
                opfer();
            }
            else {
                if (rechtsvorne < (rechtshinten +2)) { //rechtshinten näher
                    bissr; //
                    __delay_ms(7); //
                    vorwaerts; //
                    __delay_ms(13); //
                    opfer();
                }
                else{
                vorwaerts; //alles im grünen Bereich
                __delay_ms(20);
                opfer();
                }
            }
        }
    
        
    
    }

    
    }
void schwarzeplatte(void){  //     >schwarz    <weiß
    if ((schwarzrechts > schwarz  )||(schwarzlinks > schwarz )){ //einer ODER der andere schwarz?
        stopp;
        Strecke_fahren (30,vorwaerts); //steht jetzt auf der schwarzen platte für kontrollabfrage.
        while(!Flag_Strecke_erreicht);
        stopp;
        if ((schwarzrechts > schwarz  )&&(schwarzlinks > schwarz )){
            stopp;
            VAR_Schwarz_ausrichten=1;
            Encoder=0;
            while ((schwarzrechts > schwarz  )&&(schwarzlinks > schwarz )){ // beide nicht weiß
                while ((schwarzrechts > schwarz  )&&(schwarzlinks > schwarz )){ //solange beide schwarz fahr nach hinten.
                    nachhinten;
                    __delay_ms(20);
                    stopp;
                    __delay_ms(12);
                }
                while ((schwarzrechts > schwarz  )&&(schwarzlinks < schwarz )){ //wenn links weiß und rechts schwarz 
                    bisslrueck;                                              //dann dreh dich so dass du grade wirst
                    __delay_ms(20);
                    bissr;
                    __delay_ms(12);
                    stopp;
                }
                while ((schwarzrechts < schwarz  )&&(schwarzlinks > schwarz )){ //wenn links schwarz und rchts weiß
                    bissrrueck;
                    __delay_ms(15);
                    bissl;
                    __delay_ms(5);
                    stopp;
                }
                
            }
            stopp; //jetzt steht er am Rand von der Platte
            __delay_ms(50);
            Strecke_fahren(48,nachhinten); //jetzt in der Mitte von der weißen vor der schwarzen
            while(!Flag_Strecke_erreicht);
            stopp;
            __delay_ms(1000);
            ausrichten();
            links90grad();
            while (!Flag_Strecke_erreicht){
                opfer();
            }
            stopp;
            ausrichten();
            if (vornerechts > wand){ //vorne ist eine Wand
                links90grad();
                while (!Flag_Strecke_erreicht){
                    opfer();
                }
                stopp;
                ausrichten();
            }
            else{                 // kaine wand, 
                stopp;
                __delay_ms(50);
            von_mitte_bis_wieder_wand_in_schwarzeplatte();
            stopp;
            __delay_ms(50);
            }  
            
        }
        else{
            Strecke_fahren (30,nachhinten);
            while(!Flag_Strecke_erreicht);
            stopp;
            //wenn nur einer in die richtige richtung fakeomnin
        }
    }
    
}
void hindernis(void){
    if (Flag_Hindernis == 1){
        Encoder = encodersave; //encoder merken.
        while((vornelinks > 110)&& (vornerechts < wand)){
            bissr;
            __delay_ms(52);
            bissl;
            __delay_ms(38);
            bissrrueck;
            __delay_ms(42);
            bisslrueck;
            __delay_ms(42);
        }
        VAR_omni=0;
        while (VAR_omni < 2){
            bissr;
            __delay_ms(52);
            bissl;
            __delay_ms(38);
            bissrrueck;
            __delay_ms(42);
            bisslrueck;
            __delay_ms(42);
            VAR_omni++;
        }
    }
    Flag_Hindernis = 0;
    VAR_omni = 0;
    encodersave = Encoder;
}

void im_Kreis_faren (void) {
    if (VAR_im_Kreis_faren > 6){
        VAR_links_ist_nichts ++ ;
        if (linksvorne > wand ){
            links90grad();
            while(!Flag_Strecke_erreicht){
            }
            stopp;
            links90grad();
            while(!Flag_Strecke_erreicht){
            }
            stopp;
            VAR_im_Kreis_faren = 0 ;
            VAR_links_ist_nichts = 0 ;
        }
        if (VAR_links_ist_nichts > 4){
            stopp;
            __delay_ms(1000);
            VAR_links_ist_nichts = 0 ;
            while (vornelinks < wand || rechtsvorne < wand ){
                vorwaerts ;
            }
            stopp;
            if (vornerechts > wand){
               links90grad();
               while (!Flag_Strecke_erreicht){}
               stopp ;   
            }
            VAR_im_Kreis_faren = 0 ;
        }
    }   
}
void Power_Ausrichten(void){
//            if (VAR_aus_zeit > Zeit_power_Ausrichten){
//                Ausrichten_Fahren=15;
//                Ausrichten_Stopp=5;
//            }
//            else{
//                Ausrichten_Fahren=7;
//                Ausrichten_Stopp=13;
//            }
}

void ausrichten (void){
    links_ausrichten();
    rechts_ausrichten();
    vorne_ausrichten(); //richte dich vorne aus, wenn möglich.
        //if((vornerechts > Schwellwert_Ausrichten)||(vornelinks > Schwellwert_Ausrichten)){//jetzt guck ob vor dir eine Wand ist (muss ja wenn du dich vorne ausgerictet hast)
        hinten_ausrichten(); // falls auf dem eine ODER anderen keine Wand war richte dich nicht aus. sonst ja. 
                             // (in hinten ausrichten guckt er auch noch mal ob da eine wand zum ausrichten ist!)
        //}
    
    links_ausrichten();
    omnifakelinks();
    stopp;
    __delay_ms(50);
    links_ausrichten();
        if((linksvorne < Schwellwert_Ausrichten)||(linkshinten < Schwellwert_Ausrichten)){ // siehe oberhalb ;)
        rechts_ausrichten();
        omnifakerechts();
        stopp;
        __delay_ms(50);
        rechts_ausrichten();
        }
}
void vorne_ausrichten (void) {
    if ((vornerechts < wand) && ( vornelinks < wand )){ //wenn da keine Wand ist
        if ((vornelang < 111) && (vornelang >68)){
            Var_lang_Ausrichten = 0;
            while (Var_lang_Ausrichten == 0){
                if (vornelang < 89){ //also weiter weg
                    vorwaerts;
                    __delay_ms(10);
                    stopp;
                    __delay_ms(10);
                }
                if (vornelang > 89){ //also näher dran
                    nachhinten;
                    __delay_ms(10);
                    stopp;
                    __delay_ms(10);
                }
                if ((vornelang > 88)&&(vornelang < 90)){
                    Var_lang_Ausrichten = 1;
                    stopp;
                    __delay_ms(1000);
                }
            }
        }
    }
    if ((vornerechts > Schwellwert_Ausrichten) && ( vornelinks > Schwellwert_Ausrichten )){
        stopp;
        VAR_aus_zeit = 0;
        stopp;
        while ((vornerechts > 81)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            nachhinten;
            __delay_ms (Ausrichten_Fahren); 
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((vornerechts < 77)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            vorwaerts;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while (vornerechts < (vornelinks +2 ) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissl;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while(vornerechts > (vornelinks + 2)&& (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissr;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((vornerechts < 77)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            vorwaerts;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((vornerechts > 81)&&(!VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            nachhinten;
            __delay_ms (Ausrichten_Fahren); 
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
    }        
}
void hinten_ausrichten (void) {
    if ((hintenrechts  >Schwellwert_Ausrichten)&&( hintenlinks > Schwellwert_Ausrichten )) {
        while ((hintenlinks < 81)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            nachhinten;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
            }
        VAR_aus_zeit = 0;
        while ((hintenlinks > 77)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            vorwaerts;
            __delay_ms (Ausrichten_Fahren); 
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;

        while (hintenrechts > (hintenlinks +2) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissl;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while(hintenrechts < (hintenlinks +2 )&& (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissr;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;

        while ((hintenlinks < 81)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            nachhinten;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
            }
        VAR_aus_zeit = 0;
        
        while ((hintenlinks > 77)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            vorwaerts;
            __delay_ms (Ausrichten_Fahren); 
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
    }
}
void links_ausrichten (void) {
    if (( linksvorne  > Schwellwert_Ausrichten )&&( linkshinten > Schwellwert_Ausrichten )){
        VAR_aus_zeit = 0;
        while ((linkshinten > linksvorne ) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissl;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((linkshinten < linksvorne ) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissr ;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
    }
}
void rechts_ausrichten (void) {
    if (( rechtsvorne  > 63 )&&( rechtshinten > 63 )) { //wenn rechts von dir eine Wand
        VAR_aus_zeit = 0;
        while ((rechtsvorne > rechtshinten +2 ) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissl;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((rechtsvorne < rechtshinten +2 ) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissr ;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            opfer();
            VAR_aus_zeit ++ ;
        }  
    }
}
void hintenausrichtenopfer (void){
    if ((hintenrechts  >Schwellwert_Ausrichten)&&( hintenlinks > Schwellwert_Ausrichten )){
        VAR_aus_zeit = 0;
        while (hintenrechts > (hintenlinks) && (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissl;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while(hintenrechts < (hintenlinks )&& (VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            bissr;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            VAR_aus_zeit ++ ;
        }
        VAR_aus_zeit = 0;
        while ((hintenlinks < 81)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            nachhinten;
            __delay_ms (Ausrichten_Fahren);
            stopp;
            __delay_ms (Ausrichten_Stopp);
            VAR_aus_zeit ++ ;
            }
        VAR_aus_zeit = 0;
        while ((hintenlinks > 79)&&(VAR_aus_zeit <= 3000)){
            Power_Ausrichten();
            vorwaerts;
            __delay_ms (Ausrichten_Fahren); 
            stopp;
            __delay_ms (Ausrichten_Stopp);
            VAR_aus_zeit ++ ;
        }
    }
}



       
void cm_fahren (void){
    
    Strecke_fahren(5,vorwaerts);
    while (!Flag_Strecke_erreicht){
        opfer();
        schwarzeplatte();
    }
    stopp;
}
void Strecke_fahren(char Length, char Direction) {
    IOCIE = 0; //Jetzt können wir keinen Interrupt on change gebrauche
    /*Encoder Integer?************************************************/
    Encoder = 0; //Encoderzähler löschen
    /*****************************************************************/
    Strecke = Length; //Strecke an "Interrupt" übergeben, 6 Ticks/cm (90 Ticks/Umdrehung)
    Flag_Strecke_erreicht = 0; //
    Flag_Strecke_messen = 1; //
    IOCIE = 1; //Interrupt wieder an
    Direction; //Bit 3 - 0 Motorsteuerung
}

//Interrupt
void interrupt Interrupt(void) { //sic
//    Status = 1; //Hilfreich zum debuggen
    //Der Quarz schwingt mit 20MHz, jeder Befehl braucht 4 Schwingungen
    //Alle 200ns wird ein Befehl ausgeführt (Zyklus) = 5 MHz
    //Prescaler 1:16 -> Interrupt alle 0,816ms = 816us = 4096 Zyklen = 4096 Befehle
    if (TMR0IF) {
        //Da mind. 816us vorbei sind hat mittlerweile die AD-Wandlung stattgefunden (s.u.)
        A_D_Wert [Channel_Index] = ADRESH; //A/D Resolution Hi-Byte, für die Auswertung im Programm
        //Analogwert initialisieren, alle 812us
        Channel_Index++; //AD-Kanal, näxter Kanal
        //näxte Zeile: n anpassen an die Anzahl der AD-Kanäle
        if (Channel_Index == 11) Channel_Index = 0; //
        ADCON0 = 0b10000001 | (Channel[Channel_Index] << 2); // Bit 6-2 wählen den Channel aus
/*******Jetzt müssen mind. 5us = 25 Zyklen vergehen, bevor die Wandlung gestartet werden darf
 *******D.H, wenn ab hier was gestrichen wird muss man das testen!!!!!!!!!!!!!!!!!!!!!!!!!!*/
//      
        Tick++; //Blinken der Status-LED, PIC arbeitet "korrekt"
        if(Flag_LED == 1) { //Soll die LED blinken?
            if (Tick >= 300) { //300 * 816us ~ 245ms
                Tick = 0; //Tick wieder auf 0 setzen
                Status = !Status; //0 ein, 1 aus
            }
        }
        else { //oder soll sie dauernd an sein?
            Status = 1; //LED dauernd aus
        }
/*Zeitstruktur für den I2C Bus:
 * zwei Daten schreiben dauert ca. 300us, ein Datum lesen ca. 250us
 * Alle 100us wird m inkrementiert
 * Alle 10ms wird n inkrementiert
 * Alle 10ms wird ein T-Wert geholt, im Wechsel links/rechts
 * Alle 20ms wird ans LCD ein Wert übertragen, 3,3ms versetzt zu T-Werte holen
 * Alle 20ms wird ein Buchstabenwert vom Sensor geholt, 6,6ms versetzt zu T-Werte holen
 * _| |___________| |___________| |__________| |__________| |___ T-Werte holen
 * _______|||_________________________|||_______________________ LCD-Werte senden
 * ________________________| |________________________| |_______ Buchstaben holen
 * Da wäre noch Platz für einige andere I2C Module
 */     
        
        m++; //Alle 100us wird m inkrementiert
        if (m == 12) { //Temperatur von MLX holen, ca. alle 10ms
        //Das auslesen des T-Sensors darf nicht unterbrochen werden, daher am Stück
            m = 0; //Wieder von vorne zählen
            n_US++; //Alle 40ms US triggern
            if (n_US == 4) { //40ms um?
                n_US = 0; //Zähler wieder auf Null setzen
                US_Trigger = 1; //An Pin28 5V anlegen
                __delay_us(10); //10us warten
                US_Trigger = 0; //Trigger-Pin wieder löschen
                TMR1 = 0; //Zeitmessung beginnen
                TMR1IF = 0; //Timer-1-Interrupt-Flag löschen, max. ca. 2m messbar
            }
            n++; //für Häufigkeit der Übertragung der Werte ans LCD
            if (n == 2) n = 0;
/**********************************************************************/ 
                MLX_read (); //Dauert ca. 500us, darf nicht unterbrochen werden
                if (Temperatur_HI_links >= Waerme_links) Flag_T_links = 1;    //wenn flag 1, dann opfer da.
                if (Temperatur_HI_rechts >= Waerme_rechts) Flag_T_rechts = 1;  //wenn flag 1, dann opfer da. 
                if (Temperatur_HI_links < Waerme_links) Flag_T_links = 0;
                if (Temperatur_HI_rechts < Waerme_rechts) Flag_T_rechts = 0;
/**********************************************************************/ 
        }
        if (m == 4) { //Zeitlich versetzt zu MLX_read() ausführen
            if (n == 0) { //Werte an Slave (LCD) übertragen, alle 20ms im Wechsel mit T holen
                SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz (PICTime)
                I2C_Count = 0; //Schrittzähler zurücksetzen
                I2C_Write = 1; //Jetzt wird geschrieben
                I2C_Read = 0; //Es wird nicht gelesen
                I2C_Add = PIC_Slave; //I2C Slave-Adresse vorbereiten
                I2C_Index_w = Count++; //Register/Daten die übertragen werden
                Register_S = I2C_Index_w;
                if (Count == 20) Count = 0; //Wieder von vorne
/***********************************************************************/                
                SEN = 1; //Übertragung starten, löst SSP-Interrupt aus
/***********************************************************************/
            }
        }        
        if (m == 4) { //Zeitlich versetzt zu MLX_read()und LCD ausführen
            if (n == 1) { //Werte vom Letter-Slave 0x15 holen, alle 20ms im Wechsel mit T und LCD holen
                SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz (PICTime)
                I2C_Count = 0; //Schrittzähler zurücksetzen
                I2C_Write = 0; //Es wird nicht geschrieben
                I2C_Read = 1; //Jetzt wird gelesen
                I2C_Add = PIC_Slave; //I2C Slave-Adresse vorbereiten
                I2C_Index_R = 0; //Index von Datum_R, hier 0 für 16F876A
/************************************************************************/
                SEN = 1; //Übertragung starten, löst SSP-Interrupt aus
/************************************************************************/
            }
        }        
if (m == 8) { //Zeitlich versetzt zu MLX_read()und LCD ausführen
        if (n == 0) { //Werte vom Letter-Slave 0x15 holen
            SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz (PICTime)
            I2C_Count = 0; //Schrittzähler zurücksetzen
            I2C_Write = 1; //Jetzt wird geschrieben
            I2C_Read = 0; //Es wird nicht gelesen
            I2C_Add = PIC_Letter_1; //I2C Slave-Adresse vorbereiten
            //Hier muss noch hin, wenn 2 Letter angeschlossen sind PIC_Letter_1 oder 2
            Register_S = Datum1_an_Letter; //1. Datum das an Letter übertragen wird
            I2C_Index_w = 15; //Datum_S[15] = 2. Datum an Letter
/***********************************************************************/                
                SEN = 1; //Übertragung starten, löst SSP-Interrupt aus
/***********************************************************************/
            }
        }
        
//Read_from_Letter;
        if (m == 8) { //Zeitlich versetzt zu MLX_read()und LCD ausführen
            if (n == 1) { //Werte vom Letter-Slave 0x15 holen
                SSPADD = 4; //1MHz I²C-Frequenz, 20MHz/(4*(4+1))=1MHz (PICTime)
                I2C_Count = 0; //Schrittzähler zurücksetzen
                I2C_Write = 0; //Jetzt wird geschrieben
                I2C_Read = 1; //Es wird nicht gelesen
                I2C_Add = PIC_Letter_1; //I2C Slave-Adresse vorbereiten
                I2C_Index_R = 1; //Daten die vom Letter x geholt werden sollen
                //Hier muss noch hin, wenn 2 Letter angeschlossen sind Index 1 oder 2
/***********************************************************************/                
                SEN = 1; //Übertragung starten, löst SSP-Interrupt aus
/***********************************************************************/
            }
        }
        
        TMR0IF = 0; //Schluss mit TMR0-Interrupt
//AD-Wandlung starten
        //Jetzt sollten die 5us vorbei sein
        //Analogwert initialisieren, alle 0,102ms
        GO_nDONE = 1; //Wandlung starten (Bit 2 von ADCON0, S. 127)
        //Die Wandlung braucht 40us, es dauert aber 100us bis der Wert beim 
        //nächsten TMR0-Interrupt ausgelesen wird.
//        Status = 0; //Zum Debuggen
    }
    //Interrupt durch I2C
    if (SSPIF) { //Hat den Interrupt das SSP-Modul ausgelöst?
        SSPIF = 0; //Flag löschen, gell?
        if (I2C_Write) { //Soll geschrieben werden?
            switch (I2C_Count) { //Je nachdem welchen Wert I2C_Count hat
                //I2C_Count = 0, erster Interrupt nach SEN
                case 0:
                    SSPBUF = (I2C_Add << 1) & 0b11111110; //Slaveadresse = 20[0010100X],
                    //nach CLK<9> SSPIF = 1;            LSB=0-> Master schreibt
                    I2C_Count++; //Beim nächsten Interrupt case 1 bearbeiten
                    break; //Die Funktion switch() verlassen
                //I2C_Count = 1;
                case 1:
                    SSPBUF = Register_S; //Erstes Datum schreiben
                    I2C_Count++; //Beim nächsten Interrupt case 2 bearbeiten
                    break; //Die Funktion switch() verlassen
                //I2C_Count = 2;
                case 2:
                    SSPBUF = Datum_S[I2C_Index_w]; //Zweites Datum schreiben
                    I2C_Count++; //Beim nächsten Interrupt case 3 bearbeiten
                    break; //Die Funktion switch() verlassen
                //I2C_Count = 3;
                case 3:
                    PEN = 1; //Übertragung beenden
                    I2C_Count++; //Brauchts eigentlich nicht, aber wer weiß...
                    I2C_Write = 0; //Flag löschen
                    break;
                //I2C_Count > 3;
                default: //Bis hierher kommt das Programm normal nicht
                    break; //Die Funktion switch() verlassen
            }
        }
        //Es wird vom PIC 16F876A nur ein Datum gelesen
        if (I2C_Read) { //Lesen ist dran
            switch (I2C_Count) { //Je nachdem welchen Wert I2C_Count hat
                //I2C_Count = 0, erster Interrupt nach SEN
                case 0:
                    SSPBUF = (I2C_Add << 1) | 0b00000001; //LSB=1-> Master liest
                    I2C_Count++; //Beim nächsten Interrupt case 1 bearbeiten
                    break; //Die Funktion switch() verlassen
                //I2C_Count = 1;
                case 1:
                    RCEN = 1; //receive enable
                    I2C_Count++; //Beim nächsten Interrupt case 2 bearbeiten
                    break; //Die Funktion switch() verlassen
                //I2C_Count = 2;
                case 2:
                    Datum_R[I2C_Index_R] = SSPBUF; //Hier ist RCEN wieder 0
                    ACKDT = 1; //Not Acknowledge, beendet die read-Sequenz
                    ACKEN = 1; //Wert von ACKDT wird gesendet, 9. Flanke
                    I2C_Count++; //Beim nächsten Interrupt case 3 bearbeiten
                    break; //Die Funktion switch() verlassen
                    //I2C_Count = 3;
                case 3:
                    PEN = 1; //Master ist fertig
                    I2C_Count++; //Beim nächsten Interrupt default bearbeiten
                    I2C_Read = 0; //Flag löschen
                    break; //Die Funktion switch() verlassen
                    //I2C_Count >3;
                default: //Bis hierher kommt das Programm normal nicht
                    break; //Die Funktion switch() verlassen
            }
        }
//        //Nach PEN springt beim erneuten SSP-Interrupt das Programmsofort hier
//        //her, weil weder I2C_Write noch I2C_Read HI ist
    }
       //PORTB Interrupt, Interrupt on Change
    //Init() anpassen: PORTB
    if (IOCIF) {
        //IOCIF wird gelöscht, wenn alle IOCBF gelöscht sind!!!
        //IOCIF = 0; geht nicht!!!
        //Encoderpin PullUp on!!!
//        if (IOCBF0) { //Hat den Interrupt PORTB,0 ausgelöst?
        if (IOCBF0) { //Hat den Interrupt PORTB,0 ausgelöst?
            Encoder++; //Encoder inkrementieren
            VAR_zeit_opfer++; // zeit hochzählen
            
            /*+++Achtung: Hier ganz speziell Ausgabe auf LCD Tab 18+++*/
//            Datum_S [3] = Encoder; //
//            IOCBF0 = 0; //Flag löschen
            IOCBF0 = 0; //Flag löschen
        }
        //
        if (Flag_Strecke_messen) { //Wird eine Strecke gefahren?
            if (Encoder >= Strecke) { //Ende der Strecke erreicht?
                Flag_Strecke_erreicht = 1; //sic
//                Flag_Strecke_messen = 0; //Fertig mit Strecke messen
            }
        }
        
        //Evtl. Variable für Hindernis/dumm an der Wand stehen setzen folgt.
        if ((vornelinks > 110)&& (vornerechts < wand)){
            Flag_Hindernis=1;
        }

        
//        if (IOCBF5) { //Hat den Interrupt PORTB,5 ausgelöst?
//            IOCBF5 = 0; //Flag löschen
//            if (!TMR1IF) { //TMR1 ist nicht übergelaufen? Strecke < 2m (13107us Laufzeit)
//                Zeit_US = TMR1H; //Timer 1 HI-Teil
//                Abstand_US = Zeit_US - 9; //Abstand_US ~ tats. Abstand(ca. 10% Abweichung)
//            }
//            else {
//                Abstand_US = 255; //Gemessene Strecke > 2m
//                TMR1IF = 0;
//            }
//            Datum_S[5] = Abstand_US; //Anzeige auf LCD
//        }
    }
//    Status = 0; //Hilfreich fürs debuggen
}