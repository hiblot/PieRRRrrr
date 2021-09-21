//C:\Users\daisywheel\Documents\Arduino\00.UNO\MOTEUR_CROUZET_24V_PROTOV3_VERSION3_ROUE_ALGO_PLENOIR
//20190924 : V1.1
// Temps de 44 / 3 / 41 / 3 

#include <Wire.h>
#include <motorshield.h>
#include <LiquidCrystal.h>

motorshield MotorShield;
LiquidCrystal lcd(8,9,4,5,6,7);

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int lcd_key     = 0;
int adc_key_in  = 0;
int nbcycles    = 0;
int state = 0;

int isOK = 0;
int nbCycles = 0;
int nbcompt1 = 0;
int nbcompt2 = 0;
int delay1 = 44000;
int delay2 = 3000;
int delay3 = 41000;
int valuenbcompt1 = 10;

// read the buttons
int read_LCD_buttons()
{
 adc_key_in = analogRead(0);      // read the value from the sensor 
// Serial.println("adc_key_in :" + adc_key_in);
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;  
 return btnNONE;  // when all others fail, return this...
}

void setup() 
{
  Serial.begin(9600);
  MotorShield.initialize();
  Serial.println("Dual DC Motor Shield test sketch");
 // lcd(8,9,4,5,6,7);
  lcd.begin(16, 2);              // start the libraryh
  lcd.print(" NB. de Cycles:");
  
      digitalWrite(In1, 0);
      analogWrite(EnA,255);
      digitalWrite(In3, 1);
      analogWrite(EnB,15);
  
  isOK = 1;
  nbCycles = 0;

}

void loop()
{
 lcd_key = read_LCD_buttons();  // read the buttons 
 switch (lcd_key)               // depending on which button was pushed, we perform an action
 {
   lcd.display();
   case btnRIGHT:
   {
     state = 1;
     break;
    }
 } 
 if (state == 1)
 {
    Proto();
 }
}

void Proto()
{
 if (isOK == 1)
 {
    lcd.begin(16, 2);              // start the libraryh
    lcd.print(" NB. de Cycles:");
    lcd.setCursor(9,1);
    lcd.print(nbCycles);
     
    digitalWrite(In1, 0);
    analogWrite(EnA,255);
    digitalWrite(In3, 1);
    analogWrite(EnB,255);
    delay(delay1);
    isOK = 2;
  }
  else if (isOK == 2)
  {
    digitalWrite(In1, 0);
    analogWrite(EnA,255);
    digitalWrite(In3, 1);
    analogWrite(EnB,0);
    delay(delay2);
    isOK = 3;
  }
  else if (isOK == 3)
  {
    digitalWrite(In1, 1);
    analogWrite(EnA,255);
    digitalWrite(In3, 1);
    analogWrite(EnB,255);
    delay(delay3);
    isOK = 4;
  }
  else if (isOK == 4)
  {
    digitalWrite(In1, 1);
    analogWrite(EnA,255);
    digitalWrite(In3, 1);
    analogWrite(EnB,0);
    delay(delay2);
    isOK = 5;
  }
  else if (isOK == 5)
  {
    nbCycles++;
    isOK = 1;
  }
  delay(250);
}
