/*******************************************************************************
 * Projet : DW DECO                                                            *
 * Titre : SLAVDUINO - ROUE                                                    *
 * Auteur : Mottin Marc                                                        *
 * Entreprise : Fillon Technologies                                            *
 * Plateforme : Arduino Uno                                                    *
 *******************************************************************************/


 
/*******************************************************************************
                                     INCLUDE                    
 *******************************************************************************/

// I2C
#include <Wire.h>

// Shield moteur
#include <microbot_motor_shield.h>
microbotMotorShield MotorShield;              // Définition de la librairie du shield moteur



/*******************************************************************************
                                     DEFINE                    
 *******************************************************************************/

// PINOUT
#define BUTTON_LEFT         0         // Pin INPUT - Bouton rotation manuelle - Gauche
#define BUTTON_RIGHT        1         // Pin INPUT - Bouton rotation manuelle - Droite
#define EVENT_PULSE         2         // Pin INPUT - Interruption - Retour de PULSE du moteur
//#define In1_1             3          // Pin OUTPUT - Bloqué par le shield moteur
#define VITESSE             5         // Pin OUTPUT - Moteur - Vitesse de rotation - 
//#define In1_2             5         // Bloqué par le shield moteur
//#define In2_1             6         // Pin OUTPUT - Bloqué par le shield moteur
//#define ONOFF               6         // Pin OUTPUT - Moteur - ON / OFF
#define ONOFF               7         // Pin OUTPUT - Moteur - ON / OFF
#define SENS                8         // Pin OUTPUT - moteur - Sens de rotation
//#define In2_2             9         // Pin OUTPUT - Bloqué par le shield moteur
#define TOP0                11        // Pin INPUT - Lecture de l'état du capteur TOP_0


// REGISTRE WRITE
#define REGW_ACTION          0
#define REGW_POSITION       1
#define REGW_SPEED_STD      2
#define REGW_SPEED_INIT     3
#define REGW_SPEED_SM       4
#define REGW_START_SM       5
#define REGW_TEMPO_SM       6
#define REGW_PULSE_SM       7
#define REGW_SWITCH_SM      8
#define REGW_MOVE           9


// REGISTRE READ
#define REGR_STATUT         0
#define REGR_PULSE          1
//#define REGR_PULSE_2      2
#define REGR_ROTATION       3
#define REGR_OBLONG         4
#define REGR_WAITSM         5
#define REGR_DOOR_CLOSED    6
#define REGR_COMM           7


// COMMANDE ACTION
#define STOP                0
#define INIT                1
#define POSITION            2
#define SM                  3
#define LEFT                4
#define RIGHT               5


// INFOS STATUT
#define WAIT                1
#define INIT_RUN            2
#define INIT_OK             3
#define POS_RUN             4
#define POS_OK              5
#define SM_RUN              6
#define ERR_COMM            8


// INFOS POSITION
#define PULSE_MAX_ROUE      15360

// INFOS VITESSE
#define VITESSE_MIN         8
#define VITESSE_MAX         240
#define VITESSE_STD         120


// INFOS SENS
#define SENS_HORAIRE        0
#define SENS_TRIGO          1


// INFOS SM
#define CONV_TIMER          1000        // 1 seconde
#define START_SM            900       // ASK SM - 900s = 15mn
#define TEMPO_SM            1           // DELAY SM - 1 seconde (mvmt continu)
#define PULSE_SM            5120        // distance parcouru à chaque DELAY du SM - 5120 pas (1/3 de roue)
#define SWITCH_SM           7200        // SWITCH SENS SM - 7200s = 120mn = 2h


/*******************************************************************************
                               VARIABLES GLOBALES
 *******************************************************************************/

// I2C
unsigned int  uiDataReceived[4] = {0x00, 0x00, 0x00, 0x00};           // Data received I2C
uint8_t       ui8NewData        = 0;                                  // Token new data I2C
int           iNbrByteRead      = 4;                                  // Verif perte I2C


// Registre Interne
unsigned int  uiRegInterne_W[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};      // Commande
unsigned int  uiRegInterne_R[8] = {0, 0, 255, 0, 0, 0, 0, 117};         // Info actuelle


// COMMANDE
uint8_t       ui8Action         = 0;                                  // Choix du fonctionnement moteur


// STATUT
uint8_t       ui8STAT_NOW       = 0;
uint8_t       ui8WAIT           = 0;


// SPEED
uint8_t ui8Speed_RAMP       = VITESSE_MIN;
uint8_t ui8SpeedMax_STD     = VITESSE_STD;
uint8_t ui8SpeedMax_INIT    = VITESSE_STD;
uint8_t ui8SpeedMax_SM      = VITESSE_MIN;


// SENS
uint8_t ui8Sens = SENS_HORAIRE;
uint8_t ui8ChoixSens = 0;
uint8_t ui8SwitchSens_SM = 0;


// POSITION
int iPULSE = 0;                         // Position (pulse) du moteur
unsigned int uiConsignePosition = 0;
int iDiffPosition         = 0;

// SM
uint8_t ui8TempoSM        = 0;
uint8_t ui8CptSM        = 0;


// TOP 0
uint8_t     ui8TrouOblong     = 0; 


// TIMER
unsigned long   ulTIMERSM     = 0;
unsigned long   ulTEMPOSM     = 0;
unsigned long   ulTIMERCOMM   = 0;

// COMPARAISON TIMER
unsigned long ulCompStartSM   = START_SM * CONV_TIMER;
unsigned long ulCompSwitchSM  = SWITCH_SM * CONV_TIMER;
unsigned long ulCompTempoSM   = TEMPO_SM * CONV_TIMER;
unsigned long ulCompComm      = 60000 * CONV_TIMER;   // 16h?



/*******************************************************************************
                                      SETUP
 *******************************************************************************/

void setup()
{  
//-----------------------------------------SETUP DRIVERS-----------------------------------------//

  Wire.begin(1);                              // Init de la liaison I2C, @ = 0x01
  Wire.onReceive(receiveEvent);               // Event en cas de réception de données
  Wire.onRequest(requestEvent);               // Event en cas de demande de données
  
  MotorShield.begin();                        // Initialisation du shield moteur (hard)

  //Serial.begin(9600);
  
//-------------------------------------------SETUP I/O-------------------------------------------//

  pinMode(ONOFF,        OUTPUT);
  pinMode(SENS,         OUTPUT);
  pinMode(EVENT_PULSE,  INPUT_PULLUP);
  pinMode(BUTTON_LEFT,  INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(TOP0,         INPUT_PULLUP);

  digitalWrite(ONOFF, LOW);
  digitalWrite(SENS,  LOW);
  digitalWrite(In1_1, LOW);

  
//----------------------------------------SETUP INTERRUPT----------------------------------------//

  attachInterrupt(digitalPinToInterrupt(EVENT_PULSE), FCTN_Interrupt, RISING);

//-------------------------------------------INIT ROUE-------------------------------------------//

  uiRegInterne_W[REGW_ACTION]     = STOP;
  uiRegInterne_W[REGW_POSITION]   = 0;
  uiRegInterne_W[REGW_SPEED_STD]  = VITESSE_STD;
  uiRegInterne_W[REGW_SPEED_INIT] = VITESSE_STD;
  uiRegInterne_W[REGW_SPEED_SM]   = VITESSE_MIN;
  uiRegInterne_W[REGW_START_SM]   = START_SM;
  uiRegInterne_W[REGW_TEMPO_SM]   = TEMPO_SM;
  uiRegInterne_W[REGW_PULSE_SM]   = PULSE_SM;
  uiRegInterne_W[REGW_SWITCH_SM]  = SWITCH_SM;
  uiRegInterne_W[REGW_MOVE]       = 0;

  ulTIMERSM   = millis();
  ulTEMPOSM   = millis();
  ulTIMERCOMM = millis();

  
}



/*******************************************************************************
                                      LOOP
 *******************************************************************************/
 
void loop()
{  
  
  if(ui8NewData == 1)
  {
    ui8NewData = 0;
    
    Gestion_Trame();
  }

  if(ui8NewData != 2)
  {
    Gestion_Commande();  
  }
  
  Gestion_Delay();


  delay(50);
}



/*******************************************************************************
                                      EVENT                  
 *******************************************************************************/

void FCTN_Interrupt()
{
  if(ui8Sens == SENS_HORAIRE)
  {
    if(iPULSE > 15359)
    {
      iPULSE = 0;
    }
    else
    {
      iPULSE++;
    }
  }
  else
  {
    if(iPULSE < 0)
    {
      iPULSE = 15359;
    }
    else
    {
      iPULSE--;
    }
  }
}


void receiveEvent(int howMany)
{    
  int iBoucle = 0;

  if(howMany == iNbrByteRead)
  {
    while (Wire.available())
    {
      uiDataReceived[iBoucle] = Wire.read();
     
      iBoucle++;
    }
     
    ui8NewData = 1;  
  
    ulTIMERCOMM = millis();
  }
  else
  {
    ui8NewData = 2;                             // erreur
    uiRegInterne_R[REGR_STATUT] = ERR_COMM;     // Erreur Comm
    Wire.flush();
  }
}


void requestEvent()
{
  Wire.write(uiRegInterne_R[REGR_STATUT]);
  Wire.write((uiRegInterne_R[REGR_PULSE] >> 8));
  Wire.write((uiRegInterne_R[REGR_PULSE] & 0x00FF));
  Wire.write(uiRegInterne_R[REGR_ROTATION]);  
  Wire.write(uiRegInterne_R[REGR_OBLONG]); 
  Wire.write(uiRegInterne_R[REGR_WAITSM]); 
  Wire.write(uiRegInterne_R[REGR_DOOR_CLOSED]); 
  Wire.write(uiRegInterne_R[REGR_COMM]); 

  ulTIMERCOMM = millis();
}



/*******************************************************************************
                                    FONCTION                  
 *******************************************************************************/

void Gestion_Trame()
{
  if(uiDataReceived[0] == 0x10)                    // WRITE
  {  
    switch(uiDataReceived[1])
    {
      case REGW_ACTION:
        ui8WAIT = 1;
        uiRegInterne_R[REGR_WAITSM] = 0;
        
        uiRegInterne_W[REGW_ACTION] = (uiDataReceived[2] << 8) + uiDataReceived[3];

        if(uiRegInterne_W[REGW_ACTION] == POSITION)
        {
          ui8ChoixSens = 1;
          // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);
          uiConsignePosition   = uiRegInterne_W[REGW_POSITION];
          ui8SpeedMax_STD   = uiRegInterne_W[REGW_SPEED_STD];
        }
        else if(uiRegInterne_W[REGW_ACTION] == STOP)
        {
          uiRegInterne_R[REGR_STATUT] = WAIT;
        }
        else if(uiRegInterne_W[REGW_ACTION] == SM)
        {
          // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);
          ui8SpeedMax_SM = uiRegInterne_W[REGW_SPEED_SM];
        }

        break;

      case REGW_POSITION:
        uiRegInterne_W[REGW_POSITION] = (uiDataReceived[2] << 8) + uiDataReceived[3];
        break;

      case REGW_SPEED_STD:
        uiRegInterne_W[REGW_SPEED_STD]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
        
        if(uiRegInterne_W[REGW_SPEED_STD] >= VITESSE_MAX)
        {
          uiRegInterne_W[REGW_SPEED_STD] = VITESSE_MAX;
        }
        else if(uiRegInterne_W[REGW_SPEED_STD] <= VITESSE_MIN)
        {
          uiRegInterne_W[REGW_SPEED_STD] = VITESSE_MIN;
        }
        
        break;
    
      case REGW_SPEED_INIT:
        uiRegInterne_W[REGW_SPEED_INIT]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
            
        if(uiRegInterne_W[REGW_SPEED_INIT] >= VITESSE_STD)
        {
          uiRegInterne_W[REGW_SPEED_INIT] = VITESSE_STD;
        }
        else if(uiRegInterne_W[REGW_SPEED_INIT] <= VITESSE_MIN)
        {
          uiRegInterne_W[REGW_SPEED_INIT] = VITESSE_MIN;
        }
        
        break;
    
      case REGW_SPEED_SM:
        uiRegInterne_W[REGW_SPEED_SM]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
            
        if(uiRegInterne_W[REGW_SPEED_SM] >= VITESSE_STD)
        {
          uiRegInterne_W[REGW_SPEED_SM] = VITESSE_STD;
        }
        else if(uiRegInterne_W[REGW_SPEED_SM] <= VITESSE_MIN)
        {
          uiRegInterne_W[REGW_SPEED_SM] = VITESSE_MIN;
        }
        
        break;
  
      case REGW_START_SM:
        uiRegInterne_W[REGW_START_SM]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
        ulCompStartSM = (unsigned long)uiRegInterne_W[REGW_START_SM] * CONV_TIMER;
        break;
        
      case REGW_TEMPO_SM:
        uiRegInterne_W[REGW_TEMPO_SM]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
        ulCompTempoSM = uiRegInterne_W[REGW_TEMPO_SM] * CONV_TIMER;
        break;
    
      case REGW_PULSE_SM:
        uiRegInterne_W[REGW_PULSE_SM]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
        break;
        
      case REGW_SWITCH_SM:
        uiRegInterne_W[REGW_SWITCH_SM]  = (uiDataReceived[2] << 8) + uiDataReceived[3];
        ulCompSwitchSM = (unsigned long)uiRegInterne_W[REGW_SWITCH_SM] * CONV_TIMER;
        break;

      case REGW_MOVE:
        uiRegInterne_W[REGW_MOVE] = (uiDataReceived[2] << 8) + uiDataReceived[3];

        if(uiRegInterne_W[REGW_MOVE] == 1)
        {
          digitalWrite(ONOFF, HIGH);
        }
        else
        {
          digitalWrite(ONOFF, LOW);
        }
        break;
    }
    ulTIMERSM = millis();    
  }
  //ulTIMERCOMM = millis();
}


// ************************************************************************************************** //

void Gestion_Commande()
{
  ui8Action = uiRegInterne_W[REGW_ACTION];
  
  if(digitalRead(TOP0) == 1)              // ROUE 
  {
    uiRegInterne_R[REGR_OBLONG] = 0;
  }
  else
  {
    uiRegInterne_R[REGR_OBLONG] = 1;
  }
  
  
  
  if(ui8Action == STOP)                                     // ------------- STOP ------------- //
  {   
    
    ui8Speed_RAMP = VITESSE_MIN;
    
    // COMMENT TEST sécu - digitalWrite(ONOFF, LOW);
    digitalWrite(ONOFF, LOW);
    digitalWrite(VITESSE, LOW);
    uiRegInterne_R[REGR_ROTATION] = 0;      // ROTATION ARRETEE

    uiRegInterne_R[REGR_STATUT] = WAIT;
    
    ui8WAIT = 0;
  }
  else if(ui8Action == INIT)                                  // ---------------- INIT ---------------- //
  {
    digitalWrite(ONOFF, HIGH);
    uiRegInterne_R[REGR_STATUT] = INIT_RUN;
    uiRegInterne_R[REGR_ROTATION] = 1;      // ROTATION EN COURS
    
    ui8SpeedMax_INIT = uiRegInterne_W[REGW_SPEED_INIT];
    
    ui8TrouOblong = 0;
  
  
    // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);    
    ui8Sens = SENS_HORAIRE;                         
    digitalWrite(SENS, ui8Sens);
  
  
    while(1)
    {
      if(digitalRead(TOP0) == 1)              // ROUE 
      {
        if(ui8TrouOblong == 1)
        {
          digitalWrite(VITESSE, LOW);
          delay(50);
          uiRegInterne_R[REGR_STATUT] = INIT_OK;
          //ui8STAT_NOW = INIT_OK;
          uiRegInterne_R[REGR_ROTATION] = 0;  // ROTATION ARRETEE
          ulTIMERSM = millis();
          
          break;
        }
        else
        {
          if(ui8Speed_RAMP >= ui8SpeedMax_INIT)
          {
            ui8Speed_RAMP = ui8SpeedMax_INIT;
          }
          else
          {
            ui8Speed_RAMP += 10;
          }
      
          analogWrite(VITESSE, ui8Speed_RAMP);
        }
      }
      
      if(digitalRead(TOP0) == 0)              // TROU
      {
        analogWrite(VITESSE, VITESSE_MIN);
        ui8TrouOblong = 1;
        delay(50);
      }
    }
    
    iPULSE = 0;
    uiRegInterne_R[REGR_PULSE] = iPULSE;
    
    uiRegInterne_W[REGW_ACTION] = STOP;
    
    //ui8STAT_NOW = WAIT;
  }
  else if(ui8Action == POSITION)                                                // ------------- POSITION ------------- //
  {
    digitalWrite(ONOFF, HIGH);
    uiRegInterne_R[REGR_STATUT] = POS_RUN;
    uiRegInterne_R[REGR_ROTATION] = 1;    // ROTATION EN COURS

    Commande_Moteur(uiConsignePosition, ui8SpeedMax_STD);
  }
  else if(ui8Action == SM)                                                      // ---------------- SM ---------------- //
  {
    digitalWrite(ONOFF, HIGH);
    uiRegInterne_R[REGR_STATUT] = SM_RUN;
    uiRegInterne_R[REGR_ROTATION] = 1;     // ROTATION EN COURS
    

    
    if(digitalRead(BUTTON_LEFT) == 0)
    {
      while(digitalRead(BUTTON_LEFT) == 0)
      {
        ui8Sens = SENS_HORAIRE;                         
        digitalWrite(SENS, ui8Sens); 
            
        if(ui8Speed_RAMP >= ui8SpeedMax_SM)
        {
          ui8Speed_RAMP = ui8SpeedMax_SM;
        }
        else
        {
          ui8Speed_RAMP += 10;
          delay(100);
        }
  
        analogWrite(VITESSE, ui8Speed_RAMP);
  
        ulTEMPOSM = millis();
        ui8TempoSM = 0;
      }
      digitalWrite(VITESSE, LOW);  
    }
    else if(digitalRead(BUTTON_RIGHT) == 0)
    {
      while(digitalRead(BUTTON_RIGHT) == 0)
      {
        ui8Sens = SENS_TRIGO;
        digitalWrite(SENS, ui8Sens);                           
  
        if(ui8Speed_RAMP >= ui8SpeedMax_SM)
        {
          ui8Speed_RAMP = ui8SpeedMax_SM;
        }
        else
        {
          ui8Speed_RAMP += 10;
          delay(100);
        }
  
        analogWrite(VITESSE, ui8Speed_RAMP);
  
        ulTEMPOSM = millis();
        ui8TempoSM = 0;
      }
      digitalWrite(VITESSE, LOW);  
    }
        
    if(ui8TempoSM == 1)
    {
      //ui8TempoSM = 2;

      analogWrite(VITESSE, 10);

      /*
      if(ui8SwitchSens_SM == SENS_TRIGO)
      {
        if(iPULSE <= uiRegInterne_W[REGW_PULSE_SM])
        {
          uiConsignePosition = 15360 + iPULSE - uiRegInterne_W[REGW_PULSE_SM];
        }
        else
        {
          uiConsignePosition = iPULSE - uiRegInterne_W[REGW_PULSE_SM];
        }
      }
      else                                                                // SENS HORAIRE
      {
        if(iPULSE >= (15360 - uiRegInterne_W[REGW_PULSE_SM]))
        {
          uiConsignePosition = uiRegInterne_W[REGW_PULSE_SM] - 15360 + iPULSE;
        }
        else
        {
          uiConsignePosition = iPULSE + uiRegInterne_W[REGW_PULSE_SM];
        }
      }*/

      
      ulTEMPOSM = millis();
    }
    else if(ui8TempoSM == 2)
    {
      Commande_Moteur(uiConsignePosition, ui8SpeedMax_SM);
    }

    uiRegInterne_R[REGR_PULSE] = iPULSE;
  }
  else if(ui8Action == LEFT)                                  // -------------- LEFT -------------- //
  {

    digitalWrite(ONOFF, HIGH);
    // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);
    uiRegInterne_R[REGR_ROTATION] = 1;    // ROTATION EN COURS
  
    ui8Sens = SENS_HORAIRE;
    digitalWrite(SENS, ui8Sens);
  
    if(ui8SpeedMax_STD >= uiRegInterne_W[REGW_SPEED_STD])
    {
      ui8SpeedMax_STD = uiRegInterne_W[REGW_SPEED_STD];
    }
    else
    {
      ui8SpeedMax_STD += 20;
    }
    
    analogWrite(VITESSE, ui8SpeedMax_STD);
  
    ulTEMPOSM = millis();

   uiRegInterne_R[REGR_PULSE] = iPULSE;

    delay(500);
  }
  else if(ui8Action == RIGHT)                                 // -------------- RIGHT -------------- //
  {
    
    digitalWrite(ONOFF, HIGH);
    // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);
    uiRegInterne_R[REGR_ROTATION] = 1;      // ROTATION EN COURS
  
    ui8Sens = SENS_TRIGO;
    digitalWrite(SENS, ui8Sens);

    if(ui8SpeedMax_STD >= uiRegInterne_W[REGW_SPEED_STD])
    {
      ui8SpeedMax_STD = uiRegInterne_W[REGW_SPEED_STD];
    }
    else
    {
      ui8SpeedMax_STD += 20;
    }
        
    analogWrite(VITESSE, ui8SpeedMax_STD);

    uiRegInterne_R[REGR_PULSE] = iPULSE;
    
    ulTEMPOSM = millis();

    delay(500);
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

void Commande_Moteur(int iPosition, uint8_t ui8SpeedLimit)
{
  // COMMENT TEST sécu - digitalWrite(ONOFF, HIGH);
  
// Definition du sens de rotation
  iDiffPosition = iPosition - iPULSE;

  if(ui8ChoixSens == 1)
  {
    ui8ChoixSens = 0;
    
    if(abs(iDiffPosition) < (PULSE_MAX_ROUE/2))
    {
      if(iDiffPosition >= 0)
      {
        ui8Sens = SENS_HORAIRE;
        digitalWrite(SENS, ui8Sens);
      }
      else
      {
        ui8Sens = SENS_TRIGO;
        digitalWrite(SENS, ui8Sens);
      }
    }
    else
    {
      if(iDiffPosition >= 0)
      {
        ui8Sens = SENS_TRIGO;
        digitalWrite(SENS, ui8Sens);
      }
      else
      {
        ui8Sens = SENS_HORAIRE;
        digitalWrite(SENS, ui8Sens);
      }
    }
  }

  if(abs(iDiffPosition) <= 10)                    // amelioration de la precision a faire
  {
    digitalWrite(VITESSE, LOW);                   // Speed Moteur > PWM 0%
    // COMMENT TEST sécu - digitalWrite(ONOFF, LOW);                     // Speed Moteur > PWM 0%
    uiRegInterne_R[REGR_ROTATION] = 0;            // ROTATION ARRETEE
    
    if(uiRegInterne_W[REGW_ACTION] == POSITION)
    {
      uiRegInterne_W[REGW_ACTION] = STOP;
      uiRegInterne_R[REGR_STATUT] = POS_OK;
      ulTIMERSM = millis();
    }
    else if(uiRegInterne_W[REGW_ACTION] == SM)
    {
      ui8TempoSM = 0;
      ulTEMPOSM = millis();
    }     

    ui8Speed_RAMP = 0;
    
    uiRegInterne_R[REGR_PULSE] = iPULSE;    
  }
  else
  {
    if(abs(iDiffPosition) >= 1000)
    {
      uiRegInterne_R[REGR_PULSE] = iPULSE;  
      if(ui8Speed_RAMP >= ui8SpeedLimit)
      {
        ui8Speed_RAMP = ui8SpeedLimit;
      }
      else
      {
        ui8Speed_RAMP += 10;
      }
    }
    else
    {
      if(ui8Speed_RAMP <= VITESSE_MIN)
      {
        ui8Speed_RAMP = VITESSE_MIN;
      }
      else
      {
        ui8Speed_RAMP -= 10;
      }
    }
  }

// Choix de la vitesse  
  analogWrite(VITESSE, ui8Speed_RAMP);  
}
// eof Commande Moteur


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

void Gestion_Delay()
{
  if((millis() < ulTEMPOSM) && (millis() < ulTIMERSM) && (millis() < ulTIMERCOMM))
  {
    ulTIMERSM   = millis();
    ulTEMPOSM   = millis();
    ulTIMERCOMM = millis();
  }

  /*if((millis() - ulTIMERCOMM) >= ulCompComm)                          // Si l'arduino ne reçoit aucune trame (lecture/écriture) pendant x secondes, 
  {
    ulTIMERCOMM = millis();                                           // 
    ulTIMERSM = millis();                                             // On reboot le timer de start SM
    uiRegInterne_W[REGW_MOVE] = 0;                                    // On reboot la dernière info reçu du PIP
  }*/


  if((uiRegInterne_W[REGW_ACTION] == STOP) && ((millis() - ulTIMERSM) >= ulCompStartSM))            // ASK WAIT SM
  {
    //Serial.println("ASK SM");
    uiRegInterne_R[REGR_WAITSM] = 1;
  }

  if((uiRegInterne_W[REGW_ACTION] == SM) && ((millis() - ulTIMERSM) >= ulCompSwitchSM))            // SWITCH de sens de rotation toutes les X minutes
  {
    if(ui8SwitchSens_SM == 0)
    {
      ui8SwitchSens_SM = 1;
    }
    else
    {
      ui8SwitchSens_SM = 0;
    }
    ulTIMERSM = millis();
  }  

  if((uiRegInterne_W[REGW_ACTION] == SM) && ((millis() - ulTEMPOSM) >= ulCompTempoSM))              // TEMPO SM
  {
    ui8TempoSM = 1;
    ulTEMPOSM = millis();
  }
}
// eof Gestion Delay


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


// END PROG
