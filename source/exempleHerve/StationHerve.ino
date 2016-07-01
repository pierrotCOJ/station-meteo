// ============================================================================================================
// Station météo à partir du kit sparkfun N° SEN-08942
//
// Version 1.0
// Copyright : ph. Schnellbach sofware
//
// Date Version : 1 Juin 2014
// ============================================================================================================
//
//  Quelques explications :
//    Nom des variables :
//        commencant par 'm_' : Variables globales
//                       'm_u' : entier non signé 32 bits
//                       'm_f' : flottant 
//                       'm_i' : entier 
//                       'm_b' : boolean
//
//  Constantes de timing (INTERVAL_xxxxxx) : Ces constantes définissent la périodicité de calcul/interrogation des sondes
//
// Les constantes __DEBUG_AFF_xxxxx permettent l'affichage des valeurs dans la console. En version définitive, ces constantes devront être
//   mise en commentaire
// La constante __SIMUL__ permet de simuler les valeurs de sonde - En version définitive, cette constante devra être
//   mise en commentaire
// ============================================================================================================
#define __SIMUL__                         1 // Cette constante permet d'avoir des valeurs simulées des sondes

//#define __DEBUG_AFF_PRESSURE__                    1  // Affiche ou non la valeur de pression dans la console
//#define __DEBUG_AFF_TEMPERATURE__                 1  // Affiche ou non la valeur de température dans la console
//#define __DEBUG_AFF_HUMIDITY__                    1  // Affiche ou non la valeur d'humidite dans la console
#define __DEBUG_AFF_WINDDIR__                     1  // Affiche ou non la valeur de direction des vents dans la console
#define __DEBUG_AFF_WINDSPEED__                   1
//#define __DEBUG_AFF_WINDGUST__                    1
#define __DEBUG_AFF_BATTERIE__                    1
#define __DEBUG_AFF_RAIN__                        1

#include <avr/wdt.h> //We need watch dog for this program
#include <Wire.h> //I2C needed for sensors
#include "MPL3115A2.h" //Pressure sensor
#include "HTU21D.h" //Humidity sensor

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D    myHumidity; //Create an instance of the humidity sensor

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;

// analog I/O pins
const byte WDIR = A0;
const byte LIGHT = A1;
const byte BATT = A2;
const byte REFERENCE_3V3 = A3;

// Constante de timing
// -------------------
#define INTERVAL_CALCUL_WINDDIR      5000    // 4 sec pour le calcul de la direction du vent
#define INTERVAL_CALCUL_WINDGUST     2500	 // 2.5 secondes (2500 milisecondes) - période de calcul de rafale de vents
#define INTERVAL_CALCUL_WINDSPEED   20000    // Toutes les 20 secondes -> moyenne glissante / 10 mn pour le calcul de la vitesse de vent moyen
#define INTERVAL_CALCUL_RAINFALL     4000	 // 4 secondes (4000 milisecondes) - période de calcul des pluies
#define INTERVAL_CALCUL_TEMPHUM      4000    // 4 sec pour les calculs température et humidité
#define INTERVAL_CALCUL_BATTERIE    60000    // 60 sec pour la tension batterie
#define INTERVAL_SEND_TO_PC          8000    // 8 sec - péride d'envoi vers le pc

// Attention : la constante suivante est à réduire en cas de grossissement du programme sinon gros risque 
//  de débordement mémoire 
// Afin de ne pas avoir de débordement mémoire, les vents moyens sur cacules sur des prélèvements de vent toutes les 15 secondes
//  puis moyennés sur 10 minutes => moyenne sur 40 valeurs
// -------------------------------------------------------------------------------------------------------
#define MAX_TBL_TICK_WIND            30     // Nombre maxi de viteese de vent (toutes les 15 secondes) pour calcul moyenne des vent sur 10 mn
                                            // 1 mesure / 15 secondes => 40 mesures / 10mn 
                                            // 1        / 20          => 30 mesures
                                            // Attention pour les cartes Uno, Nano, la taille mémoire peut être vite dépassé

// Variables globales pour les critères météo
// ------------------------------------------
float                           m_fPressure = 0                         ; // Variable globale de pression (en hPa)
float                           m_fTemperature = 0                      ; // Variable globale de la temperature exterieure (en °C)
float                           m_fHumidity = 0                         ; // Variable globale de l'humidité (en %Hr)
    // Spécifiques pour les pluies
float                           m_fRainlast = 0                         ; // Variable globales des pluies/derniers
volatile unsigned long          m_uPulsesRainfall = 0                   ; // Compteur de l'ILS du pluviomètre
unsigned long                   m_uOldPulseRainFall = 0                 ; // Mémoire du compteur du pluviometre
    // Spécifiques pour les vents
volatile unsigned long          m_uPulsesWind = 0                       ; // Compteur de l'ILS de l'anémomètre
unsigned long                   m_uOldPulseWindGust = 0                 ; // Mémoire du compteur de l'anémomètre pour Rafale
unsigned long                   m_uOldPulseWindSpeed = 0               ; // Mémoire du compteur de l'anémomètre pour Vitesse de vent
unsigned long                   m_uOldMillisWindGust = 0                ; // Mémoire du timing de l'anémomètre pour Rafale
unsigned long                   m_uOldMillisWindSpeed = 0               ; // Mémoire du timing de l'anémomètre pour Vitesse de vent
int                             m_fWinddir = 0                          ; // Variable globales de la direction du vent (en °)
float                           m_fWindgust     = 0                     ; // Variable globale des rafales vents (en Km/h)
float                           m_fWindspeed     = 0                    ; // Variable globale des vents moyens / 10 mn (en km/h)
float                           m_fTblWind[120]                          ; // Table pour le calcul des vents moyens sur 10 minutes
int                             m_iIndexTblWindSpeed = 0                ; // Index dans la table pour calcul du vent moyen
boolean                         m_bNeedNewCalcGust = false              ; // Drapeau de forcage d'une nouvelle recherche rafale max
    // Spécifique pour les timing
unsigned long                   m_uCurrentMillis = 0                    ; // Millisseconde courante
unsigned long			        m_uNextCalculWindSpeed 	= 0			    ;	// Valeur temps pour le recalcul des vitesses de vent
unsigned long			        m_uNextCalculWindGust 	= 0			    ;	// Valeur temps pour le recalcul des rafales de vent
unsigned long			        m_uNextCalculWindDir 	= 0			    ;	// Valeur temps pour le recalcul de la direction du vent
unsigned long			        m_uNextCalculTempHumdity	= 0		    ;	// Valeur temps pour le recalcul de la température et l'humidité
unsigned long			        m_uNextCalculBatterie	= 0			    ;	// Valeur temps pour le recalcul du niveau batterie
unsigned long			        m_uNextSendDatas   	    = 0			    ;	// Valeur temps pour l'envoi des données vers la station
unsigned long			        m_uNextCalculRain  	    = 0			    ;	// Valeur temps pour le recalcul des pluies

//These are not wunderground values, they are just for us
float                           m_fNiveauBatterie = 11.8                 ; // Niveau de batterie
float                           m_fNiveauLight    = 0.72                 ; //
boolean                         m_bEtatLed        = false                ; // Etat de la LED

// variaibles utilisées dans les routines d'interruption (volatile obligatoire)
volatile unsigned long m_uRaintime, m_uRainlast, m_uRaininterval ;
volatile unsigned long m_uWindtime, m_uWindlast, m_uWindinterval ;

// ==========================================================================================================
// Routine d'interruption pour le pluviomètre
// ==========================================================================================================
void CountPluvio() 
{
    m_uRaintime = millis(); // récupère le timing 
    m_uRaininterval = labs(m_uRaintime - m_uRainlast); // calcul de l'interval entre deux interruptio

    // si deux interruptions en moins de 10 ms : on n'en tient pas compte (rebond de contact) 
    if (m_uRaininterval < 10) return ;
	     
    m_uPulsesRainfall ++ ;
    m_uRainlast = m_uRaintime; // conserve le temps pour la prochaine interruption
}

// =========================================================================================================
// Fonction de calcul des pluies/dernier
//  chaque basculement du pluviomètre correspond à 0.2794 mm
// =========================================================================================================
float get_rain_last ()
{
    unsigned long NbPulses = m_uPulsesRainfall ;

    float fLastRain = (float)labs(NbPulses - m_uOldPulseRainFall) * 0.2794f ;
    m_uOldPulseRainFall = NbPulses ;
    #ifdef __SIMUL__
        fLastRain = (float)random(0, 50) / 100.0f;
    #endif
    return(fLastRain);
}

// ==========================================================================================================
// Routine d'interruption pour l'anémomètre
// ==========================================================================================================
void CountWind() 
{
    m_uWindtime = millis(); // grab current time
    m_uWindinterval = labs(m_uWindtime - m_uWindlast); // calcul de l'interval entre deux interruptio

    // si deux interruptions en moins de 10 ms : on n'en tient pas compte (rebond de contact) 
    if (m_uWindinterval < 10) return ;
	     
    m_uPulsesWind ++ ;
    m_uWindlast = m_uWindtime; // conserve le temps pour la prochaine interruption
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//void rainIRQ()
//// Count rain gauge bucket tips as they occur
//// Activated by the magnet and reed switch in the rain gauge, attached to input D2
//{
//  raintime = millis(); // grab current time
//  raininterval = raintime - rainlast; // calculate interval between this and last event
//
//    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
//  {
//    dailyrainin += 0.011; //Each dump is 0.011" of water
//    rainHour[minutes] += 0.011; //Increase this minute's amount of rain
//    rainlast = raintime; // set up for next event
//  }
//}

//void wspeedIRQ()
//// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
//{
//  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
//  {
//    lastWindIRQ = millis(); //Grab the current time
//    windClicks++; //There is 1.492MPH for each click per second.
//  }
//}

// ==========================================================================================================================
// Fonction de calcul des rafales de vents (en Km/h)
//  Les rafales sont calculées à chaque passage
// ==========================================================================================================================
float get_windgust()
{
    unsigned long NbPulses = m_uPulsesWind ;

    unsigned long NbPulsesGust = labs(NbPulses - m_uOldPulseWindGust);
    unsigned long ElapseTimeGust = labs(m_uCurrentMillis - m_uOldMillisWindGust);

    float fwindgust = calcul_wind(NbPulsesGust, ElapseTimeGust);
    m_uOldPulseWindGust = NbPulses ;
    m_uOldMillisWindGust = m_uCurrentMillis ;

    #ifdef __SIMUL__
        fwindgust = (float)random(0, 500) / 400;
    #endif

    if ( fwindgust > m_fWindgust || m_bNeedNewCalcGust ) m_fWindgust = fwindgust ;

    m_bNeedNewCalcGust = false ;
    
    return(m_fWindgust);
}

// ===============================================================================
// Calcul de la vitesse du vent :
//  en entrée :
//     unsigned long NbPulses : Nbr d'impulsions de l'ILS depuis le dernier calcul
//     unsigned long Duration : temps en ms passé depuis le dernier calcul
// ===============================================================================
float calcul_wind( unsigned long NbPulses, unsigned long DurationMilliSecondes )
{
	// Calcul du temps passé en secondes
	// --------------------------------
	if ( DurationMilliSecondes == 0 || !NbPulses ) return(0.0f);


    // calcul du temps de mesure (en secondes)
    // ---------------------------------------
	float TimeOfMesure = (float)DurationMilliSecondes / 1000.0 ;

	// calcul du nbr de tour par seconde
	// ---------------------------------
	float NbToursBySecond = (float)NbPulses / TimeOfMesure ; 

	// Calcul de la vitesse du vent  (en m/s)
	// --------------------------------------
	float fWind = NbToursBySecond * 2.401141f ;

	return(fWind);
}
 
// ===========================================================================================
// Calcul des vitesses de Vent
//           La vitesse moyenne du vent est calculée sur une période de 10 minutes (INTERVAL_CALCUL_WINDSPEED)
//           conformement au préco Meteo-France
// ===========================================================================================
float get_Windspeed()
{
    unsigned long NbPulses = m_uPulsesWind ;
    unsigned long NbPulsesSpeed = labs(NbPulses - m_uOldPulseWindSpeed);
    unsigned long ElapseTimeSpeed = labs(m_uCurrentMillis - m_uOldMillisWindSpeed);


    float fWind = calcul_wind(NbPulsesSpeed, ElapseTimeSpeed);
 
    #ifdef __SIMUL__
        fWind = (float)random(0, 500) / 400;
    #endif
    m_uOldPulseWindSpeed = NbPulses ;
    m_uOldMillisWindSpeed = m_uCurrentMillis ;

	// Si le tableau m_TblWindSpeed est déjà rempli : FIFO
	// ------------------------------------------------------
	if ( m_iIndexTblWindSpeed >= MAX_TBL_TICK_WIND ) {

    	#ifdef __DEBUG_AFF_WINDSPEED__
	    	Serial.print("CalculWindSeed : Rotation tbl depuis :");
	    	Serial.println(m_iIndexTblWindSpeed);
	    #endif
		for (int i = 0 ; i < (MAX_TBL_TICK_WIND-1) ; i++ ) {
			m_fTblWind[i] = m_fTblWind[i+1] ;
		}
		m_iIndexTblWindSpeed = MAX_TBL_TICK_WIND - 1 ;
	}

    if ( m_iIndexTblWindSpeed < MAX_TBL_TICK_WIND ) {
        m_fTblWind[m_iIndexTblWindSpeed] = fWind ;
        m_iIndexTblWindSpeed++ ;
    }

	float somme = 0.0 ;
	for (int i = 0 ; i < m_iIndexTblWindSpeed ; i++ ) {
		somme += m_fTblWind[i] ;
	}

    float fWindSpeed = somme/(float)m_iIndexTblWindSpeed ;

	#ifdef __DEBUG_AFF_WINDSPEED__
		Serial.print("- Vitesse : ");
		Serial.print(fWindSpeed);
		Serial.println( " - Fin calcul windspeed");
	#endif

	return(fWindSpeed);
}

// =====================================================================================================================
// Routine d'intialisation
// =====================================================================================================================
void setup()
{
    wdt_reset(); //Pet the dog
    wdt_disable(); //We don't want the watchdog during init

    Serial.begin(115200);
  //imp.begin(19200);

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

    pinMode(WDIR, INPUT);
    pinMode(LIGHT, INPUT);
    pinMode(BATT, INPUT);
    pinMode(REFERENCE_3V3, INPUT);
  
    pinMode(STAT1, OUTPUT);

    //Configure the pressure sensor
    myPressure.begin(); // Get sensor online
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(128); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
    myPressure.setModeActive(); // Go to active mode and start measuring!

    //Configure the humidity sensor
    myHumidity.begin();

    // Raz du tableau pour les vents moyens
    memset ( m_fTblWind, 0, sizeof(float) * MAX_TBL_TICK_WIND);

    // attache m'interruption pour l'ILS du pluviometre
    // ------------------------------------------------
    attachInterrupt(0, CountPluvio, FALLING);

    // attache m'interruption pour l'ILS de l'anémomètre
    // ------------------------------------------------
    attachInterrupt(1, CountWind, FALLING);

    // turn on interrupts
    interrupts();

    #ifdef __SIMUL__
        randomSeed(10);
    #endif

    // Initialisation des variables de timing
    m_uNextCalculWindSpeed 	= millis()			    ;	// Valeur temps pour le recalcul des vitesses de vent
    m_uNextCalculWindGust 	= millis()			    ;	// Valeur temps pour le recalcul des rafales de vent
    m_uNextCalculWindDir 	= millis()			    ;	// Valeur temps pour le recalcul de la direction du vent
    m_uNextCalculTempHumdity = millis()		        ;	// Valeur temps pour le recalcul de la température et l'humidité
    m_uNextCalculBatterie	= millis()			    ;	// Valeur temps pour le recalcul du niveau batterie
    m_uNextSendDatas   	    = millis()			    ;	// Valeur temps pour l'envoi des données vers la station
    m_uNextCalculRain  	    = millis()			    ;	// Valeur temps pour le recalcul des pluies
    m_uOldMillisWindSpeed   = millis() ;
    m_uOldMillisWindGust    = millis() ;

    wdt_enable(WDTO_1S); //Unleash the beast
}

// =====================================================================================================================
// Routine  de boucle (effectuée en rotation)
// =====================================================================================================================
void loop()
{
    // On  met à jour la variable de la milliseconde courante
    // ------------------------------------------------------
    m_uCurrentMillis = millis() ;

    wdt_reset(); //Pet the dog

 	// Si le timer > INTERVAL_CALCUL_TEMPHUM : Mesure temperature, humidité et pression
	// --------------------------------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculTempHumdity ) {
	    
        // Interrogation de l'Humidité (en %Hr) depuis la carte HTU21D
        // -----------------------------------------------------------
        m_fHumidity = myHumidity.readHumidity();

        // Interrogation de la température (en °C) depuis la carte HTU21D
        // --------------------------------------------------------------
        m_fTemperature = myHumidity.readTemperature();

        // Interrogation de la pression (en hPA)
        // -------------------------------------
        m_fPressure = myPressure.readPressure() / 100.0f ;
        #ifdef __SIMUL__
            m_fHumidity = 80 + (float)random(0, 2000)/100.0f;
            m_fTemperature = 20 + (float)random(0, 200)/100.0f;
            m_fPressure = 1013.0f + (float)random(0, 200)/100.0f;
        #endif
        #ifdef __DEBUG_AFF_PRESSURE__
            Serial.print ( "Pression = ");
            Serial.println(m_fPressure);
        #endif
         #ifdef __DEBUG_AFF_TEMPERATURE__
            Serial.print ( "Temperature = ");
            Serial.println(m_fTemperature);
        #endif
         #ifdef __DEBUG_AFF_HUMIDITY__
            Serial.print ( "Humidite = ");
            Serial.println(m_fHumidity);
        #endif
       m_uNextCalculTempHumdity  = m_uCurrentMillis + INTERVAL_CALCUL_TEMPHUM ;
	}

    // Si le timer > INTERVAL_CALCUL_WINDDIR : Calcul Direction du vent
	// ----------------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculWindDir ) {
	    
        m_fWinddir   = get_wind_direction();
        #ifdef __DEBUG_AFF_WINDDIR__
            Serial.print ( "Direction Vent = ");
            Serial.println(m_fWinddir);
        #endif
        m_uNextCalculWindDir  = m_uCurrentMillis + INTERVAL_CALCUL_WINDDIR ;
	}

	// Si le timer > INTERVAL_CALCUL_WINDGUST : Calcul des rafales
	// ------------------------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculWindGust || m_bNeedNewCalcGust ) {

        m_fWindgust = get_windgust();
        #ifdef __DEBUG_AFF_WINDGUST__
            Serial.print ( "Rafale = ");
            Serial.println(m_fWindgust);
        #endif
		m_uNextCalculWindGust  = m_uCurrentMillis + INTERVAL_CALCUL_WINDGUST ;
 	}

	// Si le timer > INTERVAL_CALCUL_WINDSPEED : Calcul des vents moyens
	// ------------------------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculWindSpeed ) {

        m_fWindspeed = get_Windspeed();
        #ifdef __DEBUG_AFF_WINDSPEED__
            Serial.print ( "Vent moyen = ");
            Serial.println(m_fWindspeed);
        #endif
        m_uNextCalculWindSpeed  = m_uCurrentMillis + INTERVAL_CALCUL_WINDSPEED ;
 	}

	// Si le timer > INTERVAL_CALCUL_BATTERIE : Mesure le niveau de la batterie
	// ------------------------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculBatterie ) {

        // Interrogation du niveau de batterie
        // -----------------------------------
        m_fNiveauBatterie = get_battery_level();
 
         #ifdef __DEBUG_AFF_BATTERIE__
            Serial.print ( "Niveau batterie = ");
            Serial.println(m_fNiveauBatterie);
        #endif
       // Interrogation du capteur de lumière
        // ----------------------------------
        m_fNiveauLight = get_light_level();

        m_uNextCalculBatterie  = m_uCurrentMillis + INTERVAL_CALCUL_BATTERIE ;
 	}
    
	// Si le timer > INTERVAL_CALCUL_RAINFALL : Mesure la pluie
	// --------------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextCalculRain ) {

        // Interrogation du pluviometre
        // ----------------------------
        m_fRainlast += get_rain_last();
 
         #ifdef __DEBUG_AFF_RAIN__
            Serial.print ( "Pluie/Dernier = ");
            Serial.println(m_fRainlast);
        #endif
        m_uNextCalculRain  = m_uCurrentMillis + INTERVAL_CALCUL_RAINFALL ;
 	}
    
    
    // Si le timer > INTERVAL_SEND_TO_PC : Envoi vers le PC
	// ----------------------------------------------------
    if ( m_uCurrentMillis >= m_uNextSendDatas ) {

         // Change l'état de la LED
        m_bEtatLed = !m_bEtatLed ;
        digitalWrite(STAT1, m_bEtatLed);

        // Envoi trame vers le PC
        ReportWeather(); 

        // Raz des pluies/dernier
        m_fRainlast = 0 ;

        // Force un nouveau calcul de rafale
        m_bNeedNewCalcGust = true ;
        m_uNextSendDatas  = m_uCurrentMillis + INTERVAL_SEND_TO_PC ;
 	}
}


// ===================================================================================================================
// Renvoie la tension du capteur de lumière sur la base du rail de 3,3 V 
// Cela permet d'ignorer ce qui pourrait être VCC (un Arduino branché en USB a VCC de 4,5 à 5,2 V)
// ===================================================================================================================
float get_light_level()
{
    float fOperatingVoltage = averageAnalogRead(REFERENCE_3V3);

    float fLightSensor = averageAnalogRead(LIGHT);
  
    fOperatingVoltage = 3.3 / fOperatingVoltage; //The reference voltage is 3.3V
  
    fLightSensor *= fOperatingVoltage;
  
    #ifdef __SIMUL__
        fLightSensor = (float)random(0, 100) / 100.0f;
    #endif
    return(fLightSensor);
}
 
// ========================================================================================================
// Renvoie la tension de la broche brute sur la base du rail de 3,3 V
// Cela permet d'ignorer ce qui pourrait être VCC (un Arduino branché en USB a VCC de 4,5 à 5,2 V)
//  Niveau de la batterie est relié à la broche RAW sur Arduino et est alimenté par deux résistances de 5%:
// 3.9K sur le côté haut (R1), et 1K sur le côté faible (R2)float get_battery_level()
// =========================================================================================================
float get_battery_level()
{
    float fOperatingVoltage = averageAnalogRead(REFERENCE_3V3);

    float fRawVoltage = averageAnalogRead(BATT);
  
    fOperatingVoltage = 3.30 / fOperatingVoltage; //The reference voltage is 3.3V
  
    fRawVoltage *= fOperatingVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
    fRawVoltage *= 4.90; //(3.9k+1k)/1k - multiply BATT voltage by the voltage divider to get actual system voltage
  
    #ifdef __SIMUL__
        fRawVoltage = (float)random(10, 100) / 100.0f;
    #endif
    return(fRawVoltage);
}


int get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
    #ifdef __SIMUL__
        return (random(0, 360));
    #endif

  unsigned int adc;

  adc = averageAnalogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);

  return (-1); // error, disconnected?
}


//Reports the weather string to the Imp
void ReportWeather()
{
    Serial.print("#METEO#[");
    Serial.print(m_fWinddir);
    Serial.print(";");
    Serial.print(m_fWindspeed);
    Serial.print(";");
    Serial.print(m_fWindgust);
    Serial.print(";");
    Serial.print(m_fTemperature, 2);
    Serial.print(";");
    Serial.print(m_fHumidity, 1);
    Serial.print(";");
    Serial.print(m_fPressure, 2);  // passage aussi en hectopascal
    Serial.print(";");
    Serial.print(m_fRainlast, 2);
    Serial.print(";");
    Serial.print(m_fNiveauBatterie,1);
    Serial.print(";");
    Serial.println("]#");
 }

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}




