#include "functions.h"

FastCRC16 CRC16;
#define CRC_SW 0

// oled display
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_FAST); // Dev 0, Fast I2C / TWI

bool TransponderDataWaiting = false;
byte TransponderBuffer[256] = {0};
//char * temppch;
String temppch = "";
float MagCourseFloat = 0;
byte Token = 0;

int MagCourse = 0;
int Geber1 = 0;
int Geber2 = 0;
int WindGeschwindigkeit = 0;
int WindRichtung = 0;
int FDW = 0;

int MagCourseTrans = 0;
int Geber1Trans = 0;
int Geber2Trans = 0;
int WindGeschwindigkeitTrans = 0;
int WindRichtungTrans = 0;
int FDWTrans = 0;

int NMEACourseActualTrans = 0;
int NMEACourseToSteerTrans = 0;
int NMEACPNPilotActiveTrans = 0;
int NMEADistanceToWPTrans = 0;
int NMEASOGTrans = 0;
int NMEANextWPTrans = 0;

int NMEACPNPilotActive = 0;
int NMEACourseActual = 0;
int NMEACourseToSteer = 0;
int NMEADistanceToWP = 0;
int NMEANextWP = 0;
int NMEASOG = 0;

bool GotDataBlinker = false;

//Rudergeber
int Geber1Middle;
int Geber2Middle;
int Geber1Minus;
int Geber2Minus;
int Geber1Plus;
int Geber2Plus;
int GeberInPercent;
int GeberRichtung;

char GeberBoxStart;
char GeberBoxEnd;

// Variablen fuer Kompass Kalibration
int  MagOffset = 0;
bool MagCalbDone = false;
bool MagInCalc = false;

//Magnetischer Pilot
bool MagPilotEnabled = false;
int MagCourseToSteer = 0;
int CourseAbweichung = 0;
int MagCourseLast = 0;

int CourseAbweichungEimer[numReadingsCourseAbweichung];
long TotalCourseAbweichungEimer = 0;
int AverageCourseAbweichung;
int readIndexCourseAbweichung = 0;


// Variablen fuer Tastenfeld
bool TasteReadA             = false;
bool TasteReadB             = false;
bool TasteReadC             = false;
bool TasteReadD             = false;
bool TasteStateA            = false;
bool TasteStateB            = false;
bool TasteStateC            = false;
bool TasteStateD            = false;
bool TasteLangDruckStateA   = false;
bool TasteLangDruckStateB   = false;
bool TasteLangDruckStateC   = false;
bool TasteLangDruckStateD   = false;
bool TasteLastReadA         = false;
bool TasteLastReadB         = false;
bool TasteLastReadC         = false;
bool TasteLastReadD         = false;
unsigned long TasteLastDebounceTime  = 0;
unsigned char debounceDelay          = 10;
uint16_t LangDruckRunner             = 0;

// Laufvariablen
uint16_t runner = 0;
byte readIndex;
byte LastMenue;
byte ActualScreen = 0;
byte ActualScreenPrev;

// Debug zum Timing testen
unsigned long MicroStart;
unsigned long MicroStop;

//0 = Standby, 1 = Mag, 2 = Wind, 3 = OpenCPN
byte OperationMode = 0;

//Motor
byte MotorActualSetupScreen = 0;
int MotorDirection; //(-1 = reverse, 1 = forward)
bool MotorRunning = false;
uint8_t MotorSpeed = 0;
bool MotorEnabled = false;
int MotorSetPoint;
int MotorSetPointDirection;
//int MotorGeberSchlacker = 0;
bool MotorStop = false;
byte MotorMaximalSpeed;
byte MotorMinimalSpeed;

//WindPilot
bool WindPilotEnabled = false;
int WindSeite;
int WindCourseToSteer = 0;
int WindSeiteToSteer = -1;
char WindgeschwindigkeitStr[6];
int WindRichtungLast;
int WindSeiteLast;

//OpenCPPN
bool OpenCPNPilotEnabled = false;
int OpenCPNCourseLast = 0;

//logge
char FDWStr[6];
float FDWCorr;

// CRC
unsigned long CRCMicro;
unsigned long CRCMega;
bool CRCValid;
unsigned int CRCFailCounter = 0;
byte buffer[256] = "";

//Drehrate
int DrehrateEimer[numReadingsDrehrate];
long TotalDrehrateEimer = 0;
int Drehrate = 0;
float AverageDrehrate;
int readIndexDrehrate = 0;

//PID
float Kp;
float Kd;
float Ki;
float Ta;
float PIDOutAbweich;
float PIDOutDreh;
float PIDOutAbweichAlt = 0;
float PIDOutDrehAlt = 0;
float PIDOutAbweichSum = 0;
float PIDOutDrehSum = 0;
int PIDRuderlage;

float RuderLageEimer[numReadingsRuderLage];
float TotalRuderLageEimer;
float PIDAverageRuderLage;
int readIndexRuderLage;

//Regelhäufigkeits Parameter
float PilotTmax = 50.0; 						//Maximale Zeit zwischen Ruderänderungen (*100 == ms!) --> EEPROM
float PilotTmin = 5.0;							//Minimale Zeit zwischen Ruderänderungen (*100 == ms!) --> EEPROM
float TSteilheit = 50.0; 						//Steilheit bei 1/x 																	 --> EEPROM
float FDWMultiplikator = 0.5;				//Wie stark geht FDW ein 															 --> EEPROM

byte WaitUntiStart;
float DrehRuder;

unsigned long PilotNextCheck;

void ReadEEPROM(void) {
#if DEBUG
  Serial.println("EEProm Read...");
#endif

  MagOffset = EEPROM.read(HighMagOffsetAdr);
  MagOffset = (MagOffset << 8) + EEPROM.read(LowMagOffsetAdr);

  Geber1Middle = EEPROM.read(Geber1MiddleHighAdr);
  Geber1Middle = (Geber1Middle << 8) + EEPROM.read(Geber1MiddleLowAdr);

  Geber2Middle = EEPROM.read(Geber2MiddleHighAdr);
  Geber2Middle = (Geber2Middle << 8) + EEPROM.read(Geber2MiddleLowAdr);

  Geber1Minus = EEPROM.read(Geber1MinusHighAdr);
  Geber1Minus = (Geber1Minus << 8) + EEPROM.read(Geber1MinusLowAdr);

  Geber2Minus = EEPROM.read(Geber2MinusHighAdr);
  Geber2Minus = (Geber2Minus << 8) + EEPROM.read(Geber2MinusLowAdr);

  Geber1Plus = EEPROM.read(Geber1PlusHighAdr);
  Geber1Plus = (Geber1Plus << 8) + EEPROM.read(Geber1PlusLowAdr);

  Geber2Plus = EEPROM.read(Geber2PlusHighAdr);
  Geber2Plus = (Geber2Plus << 8) + EEPROM.read(Geber2PlusLowAdr);

  WindCourseToSteer = EEPROM.read(WindCourseToSteerAdr);
  WindSeiteToSteer = (char)EEPROM.read(WindSeiteToSteerAdr);

  WindCourseToSteer = WindCourseToSteer * WindSeiteToSteer;

  MagCourseToSteer = EEPROM.read(MagCourseToSteerHighAdr);
  MagCourseToSteer = (MagCourseToSteer << 8) + EEPROM.read(MagCourseToSteerLowAdr);

  MotorMaximalSpeed = EEPROM.read(MotorMaximalSpeedAdr);
  MotorMinimalSpeed = EEPROM.read(MotorMinimalSpeedAdr);

  Kp = (float)EEPROM.read(KpVorKommaAdr) + ((float)EEPROM.read(KpNachKommaAdr) / 100.00);
  Kd = (float)EEPROM.read(KdVorKommaAdr) + ((float)EEPROM.read(KdNachKommaAdr) / 100.00);
  Ki = (float)EEPROM.read(KiVorKommaAdr) + ((float)EEPROM.read(KiNachKommaAdr) / 100.00);
  Ta = (float)EEPROM.read(TaVorKommaAdr) + ((float)EEPROM.read(TaNachKommaAdr) / 100.00);

  FDWCorr = (float)EEPROM.read(FDWCorrVorKommaAdr) + ((float)EEPROM.read(FDWCorrNachKommaAdr) / 100.00);

  PilotTmax = (float)EEPROM.read(PilotTmaxVorKommaAdr) + ((float)EEPROM.read(PilotTmaxNachKommaAdr) / 100.00);
  PilotTmin = (float)EEPROM.read(PilotTminVorKommaAdr) + ((float)EEPROM.read(PilotTminNachKommaAdr) / 100.00);
  TSteilheit = (float)EEPROM.read(TSteilheitVorKommaAdr) + ((float)EEPROM.read(TSteilheitNachKommaAdr) / 100.00);
  FDWMultiplikator = (float)EEPROM.read(FDWMultiplikatorVorKommaAdr) + ((float)EEPROM.read(FDWMultiplikatorNachKommaAdr) / 100.00);
}

void WriteEEPROM(void) {

#if DEBUG
  Serial.println("Writing to EEProm!!!11");
#endif

  if (EEPROM.read(HighMagOffsetAdr) != (byte)((MagOffset & 0xFF00) >> 8)) {
    EEPROM.write(HighMagOffsetAdr, (byte)((MagOffset & 0xFF00) >> 8));
  }
  if (EEPROM.read(LowMagOffsetAdr) != (byte)(MagOffset & 0x00FF)) {
    EEPROM.write(LowMagOffsetAdr , (byte)(MagOffset & 0x00FF) );
  }
  if (EEPROM.read(Geber1MiddleHighAdr) != (byte)((Geber1Middle & 0xFF00) >> 8)) {
    EEPROM.write(Geber1MiddleHighAdr, (byte)((Geber1Middle & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber1MiddleLowAdr) != (byte)(Geber1Middle & 0x00FF)) {
    EEPROM.write(Geber1MiddleLowAdr , (byte)(Geber1Middle & 0x00FF) );
  }
  if (EEPROM.read(Geber2MiddleHighAdr) != (byte)((Geber2Middle & 0xFF00) >> 8)) {
    EEPROM.write(Geber2MiddleHighAdr, (byte)((Geber2Middle & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber2MiddleLowAdr) != (byte)(Geber2Middle & 0x00FF)) {
    EEPROM.write(Geber2MiddleLowAdr , (byte)(Geber2Middle & 0x00FF) );
  }

  if (EEPROM.read(Geber1MinusHighAdr) != (byte)((Geber1Minus & 0xFF00) >> 8)) {
    EEPROM.write(Geber1MinusHighAdr, (byte)((Geber1Minus & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber1MinusLowAdr) != (byte)(Geber1Minus & 0x00FF)) {
    EEPROM.write(Geber1MinusLowAdr , (byte)(Geber1Minus & 0x00FF) );
  }
  if (EEPROM.read(Geber2MinusHighAdr) != (byte)((Geber2Minus & 0xFF00) >> 8)) {
    EEPROM.write(Geber2MinusHighAdr, (byte)((Geber2Minus & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber2MinusLowAdr) != (byte)(Geber2Minus & 0x00FF)) {
    EEPROM.write(Geber2MinusLowAdr , (byte)(Geber2Minus & 0x00FF) );
  }

  if (EEPROM.read(Geber1PlusHighAdr) != (byte)((Geber1Plus & 0xFF00) >> 8)) {
    EEPROM.write(Geber1PlusHighAdr, (byte)((Geber1Plus & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber1PlusLowAdr) != (byte)(Geber1Plus & 0x00FF)) {
    EEPROM.write(Geber1PlusLowAdr , (byte)(Geber1Plus & 0x00FF) );
  }
  if (EEPROM.read(Geber2PlusHighAdr) != (byte)((Geber2Plus & 0xFF00) >> 8)) {
    EEPROM.write(Geber2PlusHighAdr, (byte)((Geber2Plus & 0xFF00) >> 8));
  }
  if (EEPROM.read(Geber2PlusLowAdr) != (byte)(Geber2Plus & 0x00FF)) {
    EEPROM.write(Geber2PlusLowAdr , (byte)(Geber2Plus & 0x00FF) );
  }

  if (EEPROM.read(MotorMaximalSpeedAdr) != MotorMaximalSpeed) {
    EEPROM.write(MotorMaximalSpeedAdr , MotorMaximalSpeed);
  }
  if (EEPROM.read(MotorMinimalSpeedAdr) != MotorMinimalSpeed) {
    EEPROM.write(MotorMinimalSpeedAdr , MotorMinimalSpeed);
  }

  if (EEPROM.read(KpVorKommaAdr) != (int)Kp) {
    EEPROM.write(KpVorKommaAdr, (int)Kp);
  }
  if (EEPROM.read(KpNachKommaAdr) != int(((Kp + 0.001) - (int)Kp) * 100.00)) {
    EEPROM.write(KpNachKommaAdr, int(((Kp + 0.001) - (int)Kp) * 100.00));
  }
  if (EEPROM.read(KdVorKommaAdr) != (int)Kd) {
    EEPROM.write(KdVorKommaAdr, (int)Kd);
  }
  if (EEPROM.read(KdNachKommaAdr) != int(((Kd + 0.001) - (int)Kd) * 100.00)) {
    EEPROM.write(KdNachKommaAdr, int(((Kd + 0.001) - (int)Kd) * 100.00));
  }
  if (EEPROM.read(KiVorKommaAdr) != (int)Ki) {
    EEPROM.write(KiVorKommaAdr, (int)Ki);
  }
  if (EEPROM.read(KiNachKommaAdr) != int(((Ki + 0.001) - (int)Ki) * 100.00)) {
    EEPROM.write(KiNachKommaAdr, int(((Ki + 0.001) - (int)Ki) * 100.00));
  }
  if (EEPROM.read(TaVorKommaAdr) != (int)Ta) {
    EEPROM.write(TaVorKommaAdr, (int)Ta);
  }
  if (EEPROM.read(TaNachKommaAdr) != int(((Ta + 0.001) - (int)Ta) * 100.00)) {
    EEPROM.write(TaNachKommaAdr, int(((Ta + 0.001) - (int)Ta) * 100.00));
  }

  if (EEPROM.read(FDWCorrVorKommaAdr) != (int)FDWCorr) {
    EEPROM.write(FDWCorrVorKommaAdr, (int)FDWCorr);
  }
  if (EEPROM.read(FDWCorrNachKommaAdr) != int(((FDWCorr + 0.001) - (int)FDWCorr) * 100.00)) {
    EEPROM.write(FDWCorrNachKommaAdr, int(((FDWCorr + 0.001) - (int)FDWCorr) * 100.00));
  }

  if (EEPROM.read(PilotTmaxVorKommaAdr) != (int)PilotTmax) {
    EEPROM.write(PilotTmaxVorKommaAdr, (int)PilotTmax);
  }
  if (EEPROM.read(PilotTmaxNachKommaAdr) != int(((PilotTmax + 0.001) - (int)PilotTmax) * 100.00)) {
    EEPROM.write(PilotTmaxNachKommaAdr, int(((PilotTmax + 0.001) - (int)PilotTmax) * 100.00));
  }

  if (EEPROM.read(PilotTminVorKommaAdr) != (int)PilotTmin) {
    EEPROM.write(PilotTminVorKommaAdr, (int)PilotTmin);
  }
  if (EEPROM.read(PilotTminNachKommaAdr) != int(((PilotTmin + 0.001) - (int)PilotTmin) * 100.00)) {
    EEPROM.write(PilotTminNachKommaAdr, int(((PilotTmin + 0.001) - (int)PilotTmin) * 100.00));
  }

  if (EEPROM.read(TSteilheitVorKommaAdr) != (int)TSteilheit) {
    EEPROM.write(TSteilheitVorKommaAdr, (int)TSteilheit);
  }
  if (EEPROM.read(TSteilheitNachKommaAdr) != int(((TSteilheit + 0.001) - (int)TSteilheit) * 100.00)) {
    EEPROM.write(TSteilheitNachKommaAdr, int(((TSteilheit + 0.001) - (int)TSteilheit) * 100.00));
  }

  if (EEPROM.read(FDWMultiplikatorVorKommaAdr) != (int)FDWMultiplikator) {
    EEPROM.write(FDWMultiplikatorVorKommaAdr, (int)FDWMultiplikator);
  }
  if (EEPROM.read(FDWMultiplikatorNachKommaAdr) != int(((FDWMultiplikator + 0.001) - (int)FDWMultiplikator) * 100.00)) {
    EEPROM.write(FDWMultiplikatorNachKommaAdr, int(((FDWMultiplikator + 0.001) - (int)FDWMultiplikator) * 100.00));
  }
}

void WriteEEPROMWind(void) {
#if DEBUG
  Serial.println("Writing to EEProm!!! (Wind)");
#endif
  if (EEPROM.read(WindCourseToSteerAdr) != abs(WindCourseToSteer)) {
    EEPROM.write(WindCourseToSteerAdr, abs(WindCourseToSteer));
  }
  if (EEPROM.read(WindSeiteToSteerAdr) != WindSeiteToSteer) {
    EEPROM.write(WindSeiteToSteerAdr , WindSeiteToSteer);
  }
}

void WriteEEPROMMag(void) {
#if DEBUG
  Serial.println("Writing to EEProm!!! (Mag)");
#endif
  if (EEPROM.read(MagCourseToSteerHighAdr) != (byte)((MagCourseToSteer & 0xFF00) >> 8)) {
    EEPROM.write(MagCourseToSteerHighAdr, (byte)((MagCourseToSteer & 0xFF00) >> 8));
  }
  if (EEPROM.read(MagCourseToSteerLowAdr) != (byte)(MagCourseToSteer & 0x00FF)) {
    EEPROM.write(MagCourseToSteerLowAdr , (byte)(MagCourseToSteer & 0x00FF) );
  }
}

void InterruptTimerSetUp(void) {
  //Setup Timer2 to fire every 8ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  //  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  TCCR2B = 0x07;        //Timer2 Control Reg B: Timer Prescaler set to 1024
}

void DisplaySetUp(void) {
  /*
    // assign default color value
    if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
      u8g.setColorIndex(255);     // white
    }
    else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
      u8g.setColorIndex(3);         // max intensity
    }
    else if ( u8g.getMode() == U8G_MODE_BW ) {
      u8g.setColorIndex(1);         // pixel on
    }
    else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
      u8g.setHiColorByRGB(255,255,255);
    }
  */
  //ich nutze ur das monochrome - sinnlose Speicherverschwendung - deshalb auskommentiert
  u8g.setColorIndex(1);         // pixel on

  //  myPID.SetMode(AUTOMATIC);
}

void HandleMenue(void) {
  switch (LastMenue)  {
    case Menu_Main: //Hauptbild -> Druck auf "SetUp"
      //zeigen Kompasskalibration / Hauptbild mit aktuellen Parametern
      //Über Langdruck gemacht!
      //  DisplayShowScreen(Menu_Compass_Main_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
      break;
    case Menu_Compass_Calibrate_SetUp:
      MagInCalc = false;
      MagCalbDone = true;
      //zeigen Kompasskalibration / Hauptbild mit aktuellen Parametern
      DisplayShowScreen(Menu_Compass_Main_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
      break;
    case Menu_Compass_Main_SetUp: //SetUp -> Druck auf "Back"
    case Menu_Motor_Main_SetUp:
    case Menu_Motor_Main_Test_Run_SetUp:
    case Menu_Motor_Main_Test_Stop_SetUp:
    case Menu_Rudder_Main_SetUp:
    case Menu_Debug_SetUp:
    case Menu_MagWind_Pilot_ON:
    case Menu_MagWind_Pilot_OFF:
    case Menu_PID_SetUp:
    case Menu_Logge_SetUp:
    case Menu_Dynamics_SetUp:
    case Menu_OpenCPN_Pilot_OFF:
    case Menu_OpenCPN_Pilot_ON:
      WriteEEPROMMag();
      WriteEEPROMWind();
      if (MotorActualSetupScreen == 0) {
        //Motor Setup Menue verlassen
        ReadEEPROM();
        OperationMode = 0;
        DisplayShowScreen(Menu_Main, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
      }
      else if (MotorActualSetupScreen == 1) {
        MotorActualSetupScreen = 0;
        //aus dem Testmenue zurueck
        DisplayShowScreen(Menu_Motor_Main_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
      }
      break;
    case Menu_Motor_Limits_SetUp:
      DisplayShowScreen(Menu_Motor_Main_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
      break;
    case Menu_Motor_Main_Test_State_SetUp:
    case Menu_Motor_Main_Test_Speed_SetUp:
      MotorActualSetupScreen = 1;
      DisplayShowScreen((Menu_Motor_Main_Test_Run_SetUp + MotorRunning), GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
      break;
    case Menu_Rudder_Zero_SetUp:
    case Menu_Rudder_Limits_SetUp:
      DisplayShowScreen(Menu_Rudder_Main_SetUp, GUI_RudderSetUp); //Menue, Screen (-1  for nothing)
      break;
    case Menu_Motor_Limits_PlusMinus_SetUp:
      DisplayShowScreen(Menu_Motor_Limits_SetUp, GUI_MotorSpeedLimitsSetUp);
      break;
    case Menu_PID_Ta_SetUp:
    case Menu_PID_P_SetUp:
    case Menu_PID_I_SetUp:
    case Menu_PID_D_SetUp:
      DisplayShowScreen(Menu_PID_SetUp, GUI_PIDSetUp);
      break;
    case Menu_Logge_Adjust_SetUp:
      DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp);
      break;
    case Menu_Dynamics_TMax_SetUp:
    case Menu_Dynamics_TMin_SetUp:
    case Menu_Dynamics_Steilheit_SetUp:
    case Menu_Dynamics_STWMult_SetUp:
      DisplayShowScreen(Menu_Dynamics_SetUp, GUI_DynamicSetUp);
      break;
    case Menu_Debug_NMEA_SetUp:
      DisplayShowScreen(Menu_Debug_SetUp, GUI_433MHzDebug);
      break;
    default:
      break;
  }
}

// Menuezeile
void SetMenue(byte menue) {
  LastMenue = menue;
  u8g.setFont(u8g_font_6x10r);
  u8g.drawLine(0, 53, 128, 53);
  switch (menue) {
    case Menu_Main:
      u8g.drawStr(0, 62, "SetUp Route Wind Magn");
      break;
    case Menu_Compass_Main_SetUp:
      u8g.drawStr(0, 62, "Back Save Calib Logge");
      break;
    case Menu_Compass_Calibrate_SetUp:
      u8g.drawStr(0, 62, "Back        +++   ---");
      break;
    case Menu_Motor_Main_SetUp:
      u8g.drawStr(0, 62, "Back Param Test Ruder");
      break;
    case Menu_Motor_Main_Test_Run_SetUp:
      u8g.drawStr(0, 62, "Back State Speed  Run");
      break;
    case Menu_Motor_Main_Test_Stop_SetUp:
      u8g.drawStr(0, 62, "Back State Speed Stop");
      break;
    case Menu_Motor_Main_Test_State_SetUp:
      u8g.drawStr(0, 62, "Back On/Off Port StPd");
      break;
    case Menu_Motor_Main_Test_Speed_SetUp:
      u8g.drawStr(0, 62, "Back         +++  ---");
      break;
    case Menu_Rudder_Main_SetUp:
      u8g.drawStr(0, 62, "Back Limits  Zero PID");
      break;
    case Menu_Rudder_Zero_SetUp:
      u8g.drawStr(0, 62, "Back             Save");
      break;
    case Menu_Rudder_Limits_SetUp:
      u8g.drawStr(0, 62, "Back -Set- +Set+ Save");
      break;
    case Menu_Debug_SetUp:
      u8g.drawStr(0, 62, "Back  Clear UART Comp");
      break;
    case Menu_MagWind_Pilot_ON:
      u8g.drawStr(0, 62, "Back   <<<  >>>   ON");
      break;
    case Menu_MagWind_Pilot_OFF:
      u8g.drawStr(0, 62, "Back   <<<  >>>   OFF");
      break;
    case Menu_Motor_Limits_SetUp:
      u8g.drawStr(0, 62, "Back  Save   Min  Max");
      break;
    case Menu_Motor_Limits_PlusMinus_SetUp:
      u8g.drawStr(0, 62, "Back        +++   ---");
      break;
    case Menu_PID_SetUp:
      u8g.drawStr(0, 62, "Back Save SetUp Dynam");
      break;
    case Menu_PID_Ta_SetUp:
      u8g.drawStr(0, 62, "Back   Kp   +++   ---");
      break;
    case Menu_PID_P_SetUp:
      u8g.drawStr(0, 62, "Back   Ki   +++   ---");
      break;
    case Menu_PID_I_SetUp:
      u8g.drawStr(0, 62, "Back   Kd   +++   ---");
      break;
    case Menu_PID_D_SetUp:
      u8g.drawStr(0, 62, "Back   Ta   +++   ---");
      break;
    case Menu_Logge_SetUp:
      u8g.drawStr(0, 62, "Back      Calib Motor");
      break;
    case Menu_Logge_Adjust_SetUp:
      u8g.drawStr(0, 62, "Back  Save  +++   ---");
      break;
    case Menu_Dynamics_SetUp:
      u8g.drawStr(0, 62, "Back  Save SetUp Comm");
      break;
    case Menu_Dynamics_TMax_SetUp:
      u8g.drawStr(0, 62, "Back  PMin  +++   ---");
      break;
    case Menu_Dynamics_TMin_SetUp:
      u8g.drawStr(0, 62, "Back TSteil +++   ---");
      break;
    case Menu_Dynamics_Steilheit_SetUp:
      u8g.drawStr(0, 62, "Back FDWMul +++   ---");
      break;
    case Menu_Dynamics_STWMult_SetUp:
      u8g.drawStr(0, 62, "Back  PMax  +++   ---");
      break;
    case Menu_Debug_NMEA_SetUp:
      u8g.drawStr(0, 62, "Back  Clear          ");
      break;
    case Menu_OpenCPN_Pilot_OFF:
      u8g.drawStr(0, 62, "Back              OFF");
      break;
    case Menu_OpenCPN_Pilot_ON:
      u8g.drawStr(0, 62, "Back  -=RUNNING=-  ON");
      break;
    default:
      break;
  }
}

//ganzen Bildschirm neu zeichnen (z.B. beim Menuewechsel)
void SetScreen(byte screen) {
  ActualScreen = screen;
  u8g.setFont(u8g_font_6x10r);
  switch (screen) {
    case GUI_Mainscreen: //Mainscreen
      switch (OperationMode) {
        case 0: //Standby
          //u8g.drawLine(64, 0, 64, 64);
          u8g.setFont(u8g_font_helvB14);
          u8g.setPrintPos(OffsetOLED((int)round((float)MagCourse / (float)10), 29, 14), 14);
          u8g.print((int)round((float)MagCourse / float(10)));
          u8g.print(char(176));
          u8g.setFont(u8g_font_helvB12);
          if (MagPilotEnabled == true) {
            u8g.drawStr(25, 29, "MagMode");
          }
          else if (WindPilotEnabled == true) {
            u8g.drawStr(25, 28, "WindMode");
          }
          else if (OpenCPNPilotEnabled == true) {
            u8g.drawStr(25, 28, "OpenCPN");
          }
          else {
            u8g.drawStr(32, 28, "Standby");
          }
          break;
        case 1: //Compass Mode
          u8g.setFont(u8g_font_helvB14);

          u8g.setPrintPos(OffsetOLED((int)round((float)MagCourse / (float)10), 29, 14), 14);
          u8g.print((int)round((float)MagCourse / float(10)));
          u8g.print(char(176));

          //          u8g.setFont(u8g_font_helvB12);
          u8g.setPrintPos(OffsetOLED(MagCourseToSteer, 29, 14), 30);
          u8g.print(MagCourseToSteer);
          u8g.print(char(176));
          //          u8g.drawLine(64, 0, 64, 64);
          break;
        case 2: //Wind Mode
          u8g.setFont(u8g_font_helvB14);

          if (WindSeite == 1) {
            u8g.drawStr(0, 14, "<");
          }
          else {
            u8g.drawStr(115, 14, ">");
          }
          //u8g.drawLine(64, 0, 64, 64);
          u8g.setPrintPos(OffsetOLED((int)WindRichtung, 29, 14), 14);
          u8g.print(WindRichtung);
          u8g.print(char(176));

          //u8g.setFont(u8g_font_helvB12);
          if (WindSeiteToSteer == 1) {
            u8g.drawStr(0, 30, "<");
          }
          else {
            u8g.drawStr(115, 30, ">");
          }
          u8g.setPrintPos(OffsetOLED(abs(WindCourseToSteer), 29, 14), 30);
          u8g.print(abs(WindCourseToSteer));
          u8g.print(char(176));
          break;
        case 3: //OpenCPN
          if ((NMEACPNPilotActive < 5) || (NMEASOG < 0) || (NMEADistanceToWP < 0)) {
            u8g.setFont(u8g_font_helvR08);
            u8g.setPrintPos(0, 12);
            u8g.print("I don't receive any");
            u8g.setPrintPos(0, 26);
            u8g.print("Pilot-Data from OpenCPN");
          }
          else {
            u8g.setFont(u8g_font_helvR08);

            u8g.setPrintPos(0, 8);
            u8g.print("Course: ");
            u8g.print(round((float)NMEACourseActual / (10.0)));
            u8g.setPrintPos(64, 8);
            u8g.print("ToGo: ");
            u8g.print(round((float)NMEACourseToSteer / (10.0)));

            u8g.setPrintPos(0, 18);
            u8g.print("Dist: ");
            u8g.print((float)NMEADistanceToWP / (100.0));
            u8g.setPrintPos(64, 18);
            u8g.print("SOG: ");
            u8g.print((float)NMEASOG / (10.0));

            u8g.setPrintPos(0, 28);
            u8g.setPrintPos(0, 28);
            u8g.print("Next Waypoint: ");
            u8g.print(NMEANextWP);
          }
          break;
        default:
          break;
      }

      u8g.setFont(u8g_font_helvR08);
      u8g.setPrintPos(0, 41);
      u8g.print("STW: ");
      dtostrf((FDW / 100.0), 4, 1, FDWStr);
      u8g.print(FDWStr);
      u8g.print("kn  Wind: ");
      dtostrf((WindGeschwindigkeit / 100.0), 4, 1, WindgeschwindigkeitStr);
      u8g.print(WindgeschwindigkeitStr);
      u8g.print("kn");

      u8g.setPrintPos(OffsetOLED(GeberInPercent, 41, 8), 52);
      u8g.print(GeberInPercent);
      u8g.drawBox(GeberBoxStart, 44, GeberBoxEnd - GeberBoxStart, 8);

      break;
    case GUI_CompassSetUp: //erster Screen in SetUp (Compass Info)
      u8g.setFont(u8g_font_helvB14);
      u8g.drawStr(0, 14, "Offset: ");
      u8g.setPrintPos(OffsetOLED(MagOffset, 60, 14), 14);
      u8g.print(MagOffset);
      u8g.print(char(176));

      u8g.setFont(u8g_font_helvB14);
      u8g.drawStr(0, 40, "Course: ");
      u8g.setPrintPos(OffsetOLED((int)round((float)MagCourse / float(10)), 60, 14), 40);
      u8g.print((int)round((float)MagCourse / float(10)));
      u8g.print(char(176));
      break;
    case GUI_MotorSetUp: //Motormenue
      u8g.setFont(u8g_font_helvB10);
      if (MotorEnabled == false) {
        u8g.drawStr(0, 12, "Motor: Off");
        u8g.drawStr(0, 30, "State: N/A");
        u8g.drawStr(0, 48, "Speed: ");
        u8g.setPrintPos(55, 48);
        u8g.print(MotorSpeed);
      }
      else {
        u8g.drawStr(0, 12, "Motor: ON");
        if (MotorRunning == false) {
          u8g.drawStr(0, 30, "State: Brake");
        }
        else {
          if (MotorDirection == 1) {
            u8g.drawStr(0, 30, "State: Port");
          }
          else {
            u8g.drawStr(0, 30, "State: Starboard");
          }
        }
        u8g.drawStr(0, 48, "Speed: ");
        u8g.setPrintPos(55, 48);
        u8g.print(MotorSpeed);
      }
      break;
    case GUI_433MHzDebug: //433 MHZ Debug
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Dir: ");
      u8g.setPrintPos(30, 8);
      u8g.print(MagCourse);
      u8g.drawStr(60, 8, "STW: ");
      u8g.setPrintPos(95, 8);
      u8g.print(FDW);
      u8g.drawStr(0, 20, "Rud1: ");
      u8g.setPrintPos(30, 20);
      u8g.print(Geber1);
      u8g.drawStr(60, 20, "Rud2: ");
      u8g.setPrintPos(95, 20);
      u8g.print(Geber2);
      u8g.drawStr(0, 32, "WDir: ");
      u8g.setPrintPos(30, 32);
      //      u8g.print(WindRichtung);
      u8g.print(WindSeite * WindRichtung);
      u8g.drawStr(60, 32, "WSpd: ");
      u8g.setPrintPos(95, 32);
      u8g.print(WindGeschwindigkeit);

      u8g.drawStr(60, 44, "Errors: ");
      u8g.setPrintPos(95, 44);
      u8g.print(CRCFailCounter);

      u8g.drawStr(0, 44, "CRC: ");
      u8g.setPrintPos(30, 44);
      if (CRCValid == true) {
        u8g.print("OK");
      }
      else {
        u8g.print("FAIL");
      }
      break;
    case GUI_RudderSetUp: //Ruderlage
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "GebA: ");
      u8g.setPrintPos(32, 8);
      u8g.print(Geber1);
      u8g.drawStr(60, 8, "GebB: ");
      u8g.setPrintPos(92, 8);
      u8g.print(Geber2);
      u8g.drawStr(0, 20, "Difference: ");
      u8g.setPrintPos(55, 20);
      u8g.print(Geber1 - Geber2);
      u8g.drawStr(0, 32, "Middle: ");
      u8g.setPrintPos(55, 32);
      u8g.print(Geber1Middle - Geber2Middle);
      u8g.drawStr(0, 44, "Min: ");
      u8g.setPrintPos(32, 44);
      u8g.print(Geber1Minus - Geber2Minus);
      u8g.drawStr(60, 44, "Max: ");
      u8g.setPrintPos(92, 44);
      u8g.print(Geber1Plus - Geber2Plus);
      break;
    case GUI_RudderZeroSetUp: //Ruder Set Zero Menue
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set Rudder to middle");
      u8g.drawStr(0, 20, "and press \"Save\"");
      u8g.drawStr(0, 32, "Difference: ");
      u8g.setPrintPos(55, 32);
      u8g.print(Geber1 - Geber2);
      break;
    case GUI_RudderLimitsSetUp: //Ruder Set Limits Menue
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set Rudder-Limits");
      u8g.drawStr(0, 20, "Difference: ");
      u8g.setPrintPos(55, 20);
      u8g.print(Geber1 - Geber2);
      u8g.drawStr(0, 32, "Min: ");
      u8g.setPrintPos(55, 32);
      u8g.print(Geber1Minus - Geber2Minus);
      u8g.drawStr(0, 44, "Max: ");
      u8g.setPrintPos(55, 44);
      u8g.print(Geber1Plus - Geber2Plus);
      break;
    case GUI_MotorSpeedLimitsSetUp: //Motor Speed Limit Page
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Actual Limits:");
      u8g.drawStr(0, 20, "Minimum: ");
      u8g.setPrintPos(55, 20);
      u8g.print(MotorMinimalSpeed);
      u8g.drawStr(0, 32, "Maximum: ");
      u8g.setPrintPos(55, 32);
      u8g.print(MotorMaximalSpeed);
      if (MotorMaximalSpeed < MotorMinimalSpeed) {
        u8g.drawStr(0, 50, "Warning: Non-Sense Values");
      }
      break;
    case GUI_MotorSpeedMaxLimitsSetUp: //Motor Max Speed
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set maximal Motor Speed");
      u8g.drawStr(0, 32, "Maximum: ");
      u8g.setPrintPos(55, 32);
      u8g.print(MotorMaximalSpeed);
      break;
    case GUI_MotorSpeedMinLimitsSetUp: //Motor Min Speed
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set minimal Motor Speed");
      u8g.drawStr(0, 32, "Minimum: ");
      u8g.setPrintPos(55, 32);
      u8g.print(MotorMinimalSpeed);
      break;
    case GUI_PIDSetUp: //PID SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "PID Values");
      u8g.drawStr(0, 19, "P");
      u8g.drawStr(10, 19, "(proportional):");
      u8g.setPrintPos(90, 19);
      u8g.print(Kp);
      u8g.drawStr(0, 29, "I");
      u8g.drawStr(10, 29, "(integral):");
      u8g.setPrintPos(90, 29);
      u8g.print(Ki);
      u8g.drawStr(0, 39, "D");
      u8g.drawStr(10, 39, "(differential):");
      u8g.setPrintPos(90, 39);
      u8g.print(Kd);
      u8g.drawStr(0, 49, "T");
      u8g.drawStr(10, 49, "(time contant):");
      u8g.setPrintPos(90, 49);
      u8g.print(Ta);
      break;
    case GUI_PID_P_ValueSetUp: //PID SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set P-Value");
      // u8g.drawLine(64, 0, 64, 64);
      u8g.setPrintPos(53, 34);
      u8g.print(Kp);
      break;
    case GUI_PID_I_ValueSetUp: //PID SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set I-Value");
      u8g.setPrintPos(53, 34);
      u8g.print(Ki);
      break;
    case GUI_PID_D_ValueSetUp: //PID SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set D-Value");
      u8g.setPrintPos(53, 34);
      u8g.print(Kd);
      break;
    case GUI_PID_Ta_ValueSetUp: //PID SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set T-Value");
      u8g.setPrintPos(53, 34);
      u8g.print(Ta);
      break;
    case GUI_FDWSetUp: //FDW Setup
      u8g.setFont(u8g_font_helvB14);
      u8g.drawStr(0, 14, "STW: ");
      u8g.setPrintPos(60, 14);
      dtostrf((FDW / 100.0), 4, 1, FDWStr);
      u8g.print(FDWStr);

      u8g.drawStr(0, 40, "Corr: ");
      u8g.setPrintPos(64, 40);
      u8g.print(FDWCorr);
      break;
    case GUI_DynamicSetUp: //Dynamic Vals SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Dynamic Values");
      u8g.drawStr(0, 19, "PilotTmax:");
      u8g.setPrintPos(85, 19);
      u8g.print(PilotTmax);
      u8g.drawStr(0, 29, "PilotTmin:");
      u8g.setPrintPos(85, 29);
      u8g.print(PilotTmin);
      u8g.drawStr(0, 39, "TSteilheit:");
      u8g.setPrintPos(85, 39);
      u8g.print(TSteilheit);
      u8g.drawStr(0, 49, "FDWMultiplikator:");
      u8g.setPrintPos(85, 49);
      u8g.print(FDWMultiplikator);
      break;
    case GUI_Dynamic_PmaxSetUp: //PMax SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set Pilot T Max-Value");
      // u8g.drawLine(64, 0, 64, 64);
      u8g.setPrintPos(53, 34);
      u8g.print(PilotTmax);
      break;
    case GUI_Dynamic_PminSetUp: //PMin SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set Pilot T Min-Value");
      // u8g.drawLine(64, 0, 64, 64);
      u8g.setPrintPos(53, 34);
      u8g.print(PilotTmin);
      break;
    case GUI_Dynamic_TSteilheitSetUp: //TSteilheit SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set Pilot Steilheit");
      // u8g.drawLine(64, 0, 64, 64);
      u8g.setPrintPos(53, 34);
      u8g.print(TSteilheit);
      break;
    case GUI_Dynamic_FDWMultiSetUp: //FDWMultiplikator SetUp
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Set STW Multiplikator-Value");
      // u8g.drawLine(64, 0, 64, 64);
      u8g.setPrintPos(53, 34);
      u8g.print(FDWMultiplikator);
      break;
    case GUI_433MHzNMEADebug: //433 MHZ Debug NMEA
      u8g.setFont(u8g_font_helvR08);
      u8g.drawStr(0, 8, "Kurs: ");
      u8g.setPrintPos(30, 8);
      u8g.print(NMEACourseActual);
      u8g.drawStr(60, 8, "ToGo: ");
      u8g.setPrintPos(95, 8);
      u8g.print(NMEACourseToSteer);
      u8g.drawStr(0, 20, "Activ: ");
      u8g.setPrintPos(30, 20);
      u8g.print(NMEACPNPilotActive);
      u8g.drawStr(60, 20, "Dist: ");
      u8g.setPrintPos(95, 20);
      u8g.print(NMEADistanceToWP);
      u8g.drawStr(0, 32, "SOG: ");
      u8g.setPrintPos(30, 32);
      //      u8g.print(WindRichtung);
      u8g.print(NMEASOG);
      u8g.drawStr(60, 32, "WP: ");
      u8g.setPrintPos(95, 32);
      u8g.print(NMEANextWP);

      u8g.drawStr(60, 44, "Errors: ");
      u8g.setPrintPos(95, 44);
      u8g.print(CRCFailCounter);

      u8g.drawStr(0, 44, "CRC: ");
      u8g.setPrintPos(30, 44);
      if (CRCValid == true) {
        u8g.print("OK");
      }
      else {
        u8g.print("FAIL");
      }
      break;
    case GUI_Error_NoCalb: //Error, wenn man auf Save ohne Calb gemacht hat
      u8g.drawStr(0, 24, "Error:");
      u8g.drawStr(0, 36, "Do 'Calb' first!");
      break;
    case GUI_Saved_Calb: //Mag Calibration auf EEProm speichern
      u8g.drawStr(0, 36, "Calibration Saved!");
      break;
    case GUI_Saved_MotorLimits_Calb: //Mag Calibration auf EEProm speichern
      u8g.drawStr(0, 36, "Motor Limits Saved!");
      break;
    case GUI_Saved_PID_Params: //PID Parameter auf EEProm speichern
      u8g.drawStr(0, 36, "PID Parameters Saved!");
      break;
    case GUI_Saved_Logge_Calb: //Logge Calib Save
      u8g.drawStr(0, 36, "Log calibration Saved!");
      break;
    case GUI_Error_MotorDisabled: //Error, motor nur an, wenn enabled
      u8g.drawStr(0, 24, "Error:");
      u8g.drawStr(0, 36, "Motor disabled!");
      break;
    case GUI_Saved_Dynamics: //Dynamics Save
      u8g.drawStr(0, 36, "Dynamics Saved!");
      break;
    default:
      break;
  }
}

//offsets, damit die Zehlen immer schÃ¶n ausgerichtet sind ;)
int OffsetOLED(int value, int pos, int fontsize) {
  int off;
  switch (fontsize) {
    case 24:
      off = 18;
      break;
    case 18:
      off = 13;
      break;
    case 14:
      if (value >= 0) {
        off = 10;
      }
      else if (value >= -9) {
        off = 11;
      }
      else if (value >= -99) {
        off = 12;
      }
      else {
        off = 14;
      }
      break;
    case 12:
      off = 9;
      break;
    default:
      off = 6;
      break;
  }
  if ((value > -10000) && (value <= -1000)) {
    return off * 0 + pos;
  }
  else if ((value > -1000) && (value <= -100)) {
    return (off * 1) + pos;
  }
  else if ((value > -100) && (value <= -10)) {
    return (off * 2) + pos;
  }
  else if ((value > -10) && (value < 0)) {
    return (off * 3) + pos;
  }
  else if ((value >= 0) && (value < 10)) {
    return (off * 4) + pos;
  }
  else if ((value >= 10) && (value < 100)) {
    return (off * 3) + pos;
  }
  else if ((value >= 100) && (value < 1000)) {
    return (off * 2) + pos;
  }
  else if ((value >= 1000) && (value < 10000)) {
    return (off * 1) + pos;
  }
}

void DislayShowWelcomeMsg(void) {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_10x20);
    u8g.drawStr(34, 18, "Kutt's");
    u8g.setFont(u8g_font_7x13);
    u8g.drawStr(37, 38, "awesome");
    u8g.setFont(u8g_font_10x20);
    u8g.drawStr(20, 62, "AutoPilot");
  } while ( u8g.nextPage() );
  delay(2000);
}

void DisplayShowScreen(int Menue, int Screen) {
  u8g.firstPage();
  do {
    if (Menue >= 1) {
      SetMenue(Menue);
    }
    if (Screen >= 0) {
      SetScreen(Screen);
    }
  } while ( u8g.nextPage() );
}

void HandleActualScreen0ButtonB(void) {
  if (LastMenue == Menu_Main) {
    OperationMode = 3;
    if (OpenCPNPilotEnabled == false) {
      DisplayShowScreen(28, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_OpenCPN_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 1) {
    //MagMode
    if (MagCourseToSteer > 0) {
      MagCourseToSteer--;
    }
    else {
      MagCourseToSteer = 359;
    }
    if (MagPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 2) {
    //WindMode
    WindCourseToSteer++;
    if (WindCourseToSteer > 180) {
      WindCourseToSteer = -179;
    }
    if (WindCourseToSteer < 0) {
      WindSeiteToSteer = -1;
    }
    else {
      WindSeiteToSteer = 1;
    }
    if (WindPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
}

void HandleActualScreen1ButtonB(void) {
  if (MagCalbDone == true) {
#if DEBUG
    Serial.println("Save 2 EEPROM");
#endif
    MagCalbDone = false;
    WriteEEPROM();
    ActualScreenPrev = ActualScreen;
    //Calibration saved...
    DisplayShowScreen(Menu_None, GUI_Saved_Calb); //Menue, Screen (-1  for nothing)
    delay(1000);
    //vorherigen Screen wieder zeigen
    DisplayShowScreen(Menu_Compass_Main_SetUp, ActualScreenPrev); //Menue, Screen (-1  for nothing)
  } else {

    if (MagInCalc == false) {
      ActualScreenPrev = ActualScreen;
      //Error - do calb first!
      DisplayShowScreen(Menu_None, GUI_Error_NoCalb); //Menue, Screen (-1  for nothing)
      delay(1000);
      //vorherigen Screen wieder zeigen
      DisplayShowScreen(Menu_Compass_Main_SetUp, ActualScreenPrev); //Menue, Screen (-1  for nothing)
    }
    else {
    }
  }
}

void HandleActualScreen2ButtonB(void) {
  if (MotorActualSetupScreen == 0) {
    if (ActualScreen == GUI_MotorSetUp) {
      DisplayShowScreen(Menu_Motor_Limits_SetUp, GUI_MotorSpeedLimitsSetUp); //Menue, Screen (-1  for nothing)
    }
    //Motorparameter Menue Öffnen (PID?!)
  }
  else if (MotorActualSetupScreen == 1) {
    MotorActualSetupScreen = 2;
    //Testmenue - State einstellen
    DisplayShowScreen(Menu_Motor_Main_Test_State_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
  else if (MotorActualSetupScreen == 2) {
    MotorEnabled = !MotorEnabled;
    MotorSet(MotorEnabled, false, -1); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
    //    MotorRunning = false;
  }
}

void HandleActualScreen0ButtonC(void) {
  if (LastMenue == Menu_Main) {
    OperationMode = 2;
    if (WindPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 1) {
    //MagMode
    if (MagCourseToSteer < 359) {
      MagCourseToSteer++;
    }
    else {
      MagCourseToSteer = 0;
    }
    if (MagPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 2) {
    //WindMode
    WindCourseToSteer--;
    if (WindCourseToSteer < -180) {
      WindCourseToSteer = 179;
    }
    if (WindCourseToSteer < 0) {
      WindSeiteToSteer = -1;
    }
    else {
      WindSeiteToSteer = 1;
    }
    if (WindPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
}

void HandleActualScreen1ButtonC(void) {
  if (MagInCalc == false) {
    MagInCalc = true;
    MagCalbDone = false;
    //Kalibriermenue
    DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
  }
  else {
    //+++
    if (MagOffset < 180) {
      MagOffset++;
    }
    DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleActualScreen2ButtonC(void) {
  if (MotorActualSetupScreen == 0) {
    MotorActualSetupScreen = 1;
    //Motor Testmenue!
    DisplayShowScreen((Menu_Motor_Main_Test_Run_SetUp + MotorRunning), GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
  else if (MotorActualSetupScreen == 1) {
    MotorActualSetupScreen = 3; //Speedeinstellung
    DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
  else if (MotorActualSetupScreen == 2) {
    //Port
    MotorRunning = !MotorRunning;
    MotorSet(MotorEnabled, MotorRunning, 1); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
  }
  else if (MotorActualSetupScreen == 3) {
    //+++
    if (MotorSpeed < 255) {
      MotorSpeed++;
    }
    MotorSet(MotorEnabled, MotorRunning, MotorDirection); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
    DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleActualScreen8ButtonC(void) {
  if (MotorMaximalSpeed < 255) {
    MotorMaximalSpeed++;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen9ButtonC(void) {
  if (MotorMinimalSpeed < 255) {
    MotorMinimalSpeed++;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen11ButtonC(void) {
  if (Kp < 10) {
    Kp += 0.01;
  }
  DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen12ButtonC(void) {
  if (Ki < 10) {
    Ki += 0.01;
  }
  DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen13ButtonC(void) {
  if (Kd < 10) {
    Kd += 0.01;
  }
  DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen14ButtonC(void) {
  if (Ta < 10) {
    Ta += 0.01;
  }
  DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen15ButtonC(void) {
  if (FDWCorr < 10) {
    FDWCorr += 0.01;
  }
  DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen17ButtonC(void) {
  if (PilotTmax < 100) {
    PilotTmax += 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen18ButtonC(void) {
  if (PilotTmin < 100) {
    PilotTmin += 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen19ButtonC(void) {
  if (TSteilheit < 100) {
    TSteilheit += 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen20ButtonC(void) {
  if (FDWMultiplikator < 100) {
    FDWMultiplikator += 0.01;
  }
  DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen0ButtonD(void) {
  if (OperationMode == 1) {
    //MagPilot
    SwitchEnabledPilot(1);
    if (MagPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      WriteEEPROMMag();
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 2) {
    //WindPilot
    SwitchEnabledPilot(2);
    if (WindPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      WriteEEPROMWind();
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (OperationMode == 3) {
    //OpenCPN
    SwitchEnabledPilot(3);
    if (OpenCPNPilotEnabled == false) {
      DisplayShowScreen(28, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_OpenCPN_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
  else if (LastMenue == Menu_Main) {
    OperationMode = 1;
    if (MagPilotEnabled == false) {
      DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
    else {
      DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
    }
  }
}

void HandleActualScreen1ButtonD(void) {
  if (MagInCalc == true) {
    if (MagOffset > -180) {
      MagOffset--;
    }
    DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
  }
  else {
    //Motormenue
    MagCalbDone = false;
    MotorActualSetupScreen = 0;
    DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleActualScreen2ButtonD(void) {
  if (MotorActualSetupScreen == 0) {
    if (ActualScreen == GUI_MotorSetUp) {
      //Ruderlage Menue
      DisplayShowScreen(Menu_Rudder_Main_SetUp, GUI_RudderSetUp); //Menue, Screen (-1  for nothing)
    }
    else if (ActualScreen == GUI_433MHzDebug) {
      //433MHZ Menue
      DisplayShowScreen(Menu_Debug_SetUp, GUI_433MHzDebug); //Menue, Screen (-1  for nothing)
    }
  }
  else if (MotorActualSetupScreen == 1) {
    if (MotorEnabled == true) {
      MotorRunning = !MotorRunning;
      DisplayShowScreen((Menu_Motor_Main_Test_Run_SetUp + MotorRunning), GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
      MotorSet(MotorEnabled, MotorRunning, MotorDirection); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
      //Motor Starten!!! evtl. MotorRunning und den ganzen Rotz mit in die Subfunktion
    }
    else {
      ActualScreenPrev = ActualScreen;
      DisplayShowScreen(Menu_None, GUI_Error_MotorDisabled); //Menue, Screen (-1  for nothing)
      delay(1000);
      //vorherigen Screen wieder zeigen
      DisplayShowScreen(Menu_Motor_Main_Test_Run_SetUp, ActualScreenPrev); //Menue, Screen (-1  for nothing)
    }
  }
  else if (MotorActualSetupScreen == 2) {
    //Steuerbord
    MotorRunning = !MotorRunning;
    MotorSet(MotorEnabled, MotorRunning, -1); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
  }
  else if (MotorActualSetupScreen == 3) {
    //--
    if (MotorSpeed > 0) {
      MotorSpeed--;
    }
    MotorSet(MotorEnabled, MotorRunning, MotorDirection); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
    DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleActualScreen8ButtonD(void) {
  if (MotorMaximalSpeed > 0) {
    MotorMaximalSpeed--;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen9ButtonD(void) {
  if (MotorMinimalSpeed > 0) {
    MotorMinimalSpeed--;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen11ButtonD(void) {
  if (Kp > 0) {
    Kp -= 0.01;
  }
  DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen12ButtonD(void) {
  if (Ki > 0) {
    Ki -= 0.01;
  }
  DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen13ButtonD(void) {
  if (Kd > 0) {
    Kd -= 0.01;
  }
  DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen14ButtonD(void) {
  if (Ta > 0) {
    Ta -= 0.01;
  }
  DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen15ButtonD(void) {
  if (FDWCorr > 0) {
    FDWCorr -= 0.01;
  }
  DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen17ButtonD(void) {
  if (PilotTmax > 0) {
    PilotTmax -= 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen18ButtonD(void) {
  if (PilotTmin > 0) {
    PilotTmin -= 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen19ButtonD(void) {
  if (TSteilheit > 0) {
    TSteilheit -= 0.1;
  }
  DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp); //Menue, Screen (-1  for nothing)
}

void HandleActualScreen20ButtonD(void) {
  if (FDWMultiplikator > 0) {
    FDWMultiplikator -= 0.01;
  }
  DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressOperationMode1ButtonB(void) {
  if (MagCourseToSteer > 5) {
    MagCourseToSteer -= 5;
  }
  else {
    MagCourseToSteer = (MagCourseToSteer - 5) + 360;
  }
  if (MagPilotEnabled == false) {
    DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
  else {
    DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressOperationMode2ButtonB(void) {
  if (WindCourseToSteer < 175) {
    WindCourseToSteer += 5;
  }
  else {
    WindCourseToSteer = (WindCourseToSteer + 5) - 360;
  }
  if (WindCourseToSteer < 0) {
    WindSeiteToSteer = -1;
  }
  else {
    WindSeiteToSteer = 1;
  }
  if (WindPilotEnabled == false) {
    DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
  else {
    DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressOperationMode1ButtonC(void) {
  if (MagCourseToSteer < 355) {
    MagCourseToSteer += 5;
  }
  else {
    MagCourseToSteer = (MagCourseToSteer + 5) - 360;
  }
  if (MagPilotEnabled == false) {
    DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
  else {
    DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressOperationMode2ButtonC(void) {
  if (WindCourseToSteer > -175) {
    WindCourseToSteer -= 5;
  }
  else {
    WindCourseToSteer = (WindCourseToSteer - 5) + 360;
  }
  if (WindCourseToSteer < 0) {
    WindSeiteToSteer = -1;
  }
  else {
    WindSeiteToSteer = 1;
  }
  if (WindPilotEnabled == false) {
    DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
  else {
    DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressActualScreen1ButtonC(void) {
  if (MagOffset < 175) {
    MagOffset += 5;
  }
  else {
    MagOffset = 180;
  }
  DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen2ButtonC(void) {
  if (MotorActualSetupScreen == 3) {
    if (MotorSpeed < 250) {
      MotorSpeed += 5;
    }
    else {
      MotorSpeed = 255;
    }
    MotorSet(MotorEnabled, MotorRunning, MotorDirection); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
    DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressActualScreen8ButtonC(void) {
  if (MotorMaximalSpeed < 250) {
    MotorMaximalSpeed += 5;
  }
  else {
    MotorMaximalSpeed = 255;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp);
}

void HandleLongPressActualScreen9ButtonC(void) {
  if (MotorMinimalSpeed < 250) {
    MotorMinimalSpeed += 5;
  }
  else {
    MotorMinimalSpeed = 255;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp);
}

void HandleLongPressActualScreen11ButtonC(void) {
  if (Kp <= 9.95) {
    Kp += 0.05;
  }
  else {
    Kp = 10;
  }
  DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen12ButtonC(void) {
  if (Ki <= 9.95) {
    Ki += 0.05;
  }
  else {
    Ki = 10;
  }
  DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen13ButtonC(void) {
  if (Kd <= 9.95) {
    Kd += 0.05;
  }
  else {
    Kd = 10;
  }
  DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen14ButtonC(void) {
  if (Ta <= 9.95) {
    Ta += 0.05;
  }
  else {
    Ta = 10;
  }
  DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen15ButtonC(void) {
  if (FDWCorr <= 99.95) {
    FDWCorr += 0.05;
  }
  else {
    FDWCorr = 100;
  }
  DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen17ButtonC(void) {
  if (PilotTmax <= 99.5) {
    PilotTmax += 0.5;
  }
  else {
    PilotTmax = 100;
  }
  DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen18ButtonC(void) {
  if (PilotTmin <= 99.5) {
    PilotTmin += 0.5;
  }
  else {
    PilotTmin = 100;
  }
  DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen19ButtonC(void) {
  if (TSteilheit <= 99.5) {
    TSteilheit += 0.5;
  }
  else {
    TSteilheit = 100;
  }
  DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen20ButtonC(void) {
  if (FDWMultiplikator <= 99.95) {
    FDWMultiplikator += 0.05;
  }
  else {
    FDWMultiplikator = 100;
  }
  DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen1ButtonD(void) {
  if (MagOffset > -175) {
    MagOffset -= 5;
  }
  else {
    MagOffset = -180;
  }
  DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen2ButtonD(void) {
  if (MotorActualSetupScreen == 3) {
    if (MotorSpeed > 5) {
      MotorSpeed -= 5;
    }
    else {
      MotorSpeed = 0;
    }
    MotorSet(MotorEnabled, MotorRunning, MotorDirection); //MotorSet(Enable/Disable, Running, Left/Right, Speed[0-255])
    DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
  }
}

void HandleLongPressActualScreen8ButtonD(void) {
  if (MotorMaximalSpeed > 5) {
    MotorMaximalSpeed -= 5;
  }
  else {
    MotorMaximalSpeed = 0;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp);
}

void HandleLongPressActualScreen9ButtonD(void) {
  if (MotorMinimalSpeed > 5) {
    MotorMinimalSpeed -= 5;
  }
  else {
    MotorMinimalSpeed = 0;
  }
  DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp);
}

void HandleLongPressActualScreen11ButtonD(void) {
  if (Kp >= 0.05) {
    Kp -= 0.05;
  }
  else {
    Kp = 0;
  }
  DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen12ButtonD(void) {
  if (Ki >= 0.05) {
    Ki -= 0.05;
  }
  else {
    Ki = 0;
  }
  DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen13ButtonD(void) {
  if (Kd >= 0.05) {
    Kd -= 0.05;
  }
  else {
    Kd = 0;
  }
  DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen14ButtonD(void) {
  if (Ta >= 0.05) {
    Ta -= 0.05;
  }
  else {
    Ta = 0;
  }
  DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen15ButtonD(void) {
  if (FDWCorr >= 0.05) {
    FDWCorr -= 0.05;
  }
  else {
    FDWCorr = 0;
  }
  DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen17ButtonD(void) {
  if (PilotTmax >= 0.5) {
    PilotTmax -= 0.5;
  }
  else {
    PilotTmax = 0;
  }
  DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen18ButtonD(void) {
  if (PilotTmin >= 0.5) {
    PilotTmin -= 0.5;
  }
  else {
    PilotTmin = 0;
  }
  DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen19ButtonD(void) {
  if (TSteilheit >= 0.5) {
    TSteilheit -= 0.5;
  }
  else {
    TSteilheit = 0;
  }
  DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp); //Menue, Screen (-1  for nothing)
}

void HandleLongPressActualScreen20ButtonD(void) {
  if (FDWMultiplikator >= 0.05) {
    FDWMultiplikator -= 0.05;
  }
  else {
    FDWMultiplikator = 0;
  }
  DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp); //Menue, Screen (-1  for nothing)
}

void EntprellAndRead(void) {
  if ((TasteReadA != TasteLastReadA) || (TasteReadB != TasteLastReadB) || (TasteReadC != TasteLastReadC) || (TasteReadD != TasteLastReadD)) {
    TasteLastDebounceTime = millis();
    if (LangDruckRunner >= LangDruck) {
      TasteStateA = TasteReadA;
      TasteStateB = TasteReadB;
      TasteStateC = TasteReadC;
      TasteStateD = TasteReadD;
    }
    LangDruckRunner = 0;
    //letzten Zustand speichern
    TasteLastReadA = TasteReadA;
    TasteLastReadB = TasteReadB;
    TasteLastReadC = TasteReadC;
    TasteLastReadD = TasteReadD;
    TasteLangDruckStateA = false;
    TasteLangDruckStateB = false;
    TasteLangDruckStateC = false;
    TasteLangDruckStateD = false;
  }
}

void TransponderInit(void) {
  ELECHOUSE_cc1101.Init(F_433);  // set frequency - F_433, F_868, F_965 MHz
  ELECHOUSE_cc1101.SetReceive();
  TransponderDataWaiting = false;

}

//alle 8 ms!
void TransponderReadData(void) {
  if (TransponderDataWaiting == true)
  {
    int len = ELECHOUSE_cc1101.ReceiveData(TransponderBuffer);

    TransponderBuffer[len] = '\0';
    //		Serial.println((char *)TransponderBuffer);

    Token = 0;
    temppch = strtok (TransponderBuffer, "~");
    while (temppch != NULL)
    {
      switch (Token) {
        case 0:
          MagCourseTrans = temppch.toInt();
          break;
        case 1:
          Geber1Trans = temppch.toInt();
          break;
        case 2:
          Geber2Trans = temppch.toInt();
          break;
        case 3:
          WindGeschwindigkeitTrans = temppch.toInt();
          break;
        case 4:
          WindRichtungTrans = temppch.toInt();
          break;
        case 5:
          FDWTrans = temppch.toInt();
          break;
        case 6:
          NMEACPNPilotActiveTrans = temppch.toInt();
          break;
        case 7:
          NMEACourseActualTrans = temppch.toInt();
          break;
        case 8:
          NMEACourseToSteerTrans = temppch.toInt();
          break;
        case 9:
          NMEADistanceToWPTrans = temppch.toInt();
          break;
        case 10:
          NMEANextWPTrans = temppch.toInt();
          break;
        case 11:
          NMEASOGTrans = temppch.toInt();
          break;
        case 12:
          CRCMicro = temppch.toInt();
          break;
        default:
          break;
      }
      Token++;
      temppch = strtok (NULL, "~");
    }
    CRCValid = CheckCRC();
    if (CRCValid == true) {

      //			GotDataBlinker = true;
      digitalWrite(LED_BUILTIN, false);

      if (NMEACPNPilotActive > 0) {
        NMEACPNPilotActive = NMEACPNPilotActiveTrans;
        NMEACourseActual = NMEACourseActualTrans;
        NMEACourseToSteer = NMEACourseToSteerTrans;
        NMEADistanceToWP = NMEADistanceToWPTrans;
        NMEANextWP = NMEANextWPTrans;
        NMEASOG = NMEASOGTrans;
      }
      else {
        NMEACPNPilotActive = NMEACPNPilotActiveTrans;
        NMEACourseActual = NMEACourseActualTrans;
        NMEACourseToSteer = -1;
        NMEADistanceToWP = -1;
        NMEANextWP = -1;
        NMEASOG = NMEASOGTrans;
      }

      MagCourse = MagCourseTrans;

#if GeberInvert
      Geber1 = Geber2Trans;
      Geber2 = Geber1Trans;
#else
      Geber1 = Geber1Trans;
      Geber2 = Geber2Trans;
#endif

      WindGeschwindigkeit = WindGeschwindigkeitTrans;
      WindRichtung = WindRichtungTrans;
      FDW = FDWTrans;

      FDW = FDWCorr * (float)FDW;

      MagCourse = MagCourse - MagOffset * 10;
      if (MagCourse > 3600) {
        MagCourse = MagCourse - 3600;
      }
      else if (MagCourse < 0) {
        MagCourse = MagCourse + 3600;
      }

      if (WindRichtung < 0) {
        WindSeite = -1;
      }
      else {
        WindSeite = 1;
      }
      WindRichtung = abs(WindRichtung);
    } else {
      CRCFailCounter++;
    }

    TransponderDataWaiting = false;
    ELECHOUSE_cc1101.SetReceive();
  }
}

//call von TransponderReadData() alle 8ms
bool CheckCRC(void) {

  byte MagCourseStr[8];
  byte Geber1Str[8];
  byte Geber2Str[8];
  byte WindGeschwindigkeitStr[8];
  byte WindRichtungStr[8];
  byte FDWStr[8];
  byte NMEACourseActualChar[8];
  byte NMEACourseToSteerChar[8];
  byte NMEACPNPilotActiveChar[3];
  byte NMEADistanceToWPChar[8];
  byte NMEASOGChar[8];
  byte NMEANextWPChar[8];

  dtostrf(MagCourseTrans, LEnOfInt(MagCourseTrans), 0, MagCourseStr);
  dtostrf(Geber1Trans, LEnOfInt(Geber1Trans), 0, Geber1Str);
  dtostrf(Geber2Trans, LEnOfInt(Geber2Trans), 0, Geber2Str);
  dtostrf(WindGeschwindigkeitTrans, LEnOfInt(WindGeschwindigkeitTrans), 0, WindGeschwindigkeitStr);
  dtostrf(WindRichtungTrans, LEnOfInt(WindRichtungTrans), 0, WindRichtungStr);
  dtostrf(FDWTrans, LEnOfInt(FDWTrans), 0, FDWStr);

  dtostrf(NMEACourseActualTrans, LEnOfInt(NMEACourseActualTrans), 0, NMEACourseActualChar);
  dtostrf(NMEACourseToSteerTrans, LEnOfInt(NMEACourseToSteerTrans), 0, NMEACourseToSteerChar);
  dtostrf(NMEACPNPilotActiveTrans, LEnOfInt(NMEACPNPilotActiveTrans), 0, NMEACPNPilotActiveChar);
  dtostrf(NMEADistanceToWPTrans, LEnOfInt(NMEADistanceToWPTrans), 0, NMEADistanceToWPChar);
  dtostrf(NMEASOGTrans, LEnOfInt(NMEASOGTrans), 0, NMEASOGChar);
  dtostrf(NMEANextWPTrans, LEnOfInt(NMEANextWPTrans), 0, NMEANextWPChar);


  memset(buffer, 0, sizeof(buffer));
  sprintf(buffer, "%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s", MagCourseStr, Geber1Str, Geber2Str, WindGeschwindigkeitStr, WindRichtungStr, FDWStr, NMEACPNPilotActiveChar, NMEACourseActualChar, NMEACourseToSteerChar, NMEADistanceToWPChar, NMEANextWPChar, NMEASOGChar);
  CRCMega = CRC16.ccitt(buffer, sizeof(buffer));

  if (CRCMega == CRCMicro) {
    return true;
  } else {
    return false;
  }
}

byte LEnOfInt(int var) {
  byte ret;
  if (var > 9999) {
    ret = 5;
  }
  else if ((var > 999) && (var < 10000)) {
    ret = 4;
  }
  else if ((var > 99) && (var < 1000)) {
    ret = 3;
  }
  else if ((var > 9) && (var < 100)) {
    ret = 2;
  }
  else if ((var > -1) && (var < 10)) {
    ret = 1;
  }
  else if ((var > -10) && (var < 0)) {
    ret = 2;
  }
  else if ((var > -100) && (var < -9)) {
    ret = 3;
  }
  else if ((var > -1000) && (var < -99)) {
    ret = 4;
  }
  else if ((var > -10000) && (var < -999)) {
    ret = 5;
  }
  else if ((var > -32768) && (var < -9999)) {
    ret = 6;
  }
  return ret;
}

//alle 8ms Ruderlage lesen
void CalcRudder(void) {

  //Berechne Ruderlage in % vom Maximal zulässigen Ausschlag
  if ((Geber1 - Geber2) >= (Geber1Middle - Geber2Middle)) {
    GeberInPercent = round((99.0 / (float)((Geber1Plus - Geber2Plus) - (Geber1Middle - Geber2Middle))) * (float)((Geber1 - Geber2) - (Geber1Middle - Geber2Middle)));
    GeberRichtung = -1;
  }
  else {
    GeberInPercent = round((99.0 / (float)((Geber1Minus - Geber2Minus) - (Geber1Middle - Geber2Middle))) * (float)((Geber1 - Geber2) - (Geber1Middle - Geber2Middle)));
    GeberRichtung = 1;
  }

  //wenn Ruder über den maximalen Stellbereich des AP gedreht wird
  if (GeberInPercent > 99) {
    GeberInPercent = 99;
    //ich denke mal das ist sinnlos!
    //GeberBoxEnd = round((float)(57 * GeberInPercent) / 99.0);
  }
  else if (GeberInPercent < 0) {
    GeberInPercent = 0;
  }

  //größe des Balkens berechnen
  if (GeberRichtung == 1) {
    GeberBoxStart = round((-57.0 / 99.0) * (float)GeberInPercent + 57.0);
    GeberBoxEnd = 57;
  }
  else {
    GeberBoxStart = 72;
    GeberBoxEnd = round((float)(57 * GeberInPercent) / 99.0) + 72;
  }
}

void SwitchEnabledPilot(byte was) {
  MotorSet(true, false, -1);

  PIDOutAbweichAlt = AverageCourseAbweichung;
  PIDOutDrehSum = 0;
  PIDOutDrehAlt = AverageDrehrate;
  PIDOutAbweichSum = 0;

  PilotNextCheck = millis();

  WaitUntiStart = 0;

  switch (was) {
      break;
    case 1:
      MagPilotEnabled = !MagPilotEnabled;
      WindPilotEnabled = false;
      OpenCPNPilotEnabled = false;
      break;
    case 2:
      MagPilotEnabled = false;
      WindPilotEnabled = !WindPilotEnabled;
      OpenCPNPilotEnabled = false;
      break;
    case 3:
      MagPilotEnabled = false;
      WindPilotEnabled = false;
      OpenCPNPilotEnabled = !OpenCPNPilotEnabled;
      break;
    case 0:
    default:
      MagPilotEnabled = false;
      WindPilotEnabled = false;
      OpenCPNPilotEnabled = false;
      break;
  }
}

void DoPilot(void) {
  if (MagPilotEnabled == true) {
    CalcTurnrateAndOffcourseMag();
  }
  else if (WindPilotEnabled == true) {
    CalcTurnrateAndOffcourseWind();
  }
  else if (OpenCPNPilotEnabled == true) {
    if ((NMEACPNPilotActive > 5) && (NMEADistanceToWP > 0) && (NMEACourseActual >= 0)) {
      CalcTurnrateAndOffcourseOpenCPN();
    }
  }


  if ((MagPilotEnabled == true) || (WindPilotEnabled == true) || ((OpenCPNPilotEnabled == true) && ((NMEACPNPilotActive > 5) && (NMEADistanceToWP > 0) && (NMEACourseActual >= 0)))) {

    CalcPID();
    AveragePID();

    if ((PilotNextCheck <= millis()) && (WaitUntiStart > (numReadingsDrehrate + numReadingsCourseAbweichung + numReadingsRuderLage))) {

      MotorSetPoint = abs(PIDRuderlage);

#if SetPointInvert
      if (PIDRuderlage > 0) {
        MotorSetPointDirection = 1;
      }
      else {
        MotorSetPointDirection = -1;
      }
#else
      if (PIDRuderlage > 0) {
        MotorSetPointDirection = -1;
      }
      else {
        MotorSetPointDirection = 1;
      }
#endif

#if DEBUG
      Serial.print("AverageCourseAbweichung: ");
      Serial.print(AverageCourseAbweichung / 10.0);

      Serial.print(" PIDOutAbweich: ");
      Serial.print(PIDOutAbweich);

      Serial.print(" AverageDrehrate: ");
      Serial.print(AverageDrehrate);

      Serial.print(" PIDOutDreh: ");
      Serial.print(PIDOutDreh);

      Serial.print(" SollRuderlage: ");
      Serial.print(PIDRuderlage);

      Serial.print(" IstRuderlage: ");
      Serial.print(GeberInPercent * GeberRichtung);

      Serial.print(" Time was: ");
      Serial.println(DrehRuder);
#endif

      MotorDriveToAngle();
    }
    else if (WaitUntiStart <= (numReadingsDrehrate + numReadingsCourseAbweichung + numReadingsRuderLage)) {
      WaitUntiStart++;
    }
    CalcNextCheck();
  }
}

void MotorDriveToAngle(void) {
  if (MotorSetPoint > 99) {
    MotorSetPoint = 99;
  }

  MotorSpeed = abs(GeberInPercent * GeberRichtung - MotorSetPoint * MotorSetPointDirection) *  ((MotorMaximalSpeed - MotorMinimalSpeed) / 99) + MotorMinimalSpeed;

  if (MotorSpeed > MotorMaximalSpeed) {
    MotorSpeed = MotorMaximalSpeed;
  }
  else if (MotorSpeed < MotorMinimalSpeed) {
    MotorSpeed = MotorMinimalSpeed;
  }

  if (MotorRunning == true) {
    analogWrite(MotorPWM, MotorSpeed);
  }
  else if (abs(GeberInPercent - MotorSetPoint) > 0) {
    if ((GeberInPercent > MotorSetPoint) && (GeberRichtung == 1)) {
      MotorSet(true, true, 1);
    }
    else if ((GeberInPercent > MotorSetPoint) && (GeberRichtung == -1)) {
      MotorSet(true, true, -1);
    }
    else if ((GeberInPercent < MotorSetPoint)) {
      MotorSet(true, true, MotorSetPointDirection * (-1));
    }
  }
}

void CheckMotor(void) {
  if ((MagPilotEnabled == true) || (WindPilotEnabled == true) || (OpenCPNPilotEnabled == true)) {
    if ((MotorEnabled == true) && (MotorRunning == true)) {
      MotorStop = false;
#if TargetInvert
      if ((MotorDirection == -1)  && (MotorSetPointDirection == 1) && (GeberRichtung == 1) && (GeberInPercent <= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == -1)  && (MotorSetPointDirection == 1) && (GeberRichtung == -1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == -1)  && (MotorSetPointDirection == -1) && (GeberRichtung == -1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == 1)  && (MotorSetPointDirection == 1) && (GeberRichtung == 1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == 1)  && (MotorSetPointDirection == -1) && (GeberRichtung == 1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == 1)  && (MotorSetPointDirection == -1) && (GeberRichtung == -1) && (GeberInPercent <= MotorSetPoint)) {
        MotorStop = true;
      }
#else
      if ((MotorDirection == 1)  && (MotorSetPointDirection == 1) && (GeberRichtung == 1) && (GeberInPercent <= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == 1)  && (MotorSetPointDirection == 1) && (GeberRichtung == -1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == 1)  && (MotorSetPointDirection == -1) && (GeberRichtung == -1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == -1)  && (MotorSetPointDirection == 1) && (GeberRichtung == 1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == -1)  && (MotorSetPointDirection == -1) && (GeberRichtung == 1) && (GeberInPercent >= MotorSetPoint)) {
        MotorStop = true;
      }
      else if ((MotorDirection == -1)  && (MotorSetPointDirection == -1) && (GeberRichtung == -1) && (GeberInPercent <= MotorSetPoint)) {
        MotorStop = true;
      }
#endif
      if (MotorStop == true) {
        MotorSet(true, false, MotorDirection);
      }
    }
  }
}

//alle 8ms!
void MotorCheckIfLimit(void) {
//Limitprüfung!
  if ((MotorEnabled == true) && (MotorRunning == true)) {
#if LimitInvert
    if ((MotorDirection == -1) && ((Geber1 - Geber2) >= (Geber1Plus - Geber2Plus))) {
      #if DEBUG
        Serial.println("AUS!: true, true - PlusLimit");
      #endif
      MotorSet(true, false, MotorDirection);
    }
    else if ((MotorDirection == 1) && ((Geber1 - Geber2) <= (Geber1Minus - Geber2Minus))) {      
      #if DEBUG
        Serial.println("AUS!: true, false - MinusLimit");
      #endif
      MotorSet(true, false, MotorDirection);
    }
  }
#else
    if ((MotorDirection == 1) && ((Geber1 - Geber2) >= (Geber1Plus - Geber2Plus))) {
      #if DEBUG
        Serial.println("AUS!: true, true - PlusLimit");
      #endif
      MotorSet(true, false, MotorDirection);
    }
    else if ((MotorDirection == -1) && ((Geber1 - Geber2) <= (Geber1Minus - Geber2Minus))) {      
      #if DEBUG
        Serial.println("AUS!: true, false - MinusLimit");
      #endif
      MotorSet(true, false, MotorDirection);
    }
  }
#endif
}

void MotorSet(bool Enable, bool Running, int Direction) {
#if MotorInvert
  MotorDirection = Direction * (-1);
#else
  MotorDirection = Direction;
#endif
  if (Enable == false) {
    digitalWrite(MotorPWM, false);
    digitalWrite(MotorBitA, false);
    digitalWrite(MotorBitB, false);

    MotorEnabled = false;
    MotorRunning = false;
  }
  else {
    MotorEnabled = true;
    if (Running == false) {
      digitalWrite(MotorPWM, true );
      digitalWrite(MotorBitA, false);
      digitalWrite(MotorBitB, false);

      MotorRunning = false;
    } else {
      analogWrite(MotorPWM, MotorSpeed);
      if ((Direction == 1) && ((Geber1 - Geber2) < (Geber1Plus - Geber2Plus))) {
#if MotorInvert
          digitalWrite(MotorBitA, false);
          digitalWrite(MotorBitB, true);
#else
          digitalWrite(MotorBitA, true);
          digitalWrite(MotorBitB, false);
#endif        
        MotorRunning = true;
      }
      else if ((Direction == -1) && ((Geber1 - Geber2) > (Geber1Minus - Geber2Minus))) {
#if MotorInvert
          digitalWrite(MotorBitA, true);
          digitalWrite(MotorBitB, false);
#else
          digitalWrite(MotorBitA, false);
          digitalWrite(MotorBitB, true);
#endif
        MotorRunning = true;
      }
    }
  }
}

void CalcTurnrateAndOffcourseMag(void) {

  //Timer alle 100ms, deshalb
  //Drehrate * 10 entfällt, da Kurs schon * 10 kommt (wg Kommastelle)


  TotalDrehrateEimer = TotalDrehrateEimer - DrehrateEimer[readIndexDrehrate];

  if ((MagCourseLast > 2700) && (MagCourse < 900)) {
    Drehrate = (3600 - MagCourseLast) + MagCourse;
  }
  else if ((MagCourseLast < 900) && (MagCourse > 2700)) {
    Drehrate = (MagCourse - 3600) - MagCourseLast;
  }
  else {
    Drehrate = MagCourse - MagCourseLast;
  }

  DrehrateEimer[readIndexDrehrate] = Drehrate;

  TotalDrehrateEimer = TotalDrehrateEimer + DrehrateEimer[readIndexDrehrate];
  AverageDrehrate = (float)TotalDrehrateEimer / (float)numReadingsDrehrate;

  readIndexDrehrate = readIndexDrehrate + 1;
  if (readIndexDrehrate >= numReadingsDrehrate) {
    readIndexDrehrate = 0;
  }

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer - CourseAbweichungEimer[readIndexCourseAbweichung];

  if ((MagCourseToSteer * 10 > 2700) && (MagCourse < 900)) {
    CourseAbweichung = (3600 - MagCourseToSteer * 10) + MagCourse;
  }
  else if ((MagCourseToSteer * 10 < 900) && (MagCourse > 2700)) {
    CourseAbweichung = (MagCourse - 3600) - MagCourseToSteer * 10;
  }
  else {
    CourseAbweichung = MagCourse - MagCourseToSteer * 10;
  }

  CourseAbweichungEimer[readIndexCourseAbweichung] = CourseAbweichung;

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer + CourseAbweichungEimer[readIndexCourseAbweichung];
  AverageCourseAbweichung = (float)TotalCourseAbweichungEimer / (float)numReadingsCourseAbweichung;

  readIndexCourseAbweichung = readIndexCourseAbweichung + 1;
  if (readIndexCourseAbweichung >= numReadingsCourseAbweichung) {
    readIndexCourseAbweichung = 0;
  }

  MagCourseLast = MagCourse;
}

void CalcTurnrateAndOffcourseWind(void) {

  TotalDrehrateEimer = TotalDrehrateEimer - DrehrateEimer[readIndexDrehrate];

  Drehrate = WindRichtung * WindSeite - WindRichtungLast * WindSeiteLast;

  DrehrateEimer[readIndexDrehrate] = Drehrate;

  TotalDrehrateEimer = TotalDrehrateEimer + DrehrateEimer[readIndexDrehrate];
  AverageDrehrate = (float)TotalDrehrateEimer / (float)numReadingsDrehrate;

  readIndexDrehrate = readIndexDrehrate + 1;
  if (readIndexDrehrate >= numReadingsDrehrate) {
    readIndexDrehrate = 0;
  }

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer - CourseAbweichungEimer[readIndexCourseAbweichung];

  CourseAbweichung = (WindRichtung * WindSeite - WindCourseToSteer) * 10;

  CourseAbweichungEimer[readIndexCourseAbweichung] = CourseAbweichung;

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer + CourseAbweichungEimer[readIndexCourseAbweichung];
  AverageCourseAbweichung = (float)TotalCourseAbweichungEimer / (float)numReadingsCourseAbweichung;

  readIndexCourseAbweichung = readIndexCourseAbweichung + 1;
  if (readIndexCourseAbweichung >= numReadingsCourseAbweichung) {
    readIndexCourseAbweichung = 0;
  }

  WindRichtungLast = WindRichtung;
  WindSeiteLast = WindSeite;
}

void CalcTurnrateAndOffcourseOpenCPN(void) {
  TotalDrehrateEimer = TotalDrehrateEimer - DrehrateEimer[readIndexDrehrate];

  if ((MagCourseLast > 2700) && (MagCourse < 900)) {
    Drehrate = (3600 - MagCourseLast) + MagCourse;
  }
  else if ((MagCourseLast < 900) && (MagCourse > 2700)) {
    Drehrate = (MagCourse - 3600) - MagCourseLast;
  }
  else {
    Drehrate = MagCourse - MagCourseLast;
  }

  DrehrateEimer[readIndexDrehrate] = Drehrate;

  TotalDrehrateEimer = TotalDrehrateEimer + DrehrateEimer[readIndexDrehrate];
  AverageDrehrate = (float)TotalDrehrateEimer / (float)numReadingsDrehrate;

  readIndexDrehrate = readIndexDrehrate + 1;
  if (readIndexDrehrate >= numReadingsDrehrate) {
    readIndexDrehrate = 0;
  }

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer - CourseAbweichungEimer[readIndexCourseAbweichung];

  if ((NMEACourseToSteer > 2700) && (NMEACourseActual < 900)) {
    CourseAbweichung = (3600 - NMEACourseToSteer) + NMEACourseActual;
  }
  else if ((NMEACourseToSteer < 900) && (NMEACourseActual > 2700)) {
    CourseAbweichung = (NMEACourseActual - 3600) - NMEACourseToSteer;
  }
  else {
    CourseAbweichung = NMEACourseActual - NMEACourseToSteer;
  }

  CourseAbweichungEimer[readIndexCourseAbweichung] = CourseAbweichung;

  TotalCourseAbweichungEimer = TotalCourseAbweichungEimer + CourseAbweichungEimer[readIndexCourseAbweichung];
  AverageCourseAbweichung = (float)TotalCourseAbweichungEimer / (float)numReadingsCourseAbweichung;

  readIndexCourseAbweichung = readIndexCourseAbweichung + 1;
  if (readIndexCourseAbweichung >= numReadingsCourseAbweichung) {
    readIndexCourseAbweichung = 0;
  }

  MagCourseLast = MagCourse;
}

void CalcNextCheck(void) {
  DrehRuder = PilotTmax;

  if ((int)AverageDrehrate != 0) {
    DrehRuder = TSteilheit / abs(AverageDrehrate);
  }
  if (FDW > 0) {
    DrehRuder = DrehRuder / (FDWMultiplikator * ((float)FDW / 100.0));
  }

  if (DrehRuder > PilotTmax) {
    DrehRuder = PilotTmax;
  }
  else if (DrehRuder < PilotTmin) {
    DrehRuder = PilotTmin;
  }
  if ((PilotNextCheck > (millis() + (int)round(DrehRuder * 100.0))) || (PilotNextCheck <= millis())) {
    PilotNextCheck = millis() + (int)round(DrehRuder * 100.0);
  }
}

void CalcPID(void) {

  PIDOutAbweich = (-1.0) * Kp * (float)AverageCourseAbweichung / 10.0 + (-1.0) * Ki * Ta + (-1.0) * Kd * (((float)AverageCourseAbweichung / 10.0 - PIDOutAbweichAlt)) / Ta;
  PIDOutAbweichAlt = AverageCourseAbweichung / 10.0;

  PIDOutDreh = (-1.0) * Kp * AverageDrehrate + (-1.0) * Ki * Ta + (-1.0) * Kd * ((AverageDrehrate - PIDOutDrehAlt)) / Ta;
  PIDOutDrehAlt = AverageDrehrate;

}

void AveragePID(void) {
  PIDRuderlage = round(PIDOutAbweich + PIDOutDreh);

  TotalRuderLageEimer = TotalRuderLageEimer - RuderLageEimer[readIndexRuderLage];
  RuderLageEimer[readIndexRuderLage] = PIDRuderlage;
  TotalRuderLageEimer = TotalRuderLageEimer + RuderLageEimer[readIndexRuderLage];
  PIDAverageRuderLage = TotalRuderLageEimer / (float)numReadingsRuderLage;

  readIndexRuderLage = readIndexRuderLage + 1;
  if (readIndexRuderLage >= numReadingsRuderLage) {
    readIndexRuderLage = 0;
  }

  PIDRuderlage = PIDAverageRuderLage;
}

