#ifndef functions.h
  #define functions.h

  #include <avr/interrupt.h>
  #include <avr/io.h>
  #include <Wire.h>
  #include "U8glib.h"
  #include <math.h> 
  #include <EEPROM.h>
  #include <Arduino.h>
  #include <ELECHOUSE_CC1101.h>
  #include <FastCRC.h>
	#include <avr/wdt.h>
//  #include <PID_v1.h>
  #include <TimerOne.h>
  
  #include "functions.h"

  #define byte uint8_t

  #define DEBUG 0               //Debug über UART (115200 Baud!)
  #define GeberInvert 1
  #define MotorInvert 1
  #define LimitInvert 1
  #define TargetInvert 1
  #define SetPointInvert 0

  // EEPROM Adressen

  #define LowMagOffsetAdr         1
  #define HighMagOffsetAdr        2
  #define Geber1MiddleLowAdr      3
  #define Geber1MiddleHighAdr     4
  #define Geber2MiddleLowAdr      5
  #define Geber2MiddleHighAdr     6
  #define Geber1MinusLowAdr       7
  #define Geber1MinusHighAdr      8
  #define Geber2MinusLowAdr       9
  #define Geber2MinusHighAdr      10
  #define Geber1PlusLowAdr        11
  #define Geber1PlusHighAdr       12
  #define Geber2PlusLowAdr        13
  #define Geber2PlusHighAdr       14
  #define WindCourseToSteerAdr    15
  #define WindSeiteToSteerAdr     16
  #define MagCourseToSteerLowAdr  17
  #define MagCourseToSteerHighAdr 18
  #define MotorMaximalSpeedAdr    19
  #define MotorMinimalSpeedAdr    20

  #define KpVorKommaAdr           21
  #define KpNachKommaAdr          22
  #define KdVorKommaAdr           23
  #define KdNachKommaAdr          24
  #define KiVorKommaAdr           25
  #define KiNachKommaAdr          26
  #define TaVorKommaAdr           27
  #define TaNachKommaAdr          28

  #define FDWCorrVorKommaAdr      29
  #define FDWCorrNachKommaAdr     30

	#define PilotTmaxVorKommaAdr    31
  #define PilotTmaxNachKommaAdr   32

	#define PilotTminVorKommaAdr    33
  #define PilotTminNachKommaAdr   34

	#define TSteilheitVorKommaAdr   35
  #define TSteilheitNachKommaAdr  36
	
	#define FDWMultiplikatorVorKommaAdr  37
  #define FDWMultiplikatorNachKommaAdr 38
		
  #define OperationModeAdr 				39


	#define pi 3.14159265359
  
  // Konstanten
  #define TasterA                 36                   //Taste A
  #define TasterB                 34                   //Taste B
  #define TasterC                 32                   //Taste C
  #define TasterD                 30                   //Taste D
  
  #define LangDruck               500                //Wie lang muß Langer Tastendruck sein?
  
  #define numReadings             32                  //Mittelungen

  #define MotorBitA               22
  #define MotorBitB               24
  #define MotorPWM                6

  #define GDO0  3

  #define numReadingsDrehrate             8
  #define numReadingsCourseAbweichung     8
	#define numReadingsRuderLage						5

	//GUI Definitionen
	#define GUI_Mainscreen 									0
	#define GUI_CompassSetUp 								1
	#define GUI_MotorSetUp 									2
	#define GUI_433MHzDebug 								3
	#define GUI_RudderSetUp 								4
	#define GUI_RudderZeroSetUp 						5
	#define GUI_RudderLimitsSetUp 					6
	#define GUI_MotorSpeedLimitsSetUp 			7
	#define GUI_MotorSpeedMaxLimitsSetUp 		8
	#define GUI_MotorSpeedMinLimitsSetUp 		9
	#define GUI_PIDSetUp 										10
	#define GUI_PID_P_ValueSetUp 						11
	#define GUI_PID_I_ValueSetUp 						12
	#define GUI_PID_D_ValueSetUp 						13
	#define GUI_PID_Ta_ValueSetUp 					14
	#define GUI_FDWSetUp 										15
	#define GUI_DynamicSetUp 						  	16
	#define GUI_Dynamic_PmaxSetUp 					17
	#define GUI_Dynamic_PminSetUp 					18
	#define GUI_Dynamic_TSteilheitSetUp 		19
	#define GUI_Dynamic_FDWMultiSetUp		 		20
	#define GUI_433MHzNMEADebug 						21
	#define GUI_Error_NoCalb 								100
	#define GUI_Saved_Calb	 								101
	#define GUI_Saved_MotorLimits_Calb	 		102
	#define GUI_Saved_PID_Params			 			103
	#define GUI_Saved_Logge_Calb			 			104
	#define GUI_Error_MotorDisabled			 		105
	#define GUI_Saved_Dynamics			 				106
	
	#define Menu_None													-1
	#define Menu_Main													1
	#define Menu_Compass_Main_SetUp						2
	#define Menu_Compass_Calibrate_SetUp			21
	#define Menu_Motor_Main_SetUp							3
	#define Menu_Motor_Main_Test_Run_SetUp		4
	#define Menu_Motor_Main_Test_Stop_SetUp		5
	#define Menu_Motor_Main_Test_State_SetUp	6
	#define Menu_Motor_Main_Test_Speed_SetUp	7
	#define Menu_Rudder_Main_SetUp						8
	#define Menu_Rudder_Zero_SetUp						81
	#define Menu_Rudder_Limits_SetUp					82
	#define Menu_Debug_SetUp									9
	#define Menu_MagWind_Pilot_ON									10
	#define Menu_MagWind_Pilot_OFF								11
	#define Menu_Motor_Limits_SetUp						12
	#define Menu_Motor_Limits_PlusMinus_SetUp	13
	#define Menu_PID_SetUp										14
	#define Menu_PID_Ta_SetUp									15
	#define Menu_PID_P_SetUp									16
	#define Menu_PID_I_SetUp									17
	#define Menu_PID_D_SetUp									18
	#define Menu_Logge_SetUp									19
	#define Menu_Logge_Adjust_SetUp				  	20
	#define Menu_Dynamics_SetUp						  	22
	#define Menu_Dynamics_TMax_SetUp					23
	#define Menu_Dynamics_TMin_SetUp					24
	#define Menu_Dynamics_Steilheit_SetUp			25
	#define Menu_Dynamics_STWMult_SetUp				26
	#define Menu_Debug_NMEA_SetUp							27
	#define Menu_OpenCPN_Pilot_OFF						28
	#define Menu_OpenCPN_Pilot_ON							29
	
  //CRC
  extern unsigned long CRCMicro;
  extern unsigned long CRCMega;
  extern bool CRCValid;
  extern unsigned int CRCFailCounter;
	
  //Buffer
  extern byte buffer[256];
  extern bool TransponderDataWaiting;
  extern float MagCourseFloat;

  extern int MagCourse;
  extern int Geber1;
  extern int Geber2;
  extern int WindGeschwindigkeit;
  extern int WindRichtung;
  extern int WindRichtungLast;
  extern int FDW;

  extern int MagCourseTrans;
  extern int Geber1Trans;
  extern int Geber2Trans;
  extern int WindGeschwindigkeitTrans;
  extern int WindRichtungTrans;
  extern int FDWTrans;
  
  extern int NMEACourseActualTrans;
  extern int NMEACourseToSteerTrans;
  extern int NMEACPNPilotActiveTrans;
  extern int NMEADistanceToWPTrans;
  extern int NMEASOGTrans;
  extern int NMEANextWPTrans;

	extern int NMEACPNPilotActive;
	extern int NMEACourseActual;
	extern int NMEACourseToSteer;
	extern int NMEADistanceToWP;
	extern int NMEANextWP;
	extern int NMEASOG;
	
  extern byte Token;
	
	extern bool GotDataBlinker;

  //Rudergeber
  extern int Geber1Middle;
  extern int Geber2Middle;
  extern int Geber1Minus;
  extern int Geber2Minus;
  extern int Geber1Plus;
  extern int Geber2Plus;

  extern int GeberInPercent;
  extern int GeberRichtung;

  extern char GeberBoxStart;
  extern char GeberBoxEnd;

  // Variablen für Tastenfeld
  extern bool TasteReadA;
  extern bool TasteReadB;
  extern bool TasteReadC;
  extern bool TasteReadD;
  extern bool TasteStateA;
  extern bool TasteStateB;
  extern bool TasteStateC;
  extern bool TasteStateD;
  extern bool TasteLangDruckStateA;
  extern bool TasteLangDruckStateB;
  extern bool TasteLangDruckStateC;
  extern bool TasteLangDruckStateD;
  extern bool TasteLastReadA;
  extern bool TasteLastReadB;
  extern bool TasteLastReadC;
  extern bool TasteLastReadD;
  extern unsigned long TasteLastDebounceTime;
  extern unsigned char debounceDelay;
  extern uint16_t LangDruckRunner;
  
// Motor
  extern byte MotorActualSetupScreen;
  extern int MotorDirection; //(-1 = reverse, 1 = forward)
  extern bool MotorRunning;
  extern bool MotorEnabled;
  extern uint8_t MotorSpeed;
  extern int MotorSetPoint;
  extern int MotorSetPointDirection;
//  extern int MotorGeberSchlacker;
  extern bool MotorStop;
  extern byte MotorMaximalSpeed; 
  extern byte MotorMinimalSpeed; 
  
// Laufvariablen
  extern uint16_t runner;
  extern byte readIndex;
  extern byte LastMenue;
  extern byte ActualScreen;
  extern byte ActualScreenPrev;

// Debug zum Timing testen
  extern unsigned long MicroStart;
  extern unsigned long MicroStop;

//Kompass
  extern bool MagCalbDone;
  extern bool MagInCalc;
  extern int  MagOffset;

  extern bool MagPilotEnabled;
  extern int MagCourseToSteer;
  extern int CourseAbweichung;
  extern int MagCourseLast;

  extern int CourseAbweichungEimer[numReadingsCourseAbweichung];   
  extern long TotalCourseAbweichungEimer;                  
  extern int AverageCourseAbweichung;
  extern int readIndexCourseAbweichung;	
	
//OpenCpn
  extern bool OpenCPNPilotEnabled;
  extern int OpenCPNCourseLast;
  
//WindAutoPilot
  extern bool WindPilotEnabled;
  extern int WindSeite;
	extern int WindSeiteLast;
  extern int WindCourseToSteer;
  extern int WindSeiteToSteer;
  extern char WindgeschwindigkeitStr[6];

//Logge
  extern char FDWStr[6];
  extern float FDWCorr;

//Drehrate
  extern int DrehrateEimer[numReadingsDrehrate];   
  extern long TotalDrehrateEimer;                  
  extern int Drehrate;
  extern float AverageDrehrate;
  extern int readIndexDrehrate;

//0 = Standby, 1 = Mag, 2 = Wind, 3 = OpenCPN
  extern byte OperationMode;

//PID 
  extern float Kp;
  extern float Kd;
  extern float Ki;
  extern float Ta;
	extern float PIDOutAbweich;
	extern float PIDOutDreh;
	extern float PIDOutAbweichAlt;
	extern float PIDOutDrehAlt;
	extern float PIDOutAbweichSum;
	extern float PIDOutDrehSum;
	extern int PIDRuderlage;

  extern float RuderLageEimer[numReadingsRuderLage];   
  extern float TotalRuderLageEimer;                  
  extern float PIDAverageRuderLage;
  extern int readIndexRuderLage;

	
//Regelhäufigkeits Parameter
	extern float PilotTmax; 
	extern float PilotTmin;	
	extern float TSteilheit;
	extern float DrehRuder;	
	extern unsigned long PilotNextCheck;
	extern float FDWMultiplikator;
	extern byte WaitUntiStart;
	
  void ReadEEPROM(void); 
  void WriteEEPROM(void);
  void WriteEEPROMWind(void);
  void WriteEEPROMMag(void);
  void InterruptTimerSetUp(void);
  void DisplaySetUp(void);
  void SetMenue(byte);
  void SetScreen(byte);
  int  OffsetOLED(int,int,int);  
  void DislayShowWelcomeMsg(void);
  void DisplayShowScreen(int,int);

  void HandleMenue(void);

  void HandleActualScreen0ButtonB(void);
  void HandleActualScreen1ButtonB(void);
  void HandleActualScreen2ButtonB(void);

  void HandleActualScreen0ButtonC(void);
  void HandleActualScreen1ButtonC(void);
  void HandleActualScreen2ButtonC(void);
  void HandleActualScreen8ButtonC(void);
  void HandleActualScreen9ButtonC(void);
  void HandleActualScreen11ButtonC(void);
  void HandleActualScreen12ButtonC(void);
  void HandleActualScreen13ButtonC(void);
  void HandleActualScreen14ButtonC(void);
  void HandleActualScreen15ButtonC(void);
  void HandleActualScreen17ButtonC(void);
  void HandleActualScreen18ButtonC(void);
  void HandleActualScreen19ButtonC(void);
  void HandleActualScreen20ButtonC(void);

  void HandleActualScreen0ButtonD(void);
  void HandleActualScreen1ButtonD(void);
  void HandleActualScreen2ButtonD(void);
  void HandleActualScreen8ButtonD(void);
  void HandleActualScreen9ButtonD(void);
  void HandleActualScreen11ButtonD(void);
  void HandleActualScreen12ButtonD(void);
  void HandleActualScreen13ButtonD(void);
  void HandleActualScreen14ButtonD(void);
  void HandleActualScreen15ButtonD(void);
  void HandleActualScreen17ButtonD(void);
  void HandleActualScreen18ButtonD(void);
  void HandleActualScreen19ButtonD(void);
  void HandleActualScreen20ButtonD(void);
	
  void HandleLongPressOperationMode1ButtonB(void);
  void HandleLongPressOperationMode2ButtonB(void);

  void HandleLongPressOperationMode1ButtonC(void);
  void HandleLongPressOperationMode2ButtonC(void);
  void HandleLongPressActualScreen1ButtonC(void);
  void HandleLongPressActualScreen2ButtonC(void);
  void HandleLongPressActualScreen8ButtonC(void);
  void HandleLongPressActualScreen9ButtonC(void);

  void HandleLongPressActualScreen11ButtonC(void);
  void HandleLongPressActualScreen12ButtonC(void);
  void HandleLongPressActualScreen13ButtonC(void);
  void HandleLongPressActualScreen14ButtonC(void);
  void HandleLongPressActualScreen15ButtonC(void);

	void HandleLongPressActualScreen17ButtonC(void);	
	void HandleLongPressActualScreen18ButtonC(void);	
	void HandleLongPressActualScreen19ButtonC(void);	
	void HandleLongPressActualScreen20ButtonC(void);	
	
  void HandleLongPressActualScreen1ButtonD(void);
  void HandleLongPressActualScreen2ButtonD(void);
  void HandleLongPressActualScreen8ButtonD(void);
  void HandleLongPressActualScreen9ButtonD(void);

  void HandleLongPressActualScreen11ButtonD(void);
  void HandleLongPressActualScreen12ButtonD(void);
  void HandleLongPressActualScreen13ButtonD(void);
  void HandleLongPressActualScreen14ButtonD(void);
  void HandleLongPressActualScreen15ButtonD(void);

	void HandleLongPressActualScreen17ButtonD(void);	
	void HandleLongPressActualScreen18ButtonD(void);	
	void HandleLongPressActualScreen19ButtonD(void);	
	void HandleLongPressActualScreen20ButtonD(void);	
	
  void MotorSet(bool, bool, int);
  void EntprellAndRead(void);
  void TransponderInit(void);
  void TransponderDataAvailable(bool);
  void TransponderReadData(void);
  bool CheckCRC(void);
  byte LEnOfInt(int);
  void CalcRudder(void);
  void SwitchEnabledPilot(byte);
  void DoPilot(void);
  void CheckMotor(void);
  void MotorDriveToAngle(void);
  void MotorCheckIfLimit(void);
	void CalcTurnrateAndOffcourseMag(void);
	void CalcTurnrateAndOffcourseWind(void);
	void CalcTurnrateAndOffcourseOpenCPN(void);
	void CalcNextCheck(void);
	void CalcPID(void);
	void AveragePID(void);
  
#endif

