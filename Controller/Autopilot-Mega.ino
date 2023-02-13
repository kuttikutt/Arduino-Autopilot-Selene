/*
  MicroStart = micros();
  MicroStop = micros();
  Serial.println(MicroStop - MicroStart);
*/

/*
   Motor:
   En = H - Betrieb
   C=D = bremsen
*/

#include "functions.h"

//Serial Debug
const int n = 61;
byte serbuffer[n] = "";
int i = 0;


void DataReady(void) {
  TransponderDataWaiting = true;
}

// Timer2 Overflow Interrupt Vector, called every 8ms (125Hz)
ISR(TIMER2_OVF_vect) {
  TransponderReadData();

  CalcRudder();
  MotorCheckIfLimit();
  CheckMotor();

  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
}

void setup() {
  InterruptTimerSetUp();  //Timer2 als 8ms Interrupt

#if DEBUG
  Serial.begin(115200);   //serielle schnittstelle zum Debug
#endif

  ReadEEPROM();

  DisplaySetUp();
  DislayShowWelcomeMsg();
  DisplayShowScreen(Menu_Main, -1); //Menue, Screen (-1  for nothing)

  // Taster
  pinMode(TasterA, INPUT_PULLUP);
  pinMode(TasterB, INPUT_PULLUP);
  pinMode(TasterC, INPUT_PULLUP);
  pinMode(TasterD, INPUT_PULLUP);

  pinMode(MotorBitA, OUTPUT);
  pinMode(MotorBitB, OUTPUT);
  pinMode(MotorPWM,  OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  TransponderInit();

  attachInterrupt(digitalPinToInterrupt(GDO0), DataReady, FALLING);

  //Timer für Steuerung
  Timer1.initialize(100000);
  Timer1.attachInterrupt(DoPilot); // blinkLED to run every 0.15 seconds

  MotorSet(false, false, 1); //MotorSet(Enable/Disable[1/0],Brake/Left/Right[0/1/2],Speed[0-255])

  wdt_enable(WDTO_2S);
}

void loop() {


#if DEBUG
  //Grad,Richtung
  if (Serial.available()) {
    int len = Serial.readBytesUntil('\n', serbuffer, n);
    serbuffer[len] = '\0';
    Serial.println((char *)serbuffer);

    char *token = strtok(serbuffer, ",");

    //int MotorSetPoint;
    //bool MotorSetPointDirection;

    while (token) {
      // Serial.println((char *)token);
      switch (i) {
        case 0:
          MotorSetPoint = atoi (token);
          break;
        case 1:
          MotorSetPointDirection = atoi (token);
          break;
        default:
          break;
      }
      i++;
      token = strtok(NULL, ",");
    }
    i = 0;
    //GeberInPercent
    //GeberRichtung
    MotorSpeed = 30;

    MotorDriveToAngle();

  }
#endif

  // Tasten Einlesen
  TasteReadA = digitalRead(TasterA);
  TasteReadB = digitalRead(TasterB);
  TasteReadC = digitalRead(TasterC);
  TasteReadD = digitalRead(TasterD);


  if ((TasteReadA || TasteReadB || TasteReadC || TasteReadD) == true) {
    if (LangDruckRunner >= LangDruck) {
      if (TasteReadA == true) {
        TasteLangDruckStateA = true;
        if (LastMenue == Menu_Main) {
          DisplayShowScreen(Menu_Compass_Main_SetUp, 1);
        }
      }
      else if (TasteReadB == true) {
        TasteLangDruckStateB = true;
        if (ActualScreen == GUI_Mainscreen)  {
          if (OperationMode == 1) {
            //MagMode
            HandleLongPressOperationMode1ButtonB();
          }
          else if (OperationMode == 2) {
            //WindMode
            HandleLongPressOperationMode2ButtonB();
          }
        }
      }
      else if (TasteReadC == true) {
        TasteLangDruckStateC = true;
        if (ActualScreen == GUI_Mainscreen)  {
          if (OperationMode == 1) {
            //MagMode
            HandleLongPressOperationMode1ButtonC();
          }
          else if (OperationMode == 2) {
            //WindMode
            HandleLongPressOperationMode2ButtonC();
          }
        }
        else if (ActualScreen == GUI_CompassSetUp)  {
          //MagOffset += 5
          HandleLongPressActualScreen1ButtonC();
        }
        if (ActualScreen == GUI_MotorSetUp) {
          //MotorSpeed += 5
          HandleLongPressActualScreen2ButtonC();
        }
        else if (ActualScreen == GUI_MotorSpeedMaxLimitsSetUp) {
          //MotorMaximalSpeed += 5
          HandleLongPressActualScreen8ButtonC();
        }
        else if (ActualScreen == GUI_MotorSpeedMinLimitsSetUp) {
          //MotorMinimalSpeed += 5
          HandleLongPressActualScreen9ButtonC();
        }
        else if (ActualScreen == GUI_PID_P_ValueSetUp) {
          HandleLongPressActualScreen11ButtonC();
        }
        else if (ActualScreen == GUI_PID_I_ValueSetUp) {
          HandleLongPressActualScreen12ButtonC();
        }
        else if (ActualScreen == GUI_PID_D_ValueSetUp) {
          HandleLongPressActualScreen13ButtonC();
        }
        else if (ActualScreen == GUI_PID_Ta_ValueSetUp) {
          HandleLongPressActualScreen14ButtonC();
        }
        else if (ActualScreen == GUI_FDWSetUp) {
          HandleLongPressActualScreen15ButtonC();
        }
        else if (ActualScreen == GUI_Dynamic_PmaxSetUp) {
          HandleLongPressActualScreen17ButtonC();
        }
        else if (ActualScreen == GUI_Dynamic_PminSetUp) {
          HandleLongPressActualScreen18ButtonC();
        }
        else if (ActualScreen == GUI_Dynamic_TSteilheitSetUp) {
          HandleLongPressActualScreen19ButtonC();
        }
        else if (ActualScreen == GUI_Dynamic_FDWMultiSetUp) {
          HandleLongPressActualScreen20ButtonC();
        }
      }
      else if (TasteReadD == true) {
        TasteLangDruckStateD = true;
        if (ActualScreen == GUI_CompassSetUp)  {
          //MagOffset -= 5
          HandleLongPressActualScreen1ButtonD();
        }
        else if (ActualScreen == GUI_MotorSetUp) {
          //MotorSpeed -= 5
          HandleLongPressActualScreen2ButtonD();
        }
        else if (ActualScreen == GUI_MotorSpeedMaxLimitsSetUp) {
          //MotorMaximalSpeed -= 5
          HandleLongPressActualScreen8ButtonD();
        }
        else if (ActualScreen == GUI_MotorSpeedMinLimitsSetUp) {
          //MotorMinimalSpeed -= 5
          HandleLongPressActualScreen9ButtonD();
        }
        else if (ActualScreen == GUI_PID_P_ValueSetUp) {
          HandleLongPressActualScreen11ButtonD();
        }
        else if (ActualScreen == GUI_PID_I_ValueSetUp) {
          HandleLongPressActualScreen12ButtonD();
        }
        else if (ActualScreen == GUI_PID_D_ValueSetUp) {
          HandleLongPressActualScreen13ButtonD();
        }
        else if (ActualScreen == GUI_PID_Ta_ValueSetUp) {
          HandleLongPressActualScreen14ButtonD();
        }
        else if (ActualScreen == GUI_FDWSetUp) {
          HandleLongPressActualScreen15ButtonD();
        }
        else if (ActualScreen == GUI_Dynamic_PmaxSetUp) {
          HandleLongPressActualScreen17ButtonD();
        }
        else if (ActualScreen == GUI_Dynamic_PminSetUp) {
          HandleLongPressActualScreen18ButtonD();
        }
        else if (ActualScreen == GUI_Dynamic_TSteilheitSetUp) {
          HandleLongPressActualScreen19ButtonD();
        }
        else if (ActualScreen == GUI_Dynamic_FDWMultiSetUp) {
          HandleLongPressActualScreen20ButtonD();
        }
      }
    }
    LangDruckRunner++;
  }

  // Entprellung
  EntprellAndRead();

  if ((millis() - TasteLastDebounceTime) > debounceDelay) {
    if (TasteReadA != TasteStateA) {
      TasteStateA = TasteReadA;
      if (TasteStateA == false) {
        if (LangDruckRunner < LangDruck) {
#if DEBUG
          Serial.print(ActualScreen);
          Serial.print(", Lastmenue: ");
          Serial.println(LastMenue);
#endif
          LangDruckRunner = 0;
          //Taste Back
          HandleMenue();
        }
      }
    }
    else if (TasteReadB != TasteStateB) {
      TasteStateB = TasteReadB;
      if (TasteStateB == false) {
        if (LangDruckRunner < LangDruck) {
#if DEBUG
          Serial.print("Taste B - Actual Screen: ");
          Serial.print(ActualScreen);
#endif
          if (ActualScreen == GUI_Mainscreen) {
#if DEBUG
            Serial.print(ActualScreen);
            Serial.println(", Screen0ButtonB()");
#endif
            HandleActualScreen0ButtonB();
          }
          else if (ActualScreen == GUI_CompassSetUp) {
            //Save im Kompass Kalibration
#if DEBUG
            Serial.println(", Screen1ButtonB()");
#endif
            HandleActualScreen1ButtonB();
          }
          else if (ActualScreen == GUI_MotorSetUp) {
            //Taste B im Motormenü
#if DEBUG
            Serial.println(", Screen2ButtonB()");
#endif
            HandleActualScreen2ButtonB();
          }
          else if (ActualScreen == GUI_433MHzDebug) {
            CRCFailCounter = 0;
          }
          else if (ActualScreen == GUI_RudderSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Rudder_Limits_SetUp, GUI_RudderLimitsSetUp) + Read EEProm");
#endif
            ReadEEPROM();
            DisplayShowScreen(Menu_Rudder_Limits_SetUp, GUI_RudderLimitsSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderLimitsSetUp) {
#if DEBUG
            Serial.println(", GeberMinus setzen");
#endif
            Geber1Minus = Geber1;
            Geber2Minus = Geber2;
          }
          else if (ActualScreen == GUI_MotorSpeedLimitsSetUp) {
#if DEBUG
            Serial.println(", MotorLimits -> DisplayShowScreen(Menu_Motor_Limits_SetUp, GUI_MotorSpeedLimitsSetUp) + WriteEEPROM");
#endif
            WriteEEPROM();
            //Calibration saved...
            DisplayShowScreen(Menu_None, GUI_Saved_MotorLimits_Calb); //Menue, Screen (-1  for nothing)
            delay(1000);
            DisplayShowScreen(Menu_Motor_Limits_SetUp, GUI_MotorSpeedLimitsSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PIDSetUp) {
#if DEBUG
            Serial.println(", PID Params -> DisplayShowScreen(Menu_PID_SetUp, GUI_PIDSetUp) + WriteEEPROM");
#endif
            WriteEEPROM();
            //Calibration saved...
            DisplayShowScreen(Menu_None, GUI_Saved_PID_Params); //Menue, Screen (-1  for nothing)
            delay(1000);
            DisplayShowScreen(Menu_PID_SetUp, GUI_PIDSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PID_P_ValueSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp)");
#endif
            DisplayShowScreen(Menu_PID_I_SetUp, GUI_PID_I_ValueSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PID_I_ValueSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp)");
#endif
            DisplayShowScreen(Menu_PID_D_SetUp, GUI_PID_D_ValueSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PID_D_ValueSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp)");
#endif
            DisplayShowScreen(Menu_PID_Ta_SetUp, GUI_PID_Ta_ValueSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PID_Ta_ValueSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_P_SetUp, GUI_PIDSetUp)");
#endif
            DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_FDWSetUp) {
            if (LastMenue == Menu_Logge_Adjust_SetUp) {
#if DEBUG
              Serial.println(", Logge Calib -> DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp); + WriteEEPROM");
#endif
              WriteEEPROM();
              //Calibration saved...
              DisplayShowScreen(Menu_None, GUI_Saved_Logge_Calb); //Menue, Screen (-1  for nothing)
              delay(1000);
              DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
            }
          }
          else if (ActualScreen == GUI_DynamicSetUp) {
            if (LastMenue == 22) {
#if DEBUG
              Serial.println(", Dynamics Save -> DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp); + WriteEEPROM");
#endif
              WriteEEPROM();
              //Calibration saved...
              DisplayShowScreen(Menu_None, GUI_Saved_Dynamics); //Menue, Screen (-1  for nothing)
              delay(1000);
              DisplayShowScreen(Menu_Dynamics_SetUp, GUI_DynamicSetUp); //Menue, Screen (-1  for nothing)
            }
          }
          else if (ActualScreen == GUI_Dynamic_PmaxSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_TMin_SetUp, GUI_Dynamic_PminSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_Dynamic_PminSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_Steilheit_SetUp, GUI_Dynamic_TSteilheitSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_Dynamic_TSteilheitSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_STWMult_SetUp, GUI_Dynamic_FDWMultiSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_Dynamic_FDWMultiSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_433MHzNMEADebug) {
            CRCFailCounter = 0;
          }
          else {
#if DEBUG
            Serial.println(", Nix zu Tun?!");
#endif
          }

          LangDruckRunner = 0;
        }
      }
    }
    else if (TasteReadC != TasteStateC) {
      TasteStateC = TasteReadC;
      if (TasteStateC == false) {
        if (LangDruckRunner < LangDruck) {
#if DEBUG
          Serial.print("Taste C - Actual Screen: ");
          Serial.print(ActualScreen);
#endif
          if (ActualScreen == GUI_Mainscreen) {
            //Sprung ins WindSteuermenü
#if DEBUG
            Serial.println(", Screen0ButtonC()");
#endif
            HandleActualScreen0ButtonC();
          }
          else if (ActualScreen == GUI_CompassSetUp) {
            //Kompass Kalibrieren
#if DEBUG
            Serial.println(", Screen1ButtonC()");
#endif
            HandleActualScreen1ButtonC();
          }
          else if (ActualScreen == GUI_MotorSetUp) {
            //Motor Testmenü
#if DEBUG
            Serial.println(", Screen2ButtonC() + ReadEEProm");
#endif
            ReadEEPROM();
            HandleActualScreen2ButtonC();
          }
          else if (ActualScreen == GUI_433MHzDebug) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Debug_NMEA_SetUp, GUI_433MHzNMEADebug)");
#endif
            DisplayShowScreen(Menu_Debug_NMEA_SetUp, GUI_433MHzNMEADebug); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Rudder_Zero_SetUp, GUI_RudderZeroSetUp) + ReadEEProm");
#endif
            ReadEEPROM();
            DisplayShowScreen(Menu_Rudder_Zero_SetUp, GUI_RudderZeroSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderLimitsSetUp) {
#if DEBUG
            Serial.println(", GeberPlus setzen");
#endif
            Geber1Plus = Geber1;
            Geber2Plus = Geber2;
          }
          else if (ActualScreen == GUI_MotorSpeedLimitsSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp)");
#endif
            DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMinLimitsSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_MotorSpeedMaxLimitsSetUp) {
#if DEBUG
            Serial.println(", Screen8ButtonC()");
#endif
            HandleActualScreen8ButtonC();
          }
          else if (ActualScreen == GUI_MotorSpeedMinLimitsSetUp) {
#if DEBUG
            Serial.println(", Screen9ButtonC()");
#endif
            HandleActualScreen9ButtonC();
          }
          else if (ActualScreen == GUI_PIDSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp)");
#endif
            DisplayShowScreen(Menu_PID_P_SetUp, GUI_PID_P_ValueSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_PID_P_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen11ButtonC()");
#endif
            HandleActualScreen11ButtonC();
          }
          else if (ActualScreen == GUI_PID_I_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen12ButtonC()");
#endif
            HandleActualScreen12ButtonC();
          }
          else if (ActualScreen == GUI_PID_D_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen13ButtonC()");
#endif
            HandleActualScreen13ButtonC();
          }
          else if (ActualScreen == GUI_PID_Ta_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen14ButtonC()");
#endif
            HandleActualScreen14ButtonC();
          }
          else if (ActualScreen == GUI_FDWSetUp) {
            if (LastMenue == Menu_Logge_SetUp) {
              DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp); //Menue, Screen (-1  for nothing)
#if DEBUG
              Serial.println(", DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp);");
#endif
            }
            else if (LastMenue == Menu_Logge_Adjust_SetUp) {
#if DEBUG
              Serial.println(", Screen15ButtonC();");
#endif
              HandleActualScreen15ButtonC();
            }
          }
          else if (ActualScreen == GUI_DynamicSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_PID_P_ValueSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_TMax_SetUp, GUI_Dynamic_PmaxSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_Dynamic_PmaxSetUp) {
#if DEBUG
            Serial.println(", Screen17ButtonC()");
#endif
            HandleActualScreen17ButtonC();
          }
          else if (ActualScreen == GUI_Dynamic_PminSetUp) {
#if DEBUG
            Serial.println(", Screen18ButtonC()");
#endif
            HandleActualScreen18ButtonC();
          }
          else if (ActualScreen == GUI_Dynamic_TSteilheitSetUp) {
#if DEBUG
            Serial.println(", Screen19ButtonC()");
#endif
            HandleActualScreen19ButtonC();
          }
          else if (ActualScreen == GUI_Dynamic_FDWMultiSetUp) {
#if DEBUG
            Serial.println(", Screen20ButtonC()");
#endif
            HandleActualScreen20ButtonC();
          }
          else {
#if DEBUG
            Serial.println(", Nix zu Tun?!");
#endif
          }
          LangDruckRunner = 0;
        }
      }
    }
    else if (TasteReadD != TasteStateD) {
      TasteStateD = TasteReadD;
      if (TasteStateD == false) {
        if (LangDruckRunner < LangDruck) {
#if DEBUG
          Serial.print("Taste B - Actual Screen: ");
          Serial.print(ActualScreen);
#endif
          if (ActualScreen == GUI_Mainscreen)  {
#if DEBUG
            Serial.println(", Screen0ButtonD()");
#endif
            HandleActualScreen0ButtonD();
          }
          else if (ActualScreen == GUI_CompassSetUp)  {
#if DEBUG
            Serial.println(", Screen1ButtonD()");
#endif
            HandleActualScreen1ButtonD();
          }
          else if (ActualScreen == GUI_MotorSetUp) {
#if DEBUG
            Serial.println(", Screen2ButtonD()");
#endif
            HandleActualScreen2ButtonD();
          }
          else if (ActualScreen == GUI_433MHzDebug) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Compass_Main_SetUp, GUI_CompassSetUp)");
#endif
            DisplayShowScreen(Menu_Compass_Main_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_PID_SetUp, GUI_PIDSetUp)");
#endif
            DisplayShowScreen(Menu_PID_SetUp, GUI_PIDSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderZeroSetUp) {
#if DEBUG
            Serial.println(", GeberMiddle setzen  + WriteEEPROM");
#endif
            Geber1Middle = Geber1;
            Geber2Middle = Geber2;
            WriteEEPROM();
            //Calibration saved...
            DisplayShowScreen(Menu_None, GUI_Saved_Calb); //Menue, Screen (-1  for nothing)
            delay(1000);
            DisplayShowScreen(Menu_Rudder_Main_SetUp, GUI_RudderSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_RudderLimitsSetUp) {
#if DEBUG
            Serial.println(", Kalibration speichern  + WriteEEPROM");
#endif
            WriteEEPROM();
            //Calibration saved...
            DisplayShowScreen(Menu_None, GUI_Saved_Calb); //Menue, Screen (-1  for nothing)
            delay(1000);
            DisplayShowScreen(Menu_Rudder_Main_SetUp, GUI_RudderSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_MotorSpeedLimitsSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp)");
#endif
            DisplayShowScreen(Menu_Motor_Limits_PlusMinus_SetUp, GUI_MotorSpeedMaxLimitsSetUp); //Menue, Screen (-1  for nothing)
          }
          else if (ActualScreen == GUI_MotorSpeedMaxLimitsSetUp) {
#if DEBUG
            Serial.println(", Screen8ButtonD()");
#endif
            HandleActualScreen8ButtonD();
          }
          else if (ActualScreen == GUI_MotorSpeedMinLimitsSetUp) {
#if DEBUG
            Serial.println(", Screen9ButtonD()");
#endif
            HandleActualScreen9ButtonD();
          }
          else if (ActualScreen == GUI_PIDSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_DynamicSetUp)");
#endif
            DisplayShowScreen(Menu_Dynamics_SetUp, GUI_DynamicSetUp);
          }
          else if (ActualScreen == GUI_PID_P_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen11ButtonD()");
#endif
            HandleActualScreen11ButtonD();
          }
          else if (ActualScreen == GUI_PID_I_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen12ButtonD()");
#endif
            HandleActualScreen12ButtonD();
          }
          else if (ActualScreen == GUI_PID_D_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen13ButtonD()");
#endif
            HandleActualScreen13ButtonD();
          }
          else if (ActualScreen == GUI_PID_Ta_ValueSetUp) {
#if DEBUG
            Serial.println(", Screen14ButtonD()");
#endif
            HandleActualScreen14ButtonD();
          }
          else if (ActualScreen == GUI_FDWSetUp) {
            if (LastMenue == Menu_Logge_SetUp) {
#if DEBUG
              Serial.println(", DisplayShowScreen(Menu_Motor_Main_SetUp, GUI_MotorSetUp);");
#endif
              DisplayShowScreen(Menu_Motor_Main_SetUp, GUI_MotorSetUp);
            }
            else if (LastMenue == Menu_Logge_Adjust_SetUp) {
#if DEBUG
              Serial.println(", Screen15ButtonD();");
#endif
              HandleActualScreen15ButtonD();
            }
          }
          else if (ActualScreen == GUI_DynamicSetUp) {
#if DEBUG
            Serial.println(", DisplayShowScreen(Menu_Debug_SetUp, GUI_433MHzDebug);");
#endif
            DisplayShowScreen(Menu_Debug_SetUp, GUI_433MHzDebug);
          }
          else if (ActualScreen == GUI_Dynamic_PmaxSetUp) {
#if DEBUG
            Serial.println(", Screen17ButtonD()");
#endif
            HandleActualScreen17ButtonD();
          }
          else if (ActualScreen == GUI_Dynamic_PminSetUp) {
#if DEBUG
            Serial.println(", Screen18ButtonD()");
#endif
            HandleActualScreen18ButtonD();
          }
          else if (ActualScreen == GUI_Dynamic_TSteilheitSetUp) {
#if DEBUG
            Serial.println(", Screen19ButtonD()");
#endif
            HandleActualScreen19ButtonD();
          }
          else if (ActualScreen == GUI_Dynamic_FDWMultiSetUp) {
#if DEBUG
            Serial.println(", Screen20ButtonD()");
#endif
            HandleActualScreen20ButtonD();
          }
          else {
#if DEBUG
            Serial.println(", Nix zu Tun?!");
#endif
          }
          LangDruckRunner = 0;
        }
      }
    }
  }

  if (runner % 100 == 0) {

    digitalWrite(LED_BUILTIN, true);

    if (ActualScreen == GUI_Mainscreen) {
      //Standbybildschirm
      switch (OperationMode) {
        case 0: //Standby Screen
          DisplayShowScreen(Menu_Main, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          break;
        case 1: //MagPilotScreen
          if (MagPilotEnabled == true) {
            DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          else {
            DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          break;
        case 2: //WindPilot Screen
          if (WindPilotEnabled == true) {
            DisplayShowScreen(Menu_MagWind_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          else {
            DisplayShowScreen(Menu_MagWind_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          break;
        case 3: //OpenCPN
          if (OpenCPNPilotEnabled == false) {
            DisplayShowScreen(Menu_OpenCPN_Pilot_OFF, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          else {
            DisplayShowScreen(Menu_OpenCPN_Pilot_ON, GUI_Mainscreen); //Menue, Screen (-1  for nothing)
          }
          break;
        default:
          break;
      }
    }
    else if (ActualScreen == GUI_CompassSetUp) {

      if (MagInCalc == false) {
        //Im SetUp, aber nicht im Kalibrieren
        DisplayShowScreen(Menu_Compass_Main_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
      }
      else {
        //Kalibration
        DisplayShowScreen(Menu_Compass_Calibrate_SetUp, GUI_CompassSetUp); //Menue, Screen (-1  for nothing)
      }

    }
    else if (ActualScreen == GUI_MotorSetUp) {
      if (MotorActualSetupScreen == 0) {
        DisplayShowScreen(Menu_Motor_Main_SetUp, GUI_MotorSetUp); //Menue, Screen (-1  for nothing)
      }
      else if (MotorActualSetupScreen == 1) {
        DisplayShowScreen((Menu_Motor_Main_Test_Run_SetUp + MotorRunning), GUI_MotorSetUp);
      }
      else if (MotorActualSetupScreen == 2) {
        DisplayShowScreen(Menu_Motor_Main_Test_State_SetUp, GUI_MotorSetUp);
      }
      else if (MotorActualSetupScreen == 3) {
        DisplayShowScreen(Menu_Motor_Main_Test_Speed_SetUp, GUI_MotorSetUp);
      }
    }
    else if (ActualScreen == GUI_433MHzDebug) {
      //433MHZ Menue

      DisplayShowScreen(Menu_Debug_SetUp, GUI_433MHzDebug); //Menue, Screen (-1  for nothing)
    }
    else if (ActualScreen == GUI_RudderSetUp) {
      //Ruder Menue
      DisplayShowScreen(Menu_Rudder_Main_SetUp, GUI_RudderSetUp); //Menue, Screen (-1  for nothing)
    }
    else if (ActualScreen == GUI_RudderZeroSetUp) {
      //Ruder Menue Zero Connect
      DisplayShowScreen(Menu_Rudder_Zero_SetUp, GUI_RudderZeroSetUp); //Menue, Screen (-1  for nothing)
    }
    else if (ActualScreen == GUI_RudderLimitsSetUp) {
      //Ruder Menue Set Limits
      DisplayShowScreen(Menu_Rudder_Limits_SetUp, GUI_RudderLimitsSetUp); //Menue, Screen (-1  for nothing)
    }
    else if (ActualScreen == GUI_FDWSetUp) {
      //FDW Calib Main screen
      if (LastMenue == Menu_Logge_SetUp) {
        DisplayShowScreen(Menu_Logge_SetUp, GUI_FDWSetUp);
      }
      else if (LastMenue == Menu_Logge_Adjust_SetUp) {
        DisplayShowScreen(Menu_Logge_Adjust_SetUp, GUI_FDWSetUp);
      }
    }
    else if (ActualScreen == GUI_433MHzNMEADebug) {
      DisplayShowScreen(Menu_Debug_NMEA_SetUp, GUI_433MHzNMEADebug);
    }
  }
  /*
  	if (GotDataBlinker == true) {
  		digitalWrite(LED_BUILTIN,true);
  	}
  */
  delay(1);
  /*
  	if (GotDataBlinker == true) {
  		digitalWrite(LED_BUILTIN,false);
  		GotDataBlinker = false;
  	}
  */

#if DEBUG
  if ((MotorStop == true) && (runner % 100 == 0)) {
    Serial.print("MotorDirection: ");
    Serial.print(MotorDirection);
  
    Serial.print(" MotorSetPointDirection: ");
    Serial.print(MotorSetPointDirection);
  
    Serial.print(" GeberRichtung: ");
    Serial.print(GeberRichtung);
  
    Serial.print(" GeberInPercent: ");
    Serial.print(GeberInPercent);
  
    Serial.print(" MotorSetPoint: ");
    Serial.println(MotorSetPoint);
  }
#endif
 
  wdt_reset();

  runner++;
  if (runner >= 10000) {
    runner = 0;
  }
}

