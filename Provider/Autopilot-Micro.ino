#include <ELECHOUSE_CC1101.h>
#include <Wire.h>
#include <FastCRC.h>
#include <avr/wdt.h>

#define SDA_PORT PORTC
#define SDA_PIN 7
#define SCL_PORT PORTD
#define SCL_PIN 6

#include <SoftI2CMaster.h>

FastCRC16 CRC16;

#define CRC_SW 0

//#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define CMPS11_ADDRESS 0xC0
#define ANGLE_8  1           // Register to read 8bit angle from

//Windmesser
#define TastMin   15.5
#define TastMax  83.5
#define LastValidWinDataOverflow 1000
#define numReadingsWind 16

//Logge
#define LastValidFDWDataOverflow 1000
#define numReadingsFDW 16
#define numReadingsGeber 3

#define SERIAL_BUFFER_SIZE 256

#define NMEATimeout 10000

//CRC
uint32_t CRC;
//char CRChex[10];

//Rudergeber
const byte Ruder1     = A1;
const byte Ruder2     = A0;

//Windgeber
const byte WindPinA = 7;
const byte WindPinB = 6;

byte stateA = LOW;
byte stateB = LOW;
byte stateLastA = LOW;

volatile byte WindDir = LOW;
volatile byte DataValid = LOW;
volatile byte NewWindDataReady = LOW;

unsigned long PulseLenghtLogge = 0;
unsigned long PulseLenghtLoggeStart = micros();

//unsigned long DirDelay;
unsigned long PulseLenghtWind = 0;
unsigned long PulseLenghtWindStart = micros();
unsigned long PulseLenghtWindHighTime = 0;
unsigned long PulseLenghtWindHighTimeStart = 0;
byte DirStart = LOW;
float DirTastDelta;

int readIndexWind = 0;  

unsigned int WindDirEimer[numReadingsWind];   
unsigned long TotalWindDir = 0;                  
unsigned int DirInGrad = 0;
unsigned int AverageDirInGrad = 0;

unsigned int WindGeschwEimer[numReadingsWind];   
unsigned long TotalWindGeschwEimer = 0;                  
unsigned int WindGeschwindigkeit = 0;
unsigned int AverageWindGeschwindigkeit = 0;

unsigned long LastValidWinData = millis();                  

unsigned int WindGeschwindigkeitInt = 0;
int WindRichtungInt = 0;
byte WindGeschwindigkeitStr[8] = "";
byte WindRichtung[8] = "";


//Geber
int readIndexGeber = 0;  

int Geber1Eimer[numReadingsGeber];   
int Geber2Eimer[numReadingsGeber];   
long TotalGeber1 = 0;                  
long TotalGeber2 = 0;                  
int AverageGeber1 = 0;
int AverageGeber2 = 0;



//Kompass
unsigned char high_byte, low_byte, angle8;
unsigned int angle16;
byte kurs[8] = "";

//Ruder
int Geber1Int = 0;
int Geber2Int = 0;
byte Geber1[8] = "";
byte Geber2[8] = "";

//Logge
int FDWInt;
byte FDW[8] = "";

unsigned int FDWEimer[numReadingsFDW];   
unsigned int FDWRead = 0;
unsigned long TotalFDWEimer = 0;                  
unsigned int AverageFDW = 0;

volatile byte NewFDWDataReady = LOW;

int readIndexFDW = 0;  

unsigned long LastValidFDWData = millis();                  

const byte LoggePin = 3;


//Seriell
boolean NMEAstringComplete = false;  // whether the string is complete
char NMEA[81];
int NMEAcounter = 0;
char CheckINSumNMEA[3];
char CheckOUTSumNMEA[3];
char NMEAinChar;
int f, g, c, XOR;
byte NMEAPackageType = 0;
int NMEACourseActual = 0;
int NMEACourseToSteer = 0;
byte NMEACPNPilotActive = 0;
int NMEADistanceToWP = 0;
int NMEASOG = 0;
int NMEANextWP = 0;
byte NMEAToken;
String NEMAtemppch = "";

byte NMEACourseActualChar[8] = "";
byte NMEACourseToSteerChar[8] = "";
byte NMEACPNPilotActiveChar[3] = "";
byte NMEADistanceToWPChar[8] = "";
byte NMEASOGChar[8] = "";
byte NMEANextWPChar[8] = "";

unsigned long NMEARMCLastRecived = millis();
unsigned long NMEARMBLastRecived = millis();

//Transiver
const int n = 256;
byte buffer[n] = "";

//runner
int i = 0;
unsigned int r = 0;
byte ret;
byte SendLen = 0;


void serialEventRun(void) {
  if (Serial1.available()) serialEvent();
}
 
void serialEvent() {
	if (Serial1.available()) {
		while (Serial1.available()) {
			char NMEAinChar = (char)Serial1.read();
			if (NMEAinChar == '\n') {
				NMEA[NMEAcounter] = '\0';
				NMEAstringComplete = true;
			} 
			else {
				NMEA[NMEAcounter] = NMEAinChar;
				NMEAcounter++;
			}
		}
		if (NMEAstringComplete == true) {
			sprintf(CheckOUTSumNMEA,"%02X",getCheckSum(NMEA));
			for (g = -1, i = 0; i < sizeof(NMEA); i++) {
				if (NMEA[i] == '*') {
					g = 0;
				}
				else if (g > -1) {
					CheckINSumNMEA[g] = NMEA[i];
					g++;
				}
				if (g > 1) {
					break;
				}
			}			
			if (strcmp(CheckINSumNMEA, CheckOUTSumNMEA) == 0) {
				NMEAToken = 0;
				NEMAtemppch = strtok(NMEA, ",");
				NMEAPackageType = 0;
				
				while (NEMAtemppch != NULL)
				{
					switch (NMEAToken) {
						case 0:
							if (NEMAtemppch == "$GPRMC") {
								NMEARMCLastRecived = millis();
								NMEAPackageType = 1;
								if (NMEACPNPilotActive > 0) {
									NMEACPNPilotActive--;
								}
							}
							else if (NEMAtemppch == "$ECRMC") {
								NMEARMCLastRecived = millis();
								NMEAPackageType = 2;
								NMEACPNPilotActive = 10;
							} 
							else if (NEMAtemppch == "$ECRMB") {
								NMEARMBLastRecived = millis();
								NMEAPackageType = 3;
								NMEACPNPilotActive = 10;
							}
							break;
						case 5:
							if (NMEAPackageType == 3) {
								NMEANextWP = (int)NEMAtemppch.toInt();
							}
							break;
						case 7:
							if ((NMEAPackageType == 1) || (NMEAPackageType == 2)) { //GPRMC, ECRMC
								NMEASOG = (int)(NEMAtemppch.toFloat() * (float)10);
							}
							break;
						case 8:
							if ((NMEAPackageType == 1) || (NMEAPackageType == 2)) { //GPRMC, ECRMC
								NMEACourseActual = (int)(NEMAtemppch.toFloat() * (float)10);
							}
							break;
						case 10:
							if (NMEAPackageType == 3) { 
								NMEADistanceToWP = (int)(NEMAtemppch.toFloat() * (float)100);
							}
							break;
						case 11:
							if (NMEAPackageType == 3) { 
								NMEACourseToSteer = (int)(NEMAtemppch.toFloat() * (float)10);
							}
							break;
						default:
							break;
					}
					NEMAtemppch = strtok (NULL, ",");
					NMEAToken++;
				}
			}		
			NMEAcounter = 0;
			NMEAstringComplete = false;
		}
  }	
} 

void setup() {
//debug
//  pinMode(10, OUTPUT);

	for(i=0;i<readIndexWind;i++) {
		WindDirEimer[i] = 0;
		WindGeschwEimer[i] = 0;
	}
	
	for(i=0;i<readIndexFDW;i++) {
		FDWEimer[i] = 0;
	}

	for(i=0;i<readIndexGeber;i++) {
		Geber1Eimer[i] = 0;
		Geber2Eimer[i] = 0;
	}
	
	
  //Windgeber
  pinMode(WindPinA, INPUT);
  pinMode(WindPinB, INPUT);
  pinMode(LoggePin, INPUT);

  PulseLenghtWind = micros();
	PulseLenghtLogge = micros();

  analogReference(EXTERNAL);
  Wire.begin();
  ELECHOUSE_cc1101.Init(F_433); // set frequency - F_433, F_868, F_965 MHz

  Serial1.begin(9600);
//  Serial.begin(57600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB
  }	
	
	i2c_init();

	attachInterrupt(digitalPinToInterrupt(WindPinA), windinterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LoggePin), loggeinterrupt, FALLING);

  wdt_enable(WDTO_2S);

}

void loop()
{
  //Daten vom Kompass lesen
  i2c_start(CMPS11_ADDRESS | I2C_WRITE);
  i2c_write(ANGLE_8);
  i2c_rep_start(CMPS11_ADDRESS | I2C_READ);
  angle8 = i2c_read(false);
  high_byte = i2c_read(false);
  low_byte = i2c_read(true);
  i2c_stop();
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;

 // AverdageMagKursData();

  //Daten vom Ruderlagegeber holen
  Geber1Int = analogRead(Ruder1);
  Geber2Int = analogRead(Ruder2);

  AverageGeber();

  //Winddaten
  if (NewWindDataReady == true) {
    
    WindRichtungInt = (int)AverageDirInGrad;
    if (WindDir == 1) {
      WindRichtungInt = WindRichtungInt * (-1);
    }
    WindGeschwindigkeitInt = AverageWindGeschwindigkeit;
    NewWindDataReady = false;
    LastValidWinData = millis();                  
  }
  else if (millis() > (LastValidWinData + LastValidWinDataOverflow)) {
    DirInGrad = (unsigned int)round(AverageDirInGrad);
    WindGeschwindigkeit = 0;
    AverdageWindData();
  }

  //Logge
  if (NewFDWDataReady == true) {
    FDWInt = AverageFDW;
    NewFDWDataReady = false;
    LastValidFDWData = millis();                  
  }
  else if (millis() > (LastValidFDWData + LastValidFDWDataOverflow)) {
    FDWRead = 0;
    AverdageFDWData();
  }


	if ((millis() > (NMEARMBLastRecived + NMEATimeout)) || (millis() > (NMEARMCLastRecived + NMEATimeout))) {
		NMEACPNPilotActive = 0;
	} 
	if (millis() < NMEARMBLastRecived) {
		NMEARMBLastRecived = millis();
	}
	if (millis() < NMEARMCLastRecived) {
		NMEARMCLastRecived = millis();
	}
	
	if (NMEANextWP < 0) {
		NMEANextWP = -1;
	}
	if (NMEASOG < 0) {
		NMEASOG = -1;
	}
	if (NMEADistanceToWP < 0) {
		NMEADistanceToWP = -1;
	}
	if (NMEACourseToSteer < 0) {
		NMEACourseToSteer = -1;
	}
	if (NMEACourseActual < 0) {
		NMEACourseActual = -1;
	}
	
	byte NMEACourseActualChar[8] = "";
	byte NMEACourseToSteerChar[8] = "";
	byte NMEACPNPilotActiveChar[3] = "";
	byte NMEADistanceToWPChar[8] = "";
	byte NMEASOGChar[8] = "";
	byte NMEANextWPChar[8] = "";

/*
	//anti crash stuff	
	if (AverageGeber1 > 1024) {
		asm volatile ("  jmp 0");
	}
	if (AverageGeber2 > 1024) {
		asm volatile ("  jmp 0");
	}
	if (angle16 > 3600) {
		asm volatile ("  jmp 0");
	}
	if (WindRichtungInt > 180) {
		asm volatile ("  jmp 0");
	}
	if (WindRichtungInt < -180) {
		asm volatile ("  jmp 0");
	}
	if (AverageFDW > 2000) {
		asm volatile ("  jmp 0");
	}
	if (AverageWindGeschwindigkeit > 9999) {
		asm volatile ("  jmp 0");
	}
*/
	
  //Senden
  SendLen = LEnOfInt(NMEANextWP) + LEnOfInt(NMEASOG) + LEnOfInt(NMEADistanceToWP) + LEnOfInt(NMEACPNPilotActive) + LEnOfInt(NMEACourseToSteer) + LEnOfInt(NMEACourseActual) + LEnOfInt(angle16) + LEnOfInt(AverageGeber1) + LEnOfInt(AverageGeber2) + LEnOfInt(AverageWindGeschwindigkeit) + LEnOfInt(WindRichtungInt) + LEnOfInt(AverageFDW) + 17;
 
  dtostrf(angle16, LEnOfInt(angle16), 0, kurs);
  dtostrf(AverageGeber1, LEnOfInt(AverageGeber1), 0, Geber1);
  dtostrf(AverageGeber2, LEnOfInt(AverageGeber2), 0, Geber2);
  //durch 100 und in float casten
  dtostrf(AverageWindGeschwindigkeit, LEnOfInt(AverageWindGeschwindigkeit), 0, WindGeschwindigkeitStr);
  //negative = andere Richtung
  dtostrf(WindRichtungInt, LEnOfInt(WindRichtungInt), 0, WindRichtung);
  dtostrf(AverageFDW, LEnOfInt(AverageFDW), 0, FDW);

	dtostrf(NMEACourseActual, LEnOfInt(NMEACourseActual), 0, NMEACourseActualChar);
	dtostrf(NMEACourseToSteer, LEnOfInt(NMEACourseToSteer), 0, NMEACourseToSteerChar);
	dtostrf(NMEACPNPilotActive, LEnOfInt(NMEACPNPilotActive), 0, NMEACPNPilotActiveChar);
	dtostrf(NMEADistanceToWP, LEnOfInt(NMEADistanceToWP), 0, NMEADistanceToWPChar);
	dtostrf(NMEASOG, LEnOfInt(NMEASOG), 0, NMEASOGChar);
	dtostrf(NMEANextWP, LEnOfInt(NMEANextWP), 0, NMEANextWPChar);

	
  memset(buffer,0,sizeof(buffer));
  sprintf(buffer,"%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s",kurs,Geber1,Geber2,WindGeschwindigkeitStr,WindRichtung,FDW,NMEACPNPilotActiveChar,NMEACourseActualChar,NMEACourseToSteerChar,NMEADistanceToWPChar,NMEANextWPChar,NMEASOGChar);
  CRC = CRC16.ccitt(buffer, sizeof(buffer));
  sprintf(buffer,"%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%s~%lu",kurs,Geber1,Geber2,WindGeschwindigkeitStr,WindRichtung,FDW,NMEACPNPilotActiveChar,NMEACourseActualChar,NMEACourseToSteerChar,NMEADistanceToWPChar,NMEANextWPChar,NMEASOGChar,CRC);

  
	if (r%20 == 0) {
		ELECHOUSE_cc1101.SendData(buffer, SendLen);
//		Serial.println((char *)buffer);
//		digitalWrite(10,!digitalRead(10));
	}

  r++; 
	if (r > 50000) {
		r = 0;
	}
  // Short delay before next loop
  // Setze Watchdog Zähler zurück
  wdt_reset();
}

/* 
 * was: 
 *1 = kurs
 *2 = Geber1
 *3 = Geber2
 *4 = Windrichtung
 *5 = Windgeschwindigkeit
 *6 = FDW
 */ 



byte LEnOfInt(int var) {
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

void loggeinterrupt(void){
 
  detachInterrupt(digitalPinToInterrupt(LoggePin));

  if (micros() > PulseLenghtLoggeStart) {
    PulseLenghtLogge = micros() - PulseLenghtLoggeStart;
    NewDataLogge();
  }
  PulseLenghtLoggeStart = micros();

  attachInterrupt(digitalPinToInterrupt(LoggePin), loggeinterrupt, FALLING);

}

void windinterrupt(void) {
  
  detachInterrupt(digitalPinToInterrupt(WindPinA));
  
  stateA = digitalRead(WindPinA);
  stateB = digitalRead(WindPinB);

  if (stateA == true) {
    if (micros() <  PulseLenghtWindStart) {
      DataValid = false;
      WindGeschwindigkeit = 0;
      NewWindDataReady = true;
    }
    else {
      DataValid = true;
    }
    PulseLenghtWind = micros() - PulseLenghtWindStart;
    PulseLenghtWindStart = micros();
    PulseLenghtWindHighTimeStart = PulseLenghtWindStart;
  } 
  else {
    PulseLenghtWindHighTime = micros() - PulseLenghtWindHighTimeStart;
  }
  if ((stateA == true) && (stateB == false) && (DataValid == true)) {
    NewDataWind(1); //1 == Rechts
  } 
  else if ((stateA == true) && (stateB == true) && (DataValid == true)) {
    NewDataWind(0); //0 == Links
  }
  stateLastA = stateA;

  attachInterrupt(digitalPinToInterrupt(WindPinA), windinterrupt, CHANGE);
  
}

void NewDataWind(byte Dir) {
  DirTastDelta = round(((float)PulseLenghtWindHighTime/(float)PulseLenghtWind)*(float)1000)/(float)10;
  WindDir = Dir;
  DirInGrad = round(((float)180 / (TastMin - TastMax)) * (DirTastDelta - TastMax));
  WindGeschwindigkeit = round(1/(float)PulseLenghtWind * 1000000.0 * 61.0);
  if ((DirInGrad < 200) && (WindGeschwindigkeit < 10000)) {
    if (DirInGrad > 180) {
      DirInGrad = 180;
    } 
    else if (DirInGrad < 0) {
      DirInGrad = 0;
    }
    AverdageWindData();
  }
}

void AverdageWindData(void) {
  TotalWindDir = TotalWindDir - WindDirEimer[readIndexWind];
  WindDirEimer[readIndexWind] = DirInGrad;
  TotalWindDir = TotalWindDir + WindDirEimer[readIndexWind];
  AverageDirInGrad = round((float)TotalWindDir / (float)numReadingsWind);

  TotalWindGeschwEimer = TotalWindGeschwEimer - WindGeschwEimer[readIndexWind];
  WindGeschwEimer[readIndexWind] = WindGeschwindigkeit;
  TotalWindGeschwEimer = TotalWindGeschwEimer + WindGeschwEimer[readIndexWind];
  AverageWindGeschwindigkeit = round((float)TotalWindGeschwEimer / (float)numReadingsWind);

  readIndexWind = readIndexWind + 1;
  if (readIndexWind >= numReadingsWind) {
    readIndexWind = 0;
  }	
  NewWindDataReady = true;
}

void NewDataLogge(void) {
  FDWRead = round(1.0/(float)PulseLenghtLogge * 1000000.0 * 61.0);
  if (FDWRead < 1000) {
    AverdageFDWData();
  }
}

void AverdageFDWData(void) {
  TotalFDWEimer = TotalFDWEimer - FDWEimer[readIndexFDW];
  FDWEimer[readIndexFDW] = FDWRead;
  TotalFDWEimer = TotalFDWEimer + FDWEimer[readIndexFDW];
  AverageFDW = round((float)TotalFDWEimer / (float)numReadingsFDW);  

  readIndexFDW = readIndexFDW + 1;
  if (readIndexFDW >= numReadingsFDW) {
    readIndexFDW = 0;
  }
  NewFDWDataReady = true;
}

void AverageGeber(void) {
  TotalGeber1 = TotalGeber1 - Geber1Eimer[readIndexGeber];
  Geber1Eimer[readIndexGeber] = Geber1Int;
  TotalGeber1 = TotalGeber1 + Geber1Eimer[readIndexGeber];
  AverageGeber1 = round((float)TotalGeber1 / (float)numReadingsGeber);  

  TotalGeber2 = TotalGeber2 - Geber2Eimer[readIndexGeber];
  Geber2Eimer[readIndexGeber] = Geber2Int;
  TotalGeber2 = TotalGeber2 + Geber2Eimer[readIndexGeber];
  AverageGeber2 = round((float)TotalGeber2 / (float)numReadingsGeber);  


  readIndexGeber = readIndexGeber + 1;
  if (readIndexGeber >= numReadingsGeber) {
    readIndexGeber = 0;
  }  
}

//NMEA Checksumme berechnen
int getCheckSum(String s) {
	  for (XOR = 0, f = 0; f < s.length(); f++) {
    c = (unsigned char)s.charAt(f);
    if (c == '*') break;
    if ((c!='$') && (c!='!')) XOR ^= c;
  }
  return XOR; 
}
