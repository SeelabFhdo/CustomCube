#include <ADXL345.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Credentials.h"

int status = WL_IDLE_STATUS;
WiFiUDP udp;

float q[4];
int redled = 6;
int greenled = 5;
int blueled = 3;
int dimm = 100;
int blink = 3;
int speed = 0;
boolean wait;

int black[3]  = { 0, 0, 0 };
int white[3]  = { 255, 255, 255 };
int red[3]    = { 255, 0, 0 };
int green[3]  = { 0, 255, 0 };
int blue[3]   = { 0, 0, 255 };
int yellow[3] = { 255, 255, 0 };
int dimWhite[3] = { 76, 76, 76 };
int magenta[3] = { 255, 0, 255 };
int orange[3] = { 255, 165, 0 };
int dimgrey[3] = { 105, 105, 105 };
int current[3] = { 0, 0, 0 };

int redVal = black[0];
int grnVal = black[1]; 
int bluVal = black[2];

int prevR = redVal;
int prevG = grnVal;
int prevB = bluVal;

float angles[3];
short currentSide = 0;
short psi = 0;
short psiOld = 0;
short psiDifference = 0;
short theta = 0;
short phi = 0;

#define MESSAGE_CUBE_ACTION_POSITION_CHANGED  0x00
#define MESSAGE_CUBE_ACTION_ROTATED           0x01
#define MESSAGE_CUBE_VALUE_ROTATED_LEFT       0x00
#define MESSAGE_CUBE_VALUE_ROTATED_RIGHT      0x01

int clientPort = 8226;
int serverPort = 8225;
byte serverIPAddress[] = { 192, 168, 42, 20 };

FreeIMU my3IMU = FreeIMU();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize FreeIMU library
  delay(5);
  my3IMU.init();
  delay(5);

  // WiFi shield presence
  if (WiFi.status() == WL_NO_SHIELD) {
    for(;;)
    ;
  }

  // Connect to WLAN
  connectToWLAN();

  // Open UDP socket
  udp.begin(clientPort);
}

void connectToWLAN() {
  short count = 5;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, passphrase);
    Serial.println(status);
    count--;
    if (count == 0) {
      WLANConnectionFailed();
      break;
    }
  }
}

void WLANConnectionFailed() {
  crossFade(red, dimm);
  blinkLED(10);
}

void loop() { 

  // Check WLAN connection
  status = WiFi.status();
  while (status != WL_CONNECTED) {
    Serial.println("Reconnect ...");
    crossFade(black, dimm);
    connectToWLAN();
  }
  
  wait = true;
  
  // Eulersche Winkel abrufen
  //my3IMU.getEuler(angles);
  //toDegrees(angles);
  my3IMU.getQ(q);
  toEuler(angles, q);
  
  psi = (short) angles[0];
  theta = (short) angles[1];
  phi = (short) angles[2];
  
  // Bestimme die Seite, auf der der Wuerfel liegt
  short side = getSide(angles);
  
  // Manipuliere LED, wenn Seite gewechselt wurde
  if (currentSide != side) {
    currentSide = side;
    setLEDColor(currentSide);
    sendMessage(MESSAGE_CUBE_ACTION_POSITION_CHANGED, currentSide, 0x00);
  }
  
  // Erkenne Rotation (wenn die Differenz mindestens 3 Grad betraegt)
  if (((psiOld >= 0) && (psi >= 0)) || ((psiOld < 0) && (psi < 0)) || ((abs(psiOld) < 10) && (abs(psi) < 10))) {
    psiDifference = psiOld - psi;
  } else {
    if (psi < 0) {
      psiDifference = psiOld - (psi + 359);
    } else {
      psiDifference = psiOld - (psi - 359);
    } 
  }
  psiOld = psi;
  if (abs(psiDifference) >= 3) {
      if (psiDifference > 0) {
        sendMessage(MESSAGE_CUBE_ACTION_ROTATED, currentSide, MESSAGE_CUBE_VALUE_ROTATED_RIGHT);
      } else {
        sendMessage(MESSAGE_CUBE_ACTION_ROTATED, currentSide, MESSAGE_CUBE_VALUE_ROTATED_LEFT);
      } 
  }
  
  if(wait) {
    delay(40);
  }
}

void sendMessage(byte action, byte position, byte value) {
  // Data content
  // 3 bit: action
  // 3 bit: position (Position wird stets gesendet)
  // 2 bit: value (Bei Aktion 0 leer)
  
  // [action][position][value]
  // action   = {0, 1, 2} (0 = Wechsel der Position, 1 = Rotation, 2 = Kippbewegung (not implemented)) 
  // position = {1, 2, 3, 4, 5, 6}
  // value (for action 1 only) = {0, 1} (0 = links, 1 = rechts) (+ 2 und 3 für Kippbewegungen nach vorne/hinten)
  
  byte message;
  message = (action << 5);
  message |= (position << 2);
  message |= value;

  Serial.println("Send message ...");
  udp.beginPacket(serverIPAddress, serverPort);
  udp.write(message);
  udp.endPacket();
}

void toEuler(float *angles, float *q) {
  angles[2] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));  // phi
  angles[1] = -asin(2 * (q[0] * q[2] - q[3] * q[1]));  // theta
  angles[0] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));  // psi
  
  // Convert to degrees
  toDegrees(angles);
}

void toDegrees(float *angles) {
  angles[0] *= 180 / M_PI;
  angles[1] *= 180 / M_PI;
  angles[2] *= 180 / M_PI;
}

// Berechne die Seite anhand der eulerschen Winkel
short getSide(float *angles) {
  
  short side = currentSide;
  
//  Serial.print("Phi: ");
//  Serial.print(phi);
//  Serial.print(", Theta: ");
//  Serial.print(theta);
//  Serial.print(", Psi: ");
//  Serial.print(psi);
  
  if (abs(phi) < 10 && abs(theta) < 10) {
    side = 1;
  } else if ((-100 < phi && phi < -80) && (abs(theta) < 10)) {
    side = 2;
  } else if ((100 > phi && phi > 80) && (abs(theta) < 10)) {
    side = 3;
  } else if ((-170 > phi || phi > 170) && (abs(theta) < 10)) {
    side = 4;
  } else if (theta > 70) {  // vorher 65
    side = 5;
  } else if (theta < -70) {  // vorher -65
    side = 6;
  }

  return side;
}

// Würfelfarbe je nach Seite einfärben
void setLEDColor(short side) {
  
  if (side == 1) {
    crossFade(red, dimm); 
  } else if (side == 2) {
    crossFade(blue, dimm);    
  } else if (side == 3) {
    crossFade(green, dimm); 
  } else if (side == 4) {
    crossFade(magenta, dimm);    
  } else if(side == 5) {
    crossFade(yellow, dimm);   
  } else if(side == 6) {
    crossFade(white, dimm);   
  }
}

// LED dimmen
void dimmLED(char* dimmC) {
  
  dimm = atoi(dimmC);

  if (dimm < 0) {
    dimm = 0;
  }
  
  if (dimm > 100) {
    dimm = 100; 
  }

  crossFade(current, dimm);
}

// LED blinken lassen
void blinkLED(int blinkC) {
  
  for (int i = 0; i < blink; i++) {
    analogWrite(redled, 0);  
    analogWrite(greenled, 0);      
    analogWrite(blueled, 0); 
    delay(500 - speed / 2); 
    analogWrite(redled, redVal);   
    analogWrite(greenled, grnVal);      
    analogWrite(blueled, bluVal);
    delay(500 - speed / 2); 
  }
  
  wait = false;
}

// Blinkgeschwindigkeit festlegen
void setBlinkSpeedLED(char* speedC) {
  
  speed = atoi(speedC);
  if (speed > 1000) {
    speed = 1000;
  }
  
  wait = false;
}

// Schrittweite festlegen
int calculateStep(int prevValue, int endValue) {
  
  int step = endValue - prevValue; 
  if (step) {
    step = 1020 / step;              
  }
  
  return step;
}

// Werte berechnen
int calculateVal(int step, int val, int i) {
  
  if ((step) && i % step == 0) { 
    if (step > 0) {             
      val += 1;           
    }  else if (step < 0) {         
      val -= 1;
    } 
  }

  if (val > 255) {
    val = 255;
  } else if (val < 0) {
    val = 0;
  }
  
  return val;
}

void crossFade(int color[3], int dimm) {
  float dimmf = dimm / 100.0;  // Aktueller Dimmfaktor als float festlegen
  
  // Aktuelle Farbwerte merken (ohne Dimmfaktor)
  current[0] = color[0];
  current[1] = color[1];
  current[2] = color[2];
  
  // Schrittweiten fuer die verschiedenen Farben berechnen
  int stepR = calculateStep(prevR, color[0] * dimmf);
  int stepG = calculateStep(prevG, color[1] * dimmf); 
  int stepB = calculateStep(prevB, color[2] * dimmf);
  
  // 1020 Schritte zum Farbwechsel = ca. 1 Sekunde
  for (int i = 0; i <= 1020; i++) {
    //aktuellen Farbwert berechnen
    redVal = calculateVal(stepR, redVal, i);
    grnVal = calculateVal(stepG, grnVal, i);
    bluVal = calculateVal(stepB, bluVal, i);

    // Farbwerte an die LEDs uebergeben
    analogWrite(redled, redVal);   
    analogWrite(greenled, grnVal);      
    analogWrite(blueled, bluVal); 
    delay(1);  // 1ms Verzoegerung
  }
  
  // Aktualisierung der Farbwerte fuer die naechste Aenderung
  prevR = redVal; 
  prevG = grnVal; 
  prevB = bluVal;
  wait = false;  // Keine Pause im loop
}
