#include <SPI.h>
#include <Ethernet.h>
#include "MgsModbus.h"

MgsModbus Mb;
int inByte;

byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5};
IPAddress ip(192, 168, 2, 2);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

bool StartStop, Conveyor1Coil, Conveyor2Coil, CrusherCoil, GateCoil;
bool Fault, LevelSensor;
int Conveyor1Freq, Conveyor2Freq, CrusherFreq;

void setup()
{
  StartStop = 0;
  Conveyor1Coil = 0;
  Conveyor2Coil = 0;
  CrusherCoil = 0;
  GateCoil = 0;
  Conveyor1Freq = 0;
  Conveyor2Freq = 0;
  CrusherFreq = 0;

  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Serial interface started");

  Ethernet.begin(mac, ip, gateway, subnet);
  Serial.println("Ethernet interface started");

  // Initialize Modbus data registers
  for (int i = 0; i < 8; i++)
  {
    Mb.MbData[i] = 0;
  }

  Serial.println("0 - Print the Holding registers");
}

void loop()
{
  Conveyor1Freq = Mb.MbData[1];
  Conveyor2Freq = Mb.MbData[2];
  CrusherFreq = Mb.MbData[3];

  StartStop =     (Mb.MbData[0] >> 0) & 1;
  Conveyor1Coil = (Mb.MbData[0] >> 1) & 1;
  Conveyor2Coil = (Mb.MbData[0] >> 2) & 1;
  CrusherCoil =   (Mb.MbData[0] >> 3) & 1;
  GateCoil =      (Mb.MbData[0] >> 4) & 1;

  LevelSensor = digitalRead(A0);
  LevelSensor = !LevelSensor; // Simplified inversion

  Fault = digitalRead(A1);
  Fault = !Fault; // Simplified inversion

  // Update Modbus data register for sensors
  if (Fault)
    Mb.MbData[4] |= (1 << 0);
  else
    Mb.MbData[4] &= ~(1 << 0);

  if (LevelSensor)
    Mb.MbData[4] |= (1 << 1);
  else
    Mb.MbData[4] &= ~(1 << 1);

  // Control actuators based on StartStop status
  if (StartStop)
  {
    digitalWrite(2, !Conveyor1Coil);
    digitalWrite(4, !Conveyor2Coil);
    digitalWrite(7, !CrusherCoil);
    digitalWrite(8, !GateCoil);
  }
  else
  {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // Serial interface for debugging
  if (Serial.available() > 0)
  {
    inByte = Serial.read();

    if (inByte == '0')
    {
      for (int i = 1; i < 4; i++)
      {
        Serial.print("Motor number ");
        Serial.print(i);
        Serial.print(" Frequency value: ");
        Serial.println(Mb.MbData[i]);
      }
      Serial.print("Actuators word ");
      Serial.println(Mb.MbData[0], BIN);
      Serial.print("Sensors word ");
      Serial.println(Mb.MbData[4], BIN);
      Serial.println("0 - Print the Holding registers");
    }
  }

  Mb.MbsRun();
}
