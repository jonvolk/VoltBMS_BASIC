#pragma once
#include <FlexCAN.h>

class BMSModule
{
public:
  BMSModule();
  void decodecan(int Id, CAN_message_t &msg);
  void readStatus();
  void clearmodule();
  int getscells();
  float getCellVoltage(int cell);
  float getLowCellV();
  float getHighCellV();
  float getAverageV();
  float getLowTemp();
  float getHighTemp();
  float getHighestModuleVolt();
  float getLowestModuleVolt();
  float getHighestCellVolt(int cell);
  float getLowestCellVolt(int cell);
  float getHighestTemp();
  float getLowestTemp();
  float getAvgTemp();
  float getModuleVoltage();
  float getTemperature(int temp);
  uint8_t getFaults();
  uint8_t getAlerts();
  uint8_t getCOVCells();
  uint8_t getCUVCells();
  void setAddress(int newAddr);
  int getAddress();
  bool isExisting();
  bool isReset();
  void setReset(bool ex);
  void setExists(bool ex);
  void settempsensor(int tempsensor);
  void setIgnoreCell(float Ignore);
  int getCellsUsed();

private:
  float cellVolt[33]; // calculated as 16 bit value * 6.250 / 16383 = volts
  float lowestCellVolt[33];
  float highestCellVolt[33];
  float moduleVolt;      // calculated as 16 bit value * 33.333 / 16383 = volts
  float temperatures[5]; // Don't know the proper scaling at this point
  float lowestTemperature;
  float highestTemperature;
  float lowestModuleVolt;
  float highestModuleVolt;
  float IgnoreCell;
  bool exists;
  bool reset;
  int alerts;
  int faults;
  int COVFaults;
  int CUVFaults;
  int sensor;
  uint8_t moduleAddress; //1 to 0x3E
  int scells;
  int balstat;
  int cellsused;
  float decodeCellVoltage(int cell, CAN_message_t &msg, int msb, int lsb);
};
