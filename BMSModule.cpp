#include "config.h"
#include "BMSModule.h"
//#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
  for (int i = 0; i < 32; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;

  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

void BMSModule::clearmodule()
{
  for (int i = 0; i < 32; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  exists = false;
  reset = false;
  moduleAddress = 0;
}

float BMSModule::decodeCellVoltage(int cell, CAN_message_t &msg, int msb, int lsb)
{
  if ((((msg.buf[msb] & 0x0F) << 8) + msg.buf[lsb]) > 0)
  {
    cellVolt[cell] = float((((msg.buf[msb] & 0x0F) << 8) + msg.buf[lsb]) * 0.00125);
  }
}

void BMSModule::decodecan(int Id, CAN_message_t &msg)
{
  if (0x1 < moduleAddress && moduleAddress < 0xC) // handle 8-cell frames
  {
    switch (Id)
    {
    case 0x60:
      decodeCellVoltage(1, msg, 0, 1);
      decodeCellVoltage(2, msg, 2, 3);
      decodeCellVoltage(3, msg, 4, 5);
      decodeCellVoltage(4, msg, 6, 7);
      break;

    case 0x70:
      decodeCellVoltage(5, msg, 0, 1);
      decodeCellVoltage(6, msg, 2, 3);
      decodeCellVoltage(7, msg, 4, 5);
      decodeCellVoltage(8, msg, 6, 7);
      break;

    case 0xE0:
      temperatures[0] = float(((msg.buf[6] << 8) + msg.buf[7]) * -0.0324 + 150);
      break;

    default:
      break;
    }
  }
  else // handle 6-cell frames
  {
    switch (Id)
    {
    case 0x60:
      decodeCellVoltage(1, msg, 0, 1);
      decodeCellVoltage(2, msg, 2, 3);
      decodeCellVoltage(3, msg, 4, 5);
      break;

    case 0x70:
      decodeCellVoltage(4, msg, 0, 1);
      decodeCellVoltage(5, msg, 2, 3);
      decodeCellVoltage(6, msg, 4, 5);
      break;

    case 0xE0:
      temperatures[0] = float(((msg.buf[6] << 8) + msg.buf[7]) * -0.0324 + 150);
      break;

    default:
      break;
    }
  }
  /*
    if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
    if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();
    for (int i = 0; i < 32; i++)
    {
    if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell)
    {
      lowestCellVolt[i] = cellVolt[i];
    }
    if (highestCellVolt[i] < cellVolt[i])
    {
      highestCellVolt[i] = cellVolt[i];
    }
    }
  */
}

int BMSModule::getCellsUsed()
{
  return cellsused;
}

uint8_t BMSModule::getFaults()
{
  return faults;
}

uint8_t BMSModule::getAlerts()
{
  return alerts;
}

uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 32; i++)
    if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell)
      lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 32; i++)
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      if (cellVolt[i] > hiVal)
        hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV()
{
  int x = 0;
  cellsused = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 32; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      x++;
      avgVal += cellVolt[i];
      cellsused = i;
    }
  }

  scells = x;
  avgVal /= x;
  return avgVal;
}

int BMSModule::getscells()
{
  return scells;
}

float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return highestCellVolt[cell];
}

float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 32)
    return 0.0f;
  return lowestCellVolt[cell];
}

float BMSModule::getHighestTemp()
{
  return (temperatures[0]);
}

float BMSModule::getLowestTemp()
{
  return (temperatures[0]);
}

float BMSModule::getLowTemp()
{
  return (temperatures[0]);
}

float BMSModule::getHighTemp()
{
  return (temperatures[0]);
}

float BMSModule::getAvgTemp()
{
  return (temperatures[0]);
}

float BMSModule::getModuleVoltage()
{
  moduleVolt = 0;
  for (int I; I < 32; I++)
  {
    if (cellVolt[I] > IgnoreCell && cellVolt[I] < 60.0)
    {
      moduleVolt = moduleVolt + cellVolt[I];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
  if (temp < 0 || temp > 2)
    return 0.0f;
  return temperatures[temp];
}

void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR)
    return;
  moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
  return moduleAddress;
}

bool BMSModule::isExisting()
{
  return exists;
}

bool BMSModule::isReset()
{
  return reset;
}

void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

void BMSModule::setExists(bool ex)
{
  exists = ex;
}

void BMSModule::setReset(bool ex)
{
  reset = ex;
}

void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
  Serial.print(Ignore);
  Serial.println();
}
