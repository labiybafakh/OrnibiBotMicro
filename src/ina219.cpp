#include "ina219.hpp"

INA219::INA219(){

}

INA219::~INA219(){

}

int16_t INA219::readRegister(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(INA219_ADDRESS, 2);
  
  uint16_t value = Wire.read() << 8;
  value |= Wire.read();
  
  return (int16_t)value;
}

float INA219::getCurrent() {
  int16_t current_raw = readRegister(INA219_REG_CURRENT);
  return current_raw * 0.1;  // Scale the raw value to obtain current in mA
}

float INA219::getBusVoltage() {
  int16_t bus_voltage_raw = readRegister(INA219_REG_BUS_VOLTAGE);
  bus_voltage_raw >>= 3;
  return bus_voltage_raw * 0.004;  // Scale the raw value to obtain voltage in V
}

void INA219::writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDRESS);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}