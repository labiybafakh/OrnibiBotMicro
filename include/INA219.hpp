#ifndef INA219_HPP
#define INA219_HPP

#include <Arduino.h>
#include <Wire.h>

// I2C address of the INA219 sensor
#define INA219_ADDRESS 0x40

// INA219 register addresses
#define INA219_REG_CONFIG 0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

class INA219{
    private:
        uint8_t _address;
        TwoWire *_wire;
    public:
        INA219(const uint8_t device_address, TwoWire *wire);
        ~INA219();
        bool begin();
        bool isConnected();
        volatile int16_t readRegister(uint8_t reg);
        volatile float getCurrent();
        volatile float getBusVoltage();
        void writeRegister(uint8_t reg, uint16_t value);

};


#endif