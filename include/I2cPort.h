#ifndef __I2CPORT_H__
#define __I2CPORT_H__

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>


#define PATH_SIZE 15

namespace ceoifung {

    class I2cPort {
    public:

        I2cPort();

        I2cPort(uint8_t bus_address);

        I2cPort(uint8_t device_address, uint8_t bus_address);

        ~I2cPort();

        int openConnection();

        void closeConnection();


        bool isConnectionOpen() const {
            return connection_open;
        }

        void setBusAddress(uint8_t bus_address);

        uint8_t getBusAddress() const;

        void setDeviceAddress(uint8_t device_address);

        uint8_t getDeviceAddress() const;

        void writeBit(uint8_t DATA_REGADD, uint8_t data, uint8_t bitNum);

        void writeMoreBits(uint8_t DATA_REGADD, uint8_t data, uint8_t length,
                           uint8_t startBit);

        void writeByte(uint8_t DATA_REGADD, uint8_t data);

        void writeByteBuffer(uint8_t DATA_REGADD, uint8_t *data, uint8_t length);

        void writeByteArduino(int8_t data);

        void writeByteBufferArduino(uint8_t *data, uint8_t length);

        uint8_t readBit(uint8_t DATA_REGADD, uint8_t bitNum);

        uint8_t readMoreBits(uint8_t DATA_REGADD, uint8_t length, uint8_t startBit);

        uint8_t readByte(uint8_t DATA_REGADD);

        int readByteBuffer(uint8_t DATA_REGADD, uint8_t *data, uint8_t length);

        void readByteBufferArduino(uint8_t *data, uint8_t length);

        int16_t readWord(uint8_t MSB, uint8_t LSB);



    private:
        int file_descriptor;
        uint8_t bus_address;
        uint8_t device_address;
        char *path;
        bool connection_open;

    };
}  // namespace cacaosd_i2cport
#endif //DIDI_I2CPORT_H