
#include "I2cPort.h"

namespace ceoifung
{

    /**
     * @funtion I2cPort()
     */
    I2cPort::I2cPort()
    {
        this->connection_open = false;
    }

    /**
     * @funtion I2cPort(uint8_t bus_addr)
     * @param bus_addr I2C Bus address.
     */
    I2cPort::I2cPort(uint8_t bus_address)
    {
        this->connection_open = false;
        this->bus_address = bus_address;
        this->path = (char *)calloc(PATH_SIZE, sizeof(char));
        sprintf(path, "/dev/i2c-%d", this->bus_address);
    }

    /**
     * @funtion I2cPort(uint8_t dev_addr, uint8_t bus_addr)
     * @param bus_addr I2C Bus address.
     * @param dev_addr Device Address
     */
    I2cPort::I2cPort(uint8_t bus_address, uint8_t device_address)
    {
        this->connection_open = false;
        this->bus_address = bus_address;
        this->path = (char *)calloc(PATH_SIZE, sizeof(char));
        this->device_address = device_address;
        sprintf(path, "/dev/i2c-%d", this->bus_address);
        // printf("path=%s\n", path);
    }

    /** Default Destructor
     * @funtion ~I2cPort()
     *
     */
    I2cPort::~I2cPort()
    {
        free(path);
        printf("析构函数");
        this->closeConnection();
    }

    /**
     * @funtion setBusAddress(uint8_t bus_addr)
     * @param bus_addr I2C Bus address.
     */
    void I2cPort::setBusAddress(uint8_t bus_address)
    {
        // if (this->path != NULL)
        free(path);
        this->bus_address = bus_address;
        this->path = (char *)calloc(PATH_SIZE, sizeof(char));
        sprintf(path, "/dev/i2c-%d", this->bus_address);
    }

    /**
     * @funtion getBusAddress()
     * @return bus_addr I2C Bus address.
     */
    uint8_t I2cPort::getBusAddress() const
    {
        return this->bus_address;
    }

    /**
     * @funtion setDevAddr(uint8_t dev_addr)
     * @param dev_addr Device Address
     */
    void I2cPort::setDeviceAddress(uint8_t device_address)
    {
        this->device_address = device_address;
    }

    /**
     * @funtion getDevAddr()
     * @return dev_addr Device Address
     */
    uint8_t I2cPort::getDeviceAddress() const
    {
        return this->device_address;
    }

    /**
     * @function openConnection()
     * @return file type of int
     */
    int I2cPort::openConnection()
    {
        int file;
        // printf("path=%s\n", path);
        if ((file = open(path, O_RDWR)) < 0)
        {
            printf("%s do not open. Address %d.\n", path, device_address);
            // exit(1);
            return -1;
        }

        if (ioctl(file, I2C_SLAVE, device_address) < 0)
        {
            printf("Can not join I2C Bus. Address %d.\n", device_address);
            // exit(1);
            return -1;
        }

        if (file < 0)
        {
            this->connection_open = false;
            printf("Connection was not established.\n");
            // exit(1);
            return -1;
        }
        this->connection_open = true;
        this->file_descriptor = file;
        // printf
        // std::cout << "Already open " << path << ", Sensor address: " << device_address << std::endl;
        return 0;
    }

    void I2cPort::closeConnection()
    {
        this->connection_open = false;
        close(this->file_descriptor);
        // std::cout << "Already close " << path << ", Sensor address: " << device_address << std::endl;
    }

    /**
     * @function writeBit(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t data, int bitNum)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param data Writing data.
     * @param bitNum Bit Number for writing.
     * @return void.
     */
    void I2cPort::writeBit(uint8_t DATA_REGADD, uint8_t data, uint8_t bitNum)
    {
        int8_t temp = readByte(DATA_REGADD);
        if (data == 0)
        {
            temp = temp & ~(1 << bitNum);
        }
        else if (data == 1)
        {
            temp = temp | (1 << bitNum);
        }
        else
        {
            printf("Value must be 0 or 1! --> Address %d.\n", device_address);
        }

        writeByte(DATA_REGADD, temp);
    }

    /**
     * @function writeBits(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t data, int length, int startBit)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param length Bits length.
     * @param startBit Starting point of the data.
     * @return void.
     */
    void I2cPort::writeMoreBits(uint8_t DATA_REGADD, uint8_t data, uint8_t length,
                                uint8_t startBit)
    {
        int8_t temp = readByte(DATA_REGADD);
        uint8_t bits = 1;
        uint8_t i = 0;

        while (i < length - 1)
        {
            bits = (bits << 1);
            ++bits;
            ++i;
        }

        temp &= ~(bits << startBit);

        temp |= (data << startBit);

        writeByte(DATA_REGADD, temp);
    }

    /**
     * @function writeByte(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t data)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param data Writing data.
     * @return void.
     */
    void I2cPort::writeByte(uint8_t DATA_REGADD, uint8_t data)
    {

        uint8_t buffer[2];

        buffer[0] = DATA_REGADD;
        buffer[1] = data;

        if (write(this->file_descriptor, buffer, 2) != 2)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }
        // printf("write data %d to address %d.\n", data, device_address);
    }

    /**
     * @function writeByteBuffer(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t *data, uint8_t length)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param data Data storage array.
     * @param length Array length.
     * @return void.
     */
    void I2cPort::writeByteBuffer(uint8_t DATA_REGADD, uint8_t *data,
                                  uint8_t length)
    {

        uint8_t buffer[1];
        buffer[0] = DATA_REGADD;

        if (write(this->file_descriptor, buffer, 1) != 1)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }

        if (write(this->file_descriptor, data, length) != length)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }
    }

    /**
     * @function writeByteArduino(uint8_t dev_addr, int8_t data)
     * @param dev_addr Arduino Device Address.
     * @param data Writing data.
     * @return void.
     */
    void I2cPort::writeByteArduino(int8_t data)
    {

        int8_t buffer[1];
        buffer[0] = data;

        if (write(this->file_descriptor, buffer, 1) != 1)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }
    }

    /**
     * @function writeByteBufferArduino(uint8_t dev_addr, uint8_t *data, uint8_t length)
     * @param dev_addr Arduino Device Address.
     * @param data Data storage array.
     * @param length Array length.
     * @return void.
     */
    void I2cPort::writeByteBufferArduino(uint8_t *data, uint8_t length)
    {

        if (write(this->file_descriptor, data, length) != length)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }
    }

    /**
     * @function readBit(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t bitNum)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param bitNum Bit Number for reading.
     * @return uint8_t bit value.
     */

    uint8_t I2cPort::readBit(uint8_t DATA_REGADD, uint8_t bitNum)
    {
        int8_t temp = readByte(DATA_REGADD);
        return (uint8_t)((temp >> bitNum) % 2);
    }

    /**
     * @function readBits(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t length, uint8_t startBit)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param length Bits length.
     * @param startBit Starting point of the value.
     * @return uint8_t bit value.
     */
    uint8_t I2cPort::readMoreBits(uint8_t DATA_REGADD, uint8_t length,
                                  uint8_t startBit)
    {
        int8_t temp = readByte(DATA_REGADD);
        return (uint8_t)((temp >> startBit) % (uint8_t)pow(2, length));
    }

    /**
     * @function readByte(uint8_t dev_addr, uint8_t DATA_REGADD)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @return uint8_t bit value.
     */
    uint8_t I2cPort::readByte(uint8_t DATA_REGADD)
    {

        uint8_t buffer[1];
        buffer[0] = DATA_REGADD;

        if (write(this->file_descriptor, buffer, 1) != 1)
        {
            printf("Can not write data. Address %d.\n", device_address);
        }

        uint8_t value[1];

        if (read(this->file_descriptor, value, 1) != 1)
        {
            printf("Can not read data. Address %d.\n", device_address);
        }

        return value[0];
    }

    /**
     * @function readByteBuffer(uint8_t dev_addr, uint8_t DATA_REGADD, uint8_t *data, uint8_t length)
     * @param dev_addr Device Address.
     * @param DATA_REGADD Data Register Address.
     * @param data Data storage array.
     * @param length Array length.
     * @return void.
     */
    int I2cPort::readByteBuffer(uint8_t DATA_REGADD, uint8_t *data,
                                uint8_t length)
    {

        uint8_t buffer[1];
        buffer[0] = DATA_REGADD;

        if (write(this->file_descriptor, buffer, 1) != 1)
        {
            printf("Can not write data. Address %d.\n", device_address);
            return -1;
        }

        if (read(this->file_descriptor, data, length) != length)
        {
            printf("Can not read data. Address %d.\n", device_address);
            return -1;
        }
        return 0;
    }

    /**
     * @function readByteBufferArduino(uint8_t dev_addr, uint8_t* data, uint8_t length)
     * @param dev_addr Arduino Device Address.
     * @param data Data storage array.
     * @param length Array length.
     * @return void.
     */
    void I2cPort::readByteBufferArduino(uint8_t *data, uint8_t length)
    {

        if (read(this->file_descriptor, data, length) != length)
        {
            printf("Can not read data. Address %d.\n", device_address);
        }
    }

    /**
     * @function readWord(uint8_t dev_addr, uint8_t MSB, uint8_t LSB)
     * @param dev_addr Arduino Device Address.
     * @param MSB 16-bit values Most Significant Byte Address.
     * @param LSB 16-bit values Less Significant Byte Address..
     * @return void.
     */
    int16_t I2cPort::readWord(uint8_t MSB, uint8_t LSB)
    {

        uint8_t msb = readByte(MSB);

        uint8_t lsb = readByte(LSB);

        return ((int16_t)msb << 8) + lsb;
    }

} // namespace ceoifung_i2cport