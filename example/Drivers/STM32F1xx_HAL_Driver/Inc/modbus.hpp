#pragma once

// Includes
#include <stdint.h>
#include "stm32f1xx_hal.h"

// main studajf poahfowi
class modbus {
    public:
        // status basis for modbus connection
        enum class status : uint8_t {
            OK = 0,
            TIMEOUT,
            CRC_ERROR,
            INVALID_FUNCTION,
            OVERLOAD,
            MISMATCH,
            TX_ERROR,
            RX_ERROR,
            EXCEPTION
        };

    private:
        // Initilized values
        UART_HandleTypeDef *uart_;
        GPIO_TypeDef *dePort_;
        uint16_t dePin_;
        uint32_t timeoutMs_;
        uint8_t rxbuffer_[64]; // have more than enough so no overflowing
        uint8_t rxLen_ = 0;
        uint8_t lastException_ = 0; // for validity check later
        
        // Function Codes
        enum functionField : uint8_t { 
            FF_READ_COILS = 0x01,        // (ball ball ball) My granny called,
            FF_READ_HOLDING_REG = 0x03,  // she said, "Travvy, you work too hard
            FF_READ_INPUT_REG = 0x04,    // I'm worried you forget about me"
            FF_WRITE_SINGLE_COIL = 0x05, // I'm fallin' in and out of clouds
            FF_WRITE_SINGLE_REG = 0x06,  // Don't worry, I'ma get it, Granny, uh
            FF_WRITE_MULTIPLE_COILS = 0x0F,
            FF_WRITE_MULTIPLE_REG = 0x10,
            FF_READ_WRITE_MULTIPLE_REG = 0x17
        };

        uint16_t crc16(const uint8_t *data, uint16_t len);

        // adjusts the crc for reg send
        void appendCrc(uint8_t *frame, uint16_t len);

        // trasmit 
        void setTransmit();

        // reciv
        void setReceive();

        // transact sends data to wires, receive the response, checks the validity, and returns the ok status if request went right
        status transact(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t rxCap);
        
        status verifyResponse(uint8_t slaveAddr, uint8_t expectedfunc, uint8_t* rx, uint8_t rxLen);

    public:
        // constructor
        modbus(UART_HandleTypeDef *uart, GPIO_TypeDef *dePort, uint16_t dePin, uint32_t timeoutMs);

        // some declarations for further verification in reading, and maybe writing
        // some room for expantion for later ig
        static constexpr int MAX_REGS = 29; // the maximum possible register count per 64byte buffere
                                            // possibly it is best to change it to sizeof((rxbuffer - 5(slaveAddr+func+bytecount+CRClow+CRChigh))/2) for dynamic changes
        struct readResult{
            uint16_t regs[MAX_REGS];
            uint16_t count; // how many of regs[] are valid
        };

        status readHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, readResult& result);

        status writeSingleRegisters(uint8_t slaveAddr, uint16_t regAddr, uint16_t data);

        status writeMultipleRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint16_t* data);
};
