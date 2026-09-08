#include "modbus.hpp"

// private helpers
// crc16 calc
uint16_t modbus::crc16(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;
    // CRC calc process
    for(uint16_t index = 0; index < len; ++index){
        crc ^= data[index];
        for(uint8_t bit = 0; bit < 8; ++bit){
            if(crc & 0x0001){ crc >>=  1; crc ^= 0xA001; }
            else{ crc >>= 1; }
        }
    }

    return crc;
}

// adjusts the crc for reg send
void modbus::appendCrc(uint8_t *frame, uint16_t len){
    uint16_t crc = crc16(frame, len);
    frame[len] = static_cast<uint8_t>(crc & 0xFF);
    frame[len+1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

// transmit mode set helper
void modbus::setTransmit(){
    HAL_GPIO_WritePin(dePort_, dePin_, GPIO_PIN_SET);
    for(volatile int i = 0; i < 20; ++i){ __NOP(); }
}

// reciv mode set helper
void modbus::setReceive(){
    while(HAL_UART_GetState(uart_) == HAL_UART_STATE_BUSY_TX){}
    HAL_GPIO_WritePin(dePort_, dePin_, GPIO_PIN_RESET);
}

// request validity check
modbus::status modbus::transact(const uint8_t* tx, uint16_t txLen, uint8_t* rx, uint16_t rxCap){
    setTransmit();
    HAL_StatusTypeDef hs = HAL_UART_Transmit(uart_, const_cast<uint8_t*>(tx), txLen, timeoutMs_);

    setReceive();
    if(hs != HAL_OK){ return status::TX_ERROR; }

    // receiv array
    rxLen_ = 0;
    uint32_t start = HAL_GetTick();
    constexpr uint32_t gap = 5; // should solve the delay between drivers issue
    while(HAL_GetTick() - start < timeoutMs_ && rxLen_ < rxCap){
        uint8_t byte;
        if(HAL_UART_Receive(uart_, &byte, 1, 5) == HAL_OK){
            rx[rxLen_++] = byte;
            lastByte = HAL_GetTick();
        } else if(rxLen_ > 0 && (HAL_GetTick() - lastByte) > gap){
            break; // got a full frame hence gap says its done stop waiting
        }

    }

    // shortest modbus fram is 5bytes, less is error or smth went wrong
    if(rxLen_ < 5) return status::TIMEOUT;

    // reconstruct CRC16 from the response end in HIGH bytes and LOW bytes
    uint16_t gotchaCrc = static_cast<uint16_t>(rx[rxLen_ - 2]) | static_cast<uint16_t>(rx[rxLen_ - 1] << 8);

    // just calculation of crc16 fr0m crc16; the last two bytes are crc from signal which we do not need for this do you understand the words coming out of ma mouth 
    uint16_t calcCrc = crc16(rx, rxLen_ - 2);

    // validity check
    if(gotchaCrc != calcCrc) return status::CRC_ERROR;

    return status::OK;
}
// validity of response check
modbus::status modbus::verifyResponse(uint8_t slaveAddr, uint8_t expectedfunc, uint8_t* rx, uint8_t rxLen){
    if(rxLen < 5) return status::TIMEOUT;

    // not the same address
    if(rx[0] != slaveAddr) return status::MISMATCH;

    // exception response; syntax [slaveAddr][func | 0x80][exceptionError][CRClow][CRChigh] total 5 bytes
    if(rx[1] == static_cast<uint8_t>(expectedfunc | 0x80)){
        lastException_ = (rxLen >= 3) ? rx[2] : 0; // during exception response, the size is 5, 
                                                   // in worst case scenario if there is less than 3 bytes it is better to pass it like 0
        return status::EXCEPTION;
    }
    // not the same func
    if(rx[1] != expectedfunc) return status::INVALID_FUNCTION;

    return status::OK;
}

// PRIVATE ENDS HERE

// public
// constructor
modbus::modbus(UART_HandleTypeDef *uart, GPIO_TypeDef *dePort, uint16_t dePin, uint32_t timeoutMs) :
    uart_(uart), dePort_(dePort), dePin_(dePin), timeoutMs_(timeoutMs){
    }

modbus::status modbus::writeSingleRegisters(uint8_t slaveAddr, uint16_t regAddr, uint16_t data){
    // syntax should be [slaveAddr][func][regHigh][regLow][writeDataHigh][writeDataLow][CRC16low][CRC16high] total 8bytes
    
    // fetching the inputs into modbus rtu protocol
    uint8_t req[8]; // request register has total 8bytes as it wass written above
    req[0] = slaveAddr;
    req[1] = static_cast<uint8_t>(functionField::FF_WRITE_SINGLE_REG);
    req[2] = static_cast<uint8_t>((regAddr >> 8) & 0xFF);
    req[3] = static_cast<uint8_t>(regAddr & 0xFF);
    req[4] = static_cast<uint8_t>((data >> 8) & 0xFF);
    req[5] = static_cast<uint8_t>(data & 0xFF);
    appendCrc(req, 6);

    status st = transact(req, 8, rxbuffer_, sizeof(rxbuffer_));
    if(st != status::OK) return st;

    st = verifyResponse(slaveAddr, req[1], rxbuffer_, rxLen_);
    if(st != status::OK) return st;

    for(int i = 2; i < 6; i++){
        if(rxbuffer_[i] != req[i]) return status::MISMATCH;
    }

    return status::OK; 
}


// dont need for my project but why not iguess
// update: i do need it apparently ;D
modbus::status modbus::writeMultipleRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, const uint16_t* data){
    // syntax is [slaveAddr][func][startAddrHIGH][startAddrLOW][quantHIGH][quantLOW][byteCOUNT][Data1High][Data1Low][DataNhigh][DataNlow][CRClow][CRChigh
    if(quantity == 0 || quantity > MAX_REGS) return status::OVERLOAD;

    uint8_t dataIn = static_cast<uint8_t>(2 * quantity);
    uint16_t reqLen_ = 7 + dataIn; // slaveAddr+func+startHigh+startLow+quantHigh+quantLow+byteCOUNT and then the data in excluding CRC

    uint8_t req[reqLen_ + 2];
    req[0] = slaveAddr;
    req[1] = static_cast<uint8_t>(modbus::functionField::FF_WRITE_MULTIPLE_REG);
    req[2] = static_cast<uint8_t>((startAddr >> 8) & 0xFF);
    req[3] = static_cast<uint8_t>(startAddr & 0xFF);
    req[4] = static_cast<uint8_t>((quantity >> 8) & 0xFF);
    req[5] = static_cast<uint8_t>(quantity & 0xFF);
    req[6] = dataIn;

    for(int i = 0; i < quantity; i++){ // fills the data HIGH/LOW 2 bytes into the request register depending on the numbers of quantity
        req[7+i*2] = static_cast<uint8_t>((data[i] >> 8) & 0xFF);
        req[7+i*2+1] = static_cast<uint8_t>(data[i] & 0xFF); 
    }
    appendCrc(req, reqLen_);
    
    status st = transact(req, reqLen_+2, rxbuffer_, sizeof(rxbuffer_));
    if(st != status::OK) return status::CRC_ERROR;

    st = verifyResponse(slaveAddr, req[1], rxbuffer_, rxLen_);
    if(st != status::OK || rxLen_ != 8) return status::MISMATCH;

    for(int i = 2; i < 6; i++){
        if(rxbuffer_[i] != req[i]) return status::MISMATCH; 
    }

    return status::OK; 
}


modbus::status modbus::readHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, readResult& result){
    // syntax should be [slaveAddr][func][regHigh][regLow][quantHigh][quantLow][CRC16low][CRC16high] total 8bytes
    
    result.count = 0;
    if(quantity == 0 || quantity > MAX_REGS) return status::OVERLOAD;

    // fetch
    uint8_t req[8];
    req[0] = slaveAddr;
    req[1] = static_cast<uint8_t>(modbus::functionField::FF_READ_HOLDING_REG);
    req[2] = static_cast<uint8_t>((startAddr >> 8) & 0xFF);
    req[3] = static_cast<uint8_t>(startAddr & 0xFF);
    req[4] = static_cast<uint8_t>((quantity >> 8) & 0xFF);
    req[5] = static_cast<uint8_t>(quantity & 0xFF);
    appendCrc(req, 6);

    status st = transact(req, 8, rxbuffer_, sizeof(rxbuffer_));
    if(st != status::OK) return st;

    st = modbus::verifyResponse(slaveAddr, req[1], rxbuffer_, rxLen_);
    if(st != status::OK) return st;

    // response results in [slaveAddr][func][bytes][dataHigh][dataLow][CRC] -> data is expending depending on quantity at rate 2*quantity
    uint8_t data = rxbuffer_[2];
    if(data != 2*quantity) return status::TX_ERROR;
    if(rxLen_ != static_cast<uint8_t>(3 + data + 2)) return status::TX_ERROR; // slaveAddr+func+bytecount(3 bytes) + data + CRClow + CRChigh(2 bytes)
    
    for(uint16_t i = 0; i < quantity; i++){
        uint16_t high = rxbuffer_[3 + i * 2];
        uint16_t low = rxbuffer_[3 + i * 2 +1];
        result.regs[i] = static_cast<uint16_t>((high << 8) | low);
    }
    result.count = quantity;
    return status::OK;
}
// public ends here, go touch grass