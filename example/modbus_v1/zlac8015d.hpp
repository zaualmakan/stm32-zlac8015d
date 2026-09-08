#pragma once

#include "modbus.hpp"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string>
#include <array>

class zlac8015d {
    private:
        // initializasd values
        modbus& mb_;

        // ---- Register Address ----
        // Common
        static const uint16_t CONTROL_REG = 0x200E;
        static const uint16_t OPR_MODE = 0x200D;
        static const uint16_t L_ACL_TIME = 0x2080;
        static const uint16_t R_ACL_TIME = 0x2081;
        static const uint16_t L_DCL_TIME = 0x2082;
        static const uint16_t R_DCL_TIME = 0x2083;

        // Velocity Control
        static const uint16_t L_CMD_RPM = 0x2088;
        static const uint16_t R_CMD_RPM = 0x2089;
        static const uint16_t L_FB_RPM = 0x20AB;
        static const uint16_t R_FB_RPM = 0x20AC;

        // Position Control
        static const uint16_t POS_CONTROL_TYPE = 0x200F;
        
        static const uint16_t L_MAX_RPM_POS = 0x208E;
        static const uint16_t R_MAX_RPM_POS = 0x208F;

        static const uint16_t L_CMD_REL_POS_HI = 0x208A;
        static const uint16_t L_CMD_REL_POS_LO = 0x208B;
        static const uint16_t R_CMD_REL_POS_HI = 0x208C;
        static const uint16_t R_CMD_REL_POS_LO = 0x208D;

        static const uint16_t L_FB_POS_HI = 0x20A7;
        static const uint16_t L_FB_POS_LO = 0x20A8;
        static const uint16_t R_FB_POS_HI = 0x20A9;
        static const uint16_t R_FB_POS_LO = 0X20AA;

        // Troubleshooting
        static const uint16_t L_FAULT = 0x20A5;
        static const uint16_t R_FAULT = 0x20A6;

        // ---- Control CMDs (REG) ----
        static const uint8_t EMER_STOP = 0x05;
        static const uint8_t ALRM_CLR = 0x06;
        static const uint8_t DOWN_TIME = 0x07;
        static const uint8_t ENABLE = 0x08;
        static const uint8_t POS_SYNC = 0x10;
        static const uint8_t POS_L_START = 0x11;
        static const uint8_t POS_R_START = 0x12;
    
        // ---- Odometry ----
        static const int travel_in_one_rev = 0.638; // meters morales
        static const int cpr = 16385;
        static const int R_wheel = 0.105; // meters -- no miles
        static constexpr int16_t rpm_limit = 3000;
        static constexpr double pi = 3.14159;

        // ---- Fault Codes ----             |
        enum faultCodes : uint16_t {
            NO_FAULT = 0x0000,
            OVER_VOLT = 0x0001,
            UNDER_VOLT = 0x002,
            OVER_CURR = 0x0004,
            OVER_LOAD = 0x0008,
            CURR_OUT_TOL = 0x0010,
            ENCOD_OUT_TOL = 0x0020,
            MOTOR_BAD = 0x0040,
            REF_VOLT_ERROR = 0x080,
            EEPROM_ERROR = 0x0100,
            WALL_ERROR = 0x0200,
            HIGH_TEMP = 0x0400
        };

        // int16 to wire; just a reinterpritation for wires:l
        static uint16_t int16ToRegister(int16_t value);
    public:
        // ---- Operation Mode ----
        enum class opMode : int {
            POS_REL_CONTROL = 1,
            POS_ABS_CONTROL = 2,
            VEL_CONTROL = 3,
            FAIL = 0
        };

        enum class sync : int {
            ASYNC = 0,
            SYNC = 1
        };

        // i have three drivers; modify the numbers depending on the 
        // number of drivers. Change the values of other function in source; if needed
        // In case of having one driver, it does not matter then '^'
        int n = 3;
        int list_of_ids[n] = {1, 2, 3};

        // constructor
        zlac8015d(modbus& mb);
        
        // fail read is just readHoldingAddress
    
        // rpm to radpersec
        double rpm_to_radPerSec(double rpm);

        // rpm to linear
        double rpm_to_linear(double rpm);

        // mode
        opMode set_mode(uint16_t slaveAddr, int mode);

        // get mode
        opMode get_mode(uint16_t slaveAddr);

        // enable motor
        modbus::status enable_motor(uint16_t slaveAddr);

        // disable motro
        modbus::status disable_motor(uint16_t slaveAddr);

        // get fault code
        std::array<uint16_t, 4> get_fault_code(uint16_t slaveAddr);

        // clear alarm
        modbus::status clear_alarm(uint16_t slaveAddr);

        // accel/decel
        modbus::status set_accel_time(uint16_t slaveAddr, const int16_t* LR_ms);

        modbus::status set_decel_time(uint16_t slaveAddr, const int16_t* LR_ms);

        // set/get rpm
        modbus::status set_rpm(uint16_t slaveAddr, const int16_t* LR_ms);

        // ig their names speak for themselves
        std::array<int16_t, 2> get_rpm(uint16_t slaveAddr);

        std::array<int16_t, 2> get_rpm_all();

        std::array<double, 2> get_linear_velocities(uint16_t slaveAddr);

        modbus::status set_maxRPM_pos(uint16_t slaveAddr, int16_t const LR_maxRpm);

        modbus::status set_position_async_control(uint16_t slaveAddr);

        modbus::status move_left_wheel(uint16_t slaveAddr);

        modbus::status move_right_wheel(uint16_t slaveAddr);

        double map(double val, int in_min, int in_max,int out_min,int out_max);

        std::array<int32_t, 2> deg_to_32bitArray(double deg);

        modbus::status set_relative_angle(uint16_t slaveAddr, angL, angR);

        std::array<double, 2> get_wheels_travelled(uint16_t slaveAddr);

        std::array<double, 2> get_wheels_tick(uint16_t slaveAddr);
};
