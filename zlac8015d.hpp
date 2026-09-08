#pragma once

#include "modbus.hpp"
#include "stm32f1xx_hal.h"
#include <cstdint>
#include <stdint.h>
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
        static const uint16_t EMER_STOP = 0x0005;
        static const uint16_t ALRM_CLR = 0x0006;
        static const uint16_t DOWN_TIME = 0x0007;
        static const uint16_t ENABLE = 0x0008;
        static const uint16_t POS_SYNC = 0x0010;
        static const uint16_t POS_L_START = 0x0011;
        static const uint16_t POS_R_START = 0x0012;
    
        // ---- Odometry ----
        static constexpr double travel_in_one_rev = 0.638; // meters morales
        static constexpr int cpr = 16385;
        static constexpr double R_wheel = 0.105; // meters -- no miles
        static constexpr int16_t rpm_limit = 3000;
        static constexpr uint16_t acl_dcl_limit = 32767;
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
        // later for accel/decel value clamping
        static int16_t clamp(int16_t value, int16_t lo, int16_t hi);
        
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
        static constexpr int list_of_ids[3] = {1, 2, 3};
        static constexpr size_t NUM_DRIVERS = sizeof(list_of_ids)/sizeof(list_of_ids[0]);

        // constructor
        explicit zlac8015d(modbus& mb);

        // for every driver transaction
        modbus::status to_all_drivers(uint16_t addr, uint16_t data);
        // for multiple registers write
        modbus::status to_all_drivers_MUL(uint16_t addr, int quantity, const uint16_t* data);
        
        // fail read is just readHoldingAddress
    
        // rpm to radpersec
        double rpm_to_radPerSec(double rpm);

        // rpm to linear
        double rpm_to_linear(double rpm);

        // assigning slaveAddr as 0 will transact data to all drivers
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

        // one rpm has left and right; so two for each driver
        // will go like [id0_l][id0_r][id1_l][id_r].. you got it
        std::array<int16_t, 2*NUM_DRIVERS> get_rpm_all();

        std::array<double, 2> get_linear_velocities(uint16_t slaveAddr);

        modbus::status set_maxRPM_pos(uint16_t slaveAddr, const int16_t* LR_maxRpm);

        modbus::status set_position_async_control(uint16_t slaveAddr);

        modbus::status move_left_wheel(uint16_t slaveAddr);

        modbus::status move_right_wheel(uint16_t slaveAddr);

        double map(double val, int in_min, int in_max,int out_min,int out_max);

        std::array<int32_t, 2> deg_to_32bitArray(double deg);

        modbus::status set_relative_angle(uint16_t slaveAddr, double angL, double angR);

        std::array<double, 2> get_wheels_travelled(uint16_t slaveAddr);

        std::array<double, 2> get_wheels_tick(uint16_t slaveAddr);
};

