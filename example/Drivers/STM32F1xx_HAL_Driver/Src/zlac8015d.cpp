#include "zlac8015d.hpp"
#include "modbus.hpp"

/* Private begings --- wanted to try the stm32 bracket style lol*/
uint16_t zlac8015d::int16ToRegister(int16_t value){
    return static_cast<uint16_t>(value);
}

int16_t zlac8015d::clamp(int16_t value, int16_t lo, int16_t hi){
    if(value > hi) return hi;
    if(value < lo) return lo;
    return value;
}


/* constructor - and public begins there */
zlac8015d::zlac8015d(modbus& mb) : mb_(mb){ /*smth here iguess*/}


modbus::status zlac8015d::to_all_drivers(uint16_t addr, uint16_t data){
    for(int id : list_of_ids){
            modbus::status st = mb_.writeSingleRegisters(id, addr, data);
            if(st != modbus::status::OK) return st;
        }
    return modbus::status::OK;
}

modbus::status zlac8015d::to_all_drivers_MUL(uint16_t addr, int quantity, const uint16_t* data){
    for(int id : list_of_ids){
            modbus::status st = mb_.writeMultipleRegisters(id, addr, quantity, data);
            if(st != modbus::status::OK) return st;
        }
    return modbus::status::OK;
}

double zlac8015d::rpm_to_radPerSec(double rpm) {
    return rpm*2*pi/60.0f;
}

double zlac8015d::rpm_to_linear(double rpm){
    double wheel = rpm_to_radPerSec(rpm);
    return wheel * R_wheel;
}

        
zlac8015d::opMode zlac8015d::set_mode(uint16_t slaveAddr, int mode){

    if(slaveAddr == 0){ 
        modbus::status st = to_all_drivers(OPR_MODE, mode);
        if(st != modbus::status::OK) return opMode::FAIL;
    } 
    else{
        modbus::status st = mb_.writeSingleRegisters(slaveAddr, OPR_MODE, mode);
        if(st != modbus::status::OK) return opMode::FAIL;
    }
    

    if(mode == 1) return opMode::POS_REL_CONTROL;
    if(mode == 2) return opMode::POS_ABS_CONTROL;
    if(mode == 3) return opMode::VEL_CONTROL;
    return opMode::FAIL; // if the mode <= 0 && mode > 3
}

         
zlac8015d::opMode zlac8015d::get_mode(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, OPR_MODE, 1, result);
    if(st != modbus::status::OK) return opMode::FAIL;

    if(result.regs[0] == 1) return opMode::POS_REL_CONTROL;
    if(result.regs[0] == 2) return opMode::POS_ABS_CONTROL;
    if(result.regs[0] == 3) return opMode::VEL_CONTROL;
    
    return opMode::FAIL;
}

        
modbus::status zlac8015d::enable_motor(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers(CONTROL_REG, ENABLE);
        return st;
    }
    return mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, ENABLE);
}
       
modbus::status zlac8015d::disable_motor(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers(CONTROL_REG, DOWN_TIME);
        return st;
    } 
    return mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, DOWN_TIME);
    
}

std::array<uint16_t, 4> zlac8015d::get_fault_code(uint16_t slaveAddr){
    // left and right are contiguous [0x20A5] and [0x20A6] respectively so read both in one shot
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FAULT, 2, result);

    // the fault codes are 2byte hex, so if error then 0s
    if(st != modbus::status::OK){
        return {0, 0, 0, 0};
    }

    uint16_t L_fault_code = result.regs[0];
    uint16_t R_fault_code = result.regs[1];
    uint16_t L_fault_flag = (L_fault_code != NO_FAULT) ? 1 : 0;
    uint16_t R_fault_flag = (R_fault_code != NO_FAULT) ? 1 : 0;

    return {L_fault_code, R_fault_code, L_fault_flag, R_fault_flag};
}   

modbus::status zlac8015d::clear_alarm(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers(CONTROL_REG, ALRM_CLR);
        return st;
    } 
    return mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, ALRM_CLR);

}
       
modbus::status zlac8015d::set_accel_time(uint16_t slaveAddr, const int16_t* LR_ms){
    // minimum accel is 0 meaning no_to_movement; 32767 theoretical max value
    // i mean theoretical >:D
    int16_t L = clamp(LR_ms[0], 0, acl_dcl_limit);
    int16_t R = clamp(LR_ms[1], 0, acl_dcl_limit);

    uint16_t data[2] = { int16ToRegister(L), int16ToRegister(R) };

    // l/r accel or dcl are contiguous as well [0x2080] and [0x2081] respectively
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers_MUL(L_ACL_TIME, 2, data);
        return  st;
    } 
    return mb_.writeMultipleRegisters(slaveAddr, L_ACL_TIME, 2, data);
    
}

modbus::status zlac8015d::set_decel_time(uint16_t slaveAddr, const int16_t* LR_ms){
    int16_t L = clamp(LR_ms[0], 0, acl_dcl_limit);
    int16_t R = clamp(LR_ms[1], 0, acl_dcl_limit);

    uint16_t data[2] = { int16ToRegister(L), int16ToRegister(R) };

    if(slaveAddr == 0){
        modbus::status st = to_all_drivers_MUL(L_DCL_TIME, 2, data);
        return  st;
    } 
    return mb_.writeMultipleRegisters(slaveAddr, L_DCL_TIME, 2, data);

}
modbus::status zlac8015d::set_rpm(uint16_t slaveAddr, const int16_t* LR_ms){
    int16_t L = clamp(LR_ms[0], -rpm_limit, rpm_limit);
    int16_t R = clamp(LR_ms[1], -rpm_limit, rpm_limit);

    uint16_t data[2] = { int16ToRegister(L), int16ToRegister(R) };

    if(slaveAddr == 0){
        modbus::status st = to_all_drivers_MUL(L_CMD_RPM, 2, data);
        return  st;
    } 

    return mb_.writeMultipleRegisters(slaveAddr, L_CMD_RPM, 2, data);
}
std::array<int16_t, 2> zlac8015d::get_rpm(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_RPM, 2, result);

    std::array<int16_t, 2> registers{0, 0};
    if(st != modbus::status::OK) return registers; // i think i should change it; but later ig
                                                   // cause rpm could be 0,0 and like shadow debug

    registers[0] = static_cast<int16_t>(result.regs[0]) / 10;
    registers[1] = static_cast<int16_t>(result.regs[1]) / 10;

    return registers;
}

std::array<int16_t, 2 * zlac8015d::NUM_DRIVERS> zlac8015d::get_rpm_all(){
    std::array<int16_t, 2* NUM_DRIVERS> registers{};

    size_t idReg = 0;
    for(int id : list_of_ids){
        std::array<int16_t, 2> rpm = get_rpm(static_cast<uint16_t>(id));
        registers[idReg++] = rpm[0];
        registers[idReg++] = rpm[1];
    }

    return registers;
}

std::array<double, 2> zlac8015d::get_linear_velocities(uint16_t slaveAddr){
    std::array<int16_t, 2> rpmLR = get_rpm(slaveAddr);
    double VL = rpm_to_linear(rpmLR[0]);
    double VR = rpm_to_linear(rpmLR[1]);
    return {VL, VR};
}

modbus::status zlac8015d::set_maxRPM_pos(uint16_t slaveAddr, const int16_t* LR_maxRpm){
    int16_t L = clamp(LR_maxRpm[0], 0, 1000);
    int16_t R = clamp(LR_maxRpm[1], 0, 1000);

    uint16_t data[2] = { int16ToRegister(L), int16ToRegister(R) };

    return mb_.writeMultipleRegisters(slaveAddr, L_MAX_RPM_POS, 2, data);
   
}

modbus::status zlac8015d::set_position_async_control(uint16_t slaveAddr){
    return mb_.writeSingleRegisters(slaveAddr, POS_CONTROL_TYPE, static_cast<uint16_t>(sync::ASYNC));

}

modbus::status zlac8015d::move_left_wheel(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers(CONTROL_REG, POS_L_START);
        return st;
    } 
    return mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, POS_L_START);

}

modbus::status zlac8015d::move_right_wheel(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st = to_all_drivers(CONTROL_REG, POS_R_START);
        return st;
    } 
    return mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, POS_R_START);
}

double zlac8015d::map(double val, int in_min, int in_max,int out_min,int out_max){
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::array<int32_t, 2> zlac8015d::deg_to_32bitArray(double deg){
    int32_t dec = static_cast<int32_t>(map(deg, -1440, 1440, -65536, 65536));
    int32_t hi = (dec & 0xFFFF0000) >> 16;
    int32_t lo = dec & 0x0000FFFF;
    std::array<int32_t, 2> reg = {hi, lo};
    return reg;
}

modbus::status zlac8015d::set_relative_angle(uint16_t slaveAddr, double angL, double angR){
    std::array<int32_t, 2>L_array =deg_to_32bitArray(angL);
    std::array<int32_t, 2>R_array =deg_to_32bitArray(angR);

    uint16_t data[4] = {
        static_cast<uint16_t>(L_array[0]),
        static_cast<uint16_t>(L_array[1]),
        static_cast<uint16_t>(R_array[0]),
        static_cast<uint16_t>(R_array[1])
    };
    return mb_.writeMultipleRegisters(slaveAddr, L_CMD_REL_POS_HI, 2, data);

}

std::array<double, 2> zlac8015d::get_wheels_travelled(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_POS_HI, 4, result);
    std::array<double, 2>travelled = {0, 0};
    if(st != modbus::status::OK) return travelled;
    uint16_t l_pull_hi = result.regs[0];
    uint16_t l_pull_lo = result.regs[1];
    uint16_t r_pull_hi = result.regs[2];
    uint16_t r_pull_lo = result.regs[3];

    uint32_t l_pulse = ((l_pull_hi & 0xFFFF) << 16) | (l_pull_lo & 0xFFFF);
    uint32_t r_pulse = ((r_pull_hi & 0xFFFF) << 16) | (r_pull_lo & 0xFFFF);

    travelled[0] = (static_cast<double>(l_pulse)/cpr) * travel_in_one_rev;
    travelled[1] = (static_cast<double>(r_pulse)/cpr) * travel_in_one_rev;

    return travelled;
}
std::array<double, 2> zlac8015d::get_wheels_tick(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_POS_HI, 4, result);
    std::array<double, 2>tick = {0, 0};
    if(st != modbus::status::OK) return tick;
    uint16_t l_pull_hi = result.regs[0];
    uint16_t l_pull_lo = result.regs[1];
    uint16_t r_pull_hi = result.regs[2];
    uint16_t r_pull_lo = result.regs[3];

    uint32_t l_pulse = ((l_pull_hi & 0xFFFF) << 16) | (l_pull_lo & 0xFFFF);
    uint32_t r_pulse = ((r_pull_hi & 0xFFFF) << 16) | (r_pull_lo & 0xFFFF);

    tick[0] = static_cast<double>(l_pulse);
    tick[1] = static_cast<double>(r_pulse);
    return tick;
}
