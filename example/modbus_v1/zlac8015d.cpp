#include "zlac8015d.hpp"

/* Private begings --- wanted to try the stm32 bracket style lol*/
uint16_t zlac8015d::int16ToRegister(int16_t value){
    return static_cast<uint16_t>(value);
}

/* constructor - and public begins there */
zlac8015d(modbus& mb) : mb_(mb){ /*smth here iguess*/}

double zlac8015d::rpm_to_radPerSec(double rpm) {
    return rpm*2*pi/60.0f;
}

double zlac8015d::rpm_to_linear(double rpm){
    wheel = rpm_to_radPerSec(rpm);
    return wheel * R_wheel;
}

        
opMode zlac8015d::set_mode(uint16_t slaveAddr, int mode){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, OPR_MODE, static_cast<uint16_t>(mode));
                if(st != status::OK) return st;
            }
    }

    if(mode == 1) return opMode::POS_REL_CONTROL;
    if(mode == 2) return opMode::POS_ABS_CONTROL;
    if(mode == 3) return opMode::VEL_CONTROL;
    return opMode::FAIL; // if the mode <= 0 && mode > 3
}

         
opMode zlac8015d::get_mode(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, OPR_MODE, 1, result);
    
    if(result.regs[0] == 1) return opMode::POS_REL_CONTROL;
    if(result.regs[0] == 2) return opMode::POS_ABS_CONTROL;
    if(result.regs[0] == 3) return opMode::VEL_CONTROL;
    
    return opMode::FAIL;
}

        
modbus::status zlac8015d::enable_motor(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, CONTROL_REG, ENABLE);
                if(st != status::OK) return st;
            }
    }
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, ENABLE);
    return st;
}
       
modbus::status zlac8015d::disable_motor(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, CONTROL_REG, DOWN_TIME);
                if(st != status::OK) return st;
            }
    }
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, DOWN_TIME);
    return st;
}
        
std::array<uint16_t, 4> zlac8015d::get_fault_code(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FAULT, 1, result);
}
        
modbus::status zlac8015d::clear_alarm(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, CONTROL_REG, ALRM_CLR);
                if(st != status::OK) return st;
            }
    }
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, ALRM_CLR);
    return st;
}
       
modbus::status zlac8015d::set_accel_time(uint16_t slaveAddr, const int16_t* LR_ms){
    if(static_cast<int>(LR_ms[0]) > 32767) LR_ms[0] = 32767;
    if(static_cast<int>(LR_ms[0]) < 0) LR_ms[0] = 0;

    if(static_cast<int>(LR_ms[1]) > 32767) LR_ms[1] = 32767;
    if(static_cast<int>(LR_ms[1]) < 0) LR_ms[1] = 0;

    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, L_ACL_TIME, LR_ms);
    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, R_ACL_TIME, LR_ms);
}

modbus::status zlac8015d::set_decel_time(uint16_t slaveAddr, const int16_t* LR_ms){
    if(static_cast<int>(LR_ms[0]) > 32767) LR_ms[0] = 32767;
    if(static_cast<int>(LR_ms[0]) < 0) LR_ms[0] = 0;

    if(static_cast<int>(LR_ms[1]) > 32767) LR_ms[1] = 32767;
    if(static_cast<int>(LR_ms[1]) < 0) LR_ms[1] = 0;

    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, L_DCL_TIME, LR_ms);

}
modbus::status zlac8015d::set_rpm(uint16_t slaveAddr, const int16_t* LR_ms){
    if(static_cast<int>(LR_ms[0]) > 3000) LR_ms[0] = 3000;
    if(static_cast<int>(LR_ms[0]) > -3000) LR_ms[0] = -3000;

    if(static_cast<int>(LR_ms[1]) > 3000) LR_ms[1] = 3000;
    if(static_cast<int>(LR_ms[1]) > -3000) LR_ms[1] = -3000;

    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, L_CMD_RPM, LR_ms);
    
    return st;
}
std::array<int16_t, 2> zlac8015d::get_rpm(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_RPM, 2, result);

    std::array<int16_t, 2> registers;
    registers[0] = result.regs[0]/10;
    registers[1] = result.regs[1]/10;

    return registers;
}

std::array<int16_t, 2> zlac8015d::get_rpm_all(){
    modbus::readResult result;
    modbus::status st = mb_::readHoldingRegisters(i, L_FB_RPM, 2, result);

    /*if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st = mb_::readHoldingRegisters(i, L_FB_RPM, 2, result);
                result.regs[0]
            }
    }
    */
    std::array<int16_t, 2> registers;
    registers[0] = result.regs[0]/10;
    registers[1] = result.regs[1]/10;

    return registers;
}

std::array<double, 2> zlac8015d::get_linear_velocities(uint16_t slaveAddr){
    std::array<double, 2> rpmLR = get_rpm(slaveAddr);
    double VL = rpm_to_linear(rpmLR[0]);
    double VR = rpm_to_linear(rpmLR[1]);
    rpmLR[0] = VL;
    rpmLR[1] = VR:
    return rpmLR;
}

modbus::status zlac8015d::set_maxRPM_pos(uint16_t slaveAddr, int16_t const LR_maxRpm){
    if(static_cast<int>(LR_ms[0]) > 1000) LR_ms[0] = 1000;
    if(static_cast<int>(LR_ms[0]) < 0) LR_ms[0] = 0;

    if(static_cast<int>(LR_ms[1]) > 1000) LR_ms[1] = 1000;
    if(static_cast<int>(LR_ms[1]) < 0) LR_ms[1] = 0;

    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, L_MAX_RPM_POS, LR_ms);
    
    return st;
}

zlac8015d::sync zlac8015d::set_position_async_control(uint16_t slaveAddr){
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, POS_CONTROL_TYPE, static_cast<uint16_t>(sync::ASYNC));
    if(st != status::OK) break;
    
    return sync::ASYNC;
}

modbus::status zlac8015d::move_left_wheel(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, CONTROL_REG, POS_L_START);
                if(st != status::OK) return st;
            }
    }
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, POS_L_START);
    return st;
}

modbus::status zlac8015d::move_right_wheel(uint16_t slaveAddr){
    if(slaveAddr == 0){
        modbus::status st[n]; // <-- change the array size depending on number of drivers/slaves
            for(int i : list_of_ids){ // TODO somehow return this value later
                st[i] = mb_::writeSingleRegisters(i, CONTROL_REG, POS_R_START);
                if(st != status::OK) return st;
            }
    }
    modbus::status st = mb_.writeSingleRegisters(slaveAddr, CONTROL_REG, POS_R_START);
    return st;
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
    std::array<uint32_t, 2>L_array =deg_to_32bitArray(angL);
    std::array<uint32_t, 2>R_array =deg_to_32bitArray(angR);
    std::array<uint32_t, 2>total;
    for(int i = 0; i < 2; i++){ total[i] = L_array[i] + R_array[i]; }
    modbus::status st = mb_.writeMultipleRegisters(slaveAddr, L_CMD_REL_POS_HI, 2, total);

    return st;
}

std::array<double, 2> zlac8015d::get_wheels_travelled(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_POS_HI, 4, result);
    uint16_t l_pull_hi = result.regs[0];
    uint16_t l_pull_lo = result.regs[1];
    uint16_t r_pull_hi = result.regs[2];
    uint16_t r_pull_lo = result.regs[3];

    uint32_t l_pulse = ((l_pull_hi & 0xFFFF) << 16) (l_pull_lo & 0xFFFF);
    uint32_t r_pulse = ((r_pull_hi & 0xFFFF) << 16) (r_pull_lo & 0xFFFF);

    double l_travelled = (static_cast<double>(l_pulse)/cpr * travel_in_one_rev);
    double r_travelled = (static_cast<double>(r_pulse)/cpr * travel_in_one_rev);

    std::array<double, 2>travelled = {l_travelled, r_travelled};
    return travelled;
}
std::array<double, 2> zlac8015d::get_wheels_tick(uint16_t slaveAddr){
    modbus::readResult result;
    modbus::status st = mb_.readHoldingRegisters(slaveAddr, L_FB_POS_HI, 4, result);
    uint16_t l_pull_hi = result.regs[0];
    uint16_t l_pull_lo = result.regs[1];
    uint16_t r_pull_hi = result.regs[2];
    uint16_t r_pull_lo = result.regs[3];

    uint32_t l_pulse = ((l_pull_hi & 0xFFFF) << 16) (l_pull_lo & 0xFFFF);
    uint32_t r_pulse = ((r_pull_hi & 0xFFFF) << 16) (r_pull_lo & 0xFFFF);

    std::array<double, 2>tick = {l_pulse, r_pulse};
    return tick;
}