#include <aquarea_h.h>

AquareaH::AquareaH(uint8_t optPcbEnable, TimestampMillisecondsFunc getMillis, WriteBufferAvailableFunc availableForWrite, WriteByteFunc write) {
    uint32_t timestamp = getMillis();
    optPcbEnabled = optPcbEnable;
    init_optional_pcb_settings(&optSettings);
    _getMillis = getMillis;
    _availableForWrite = availableForWrite;
    _write = write;
    send_lock = 0;
    lastCommandSend = 0;
    lastOptPCBquery = timestamp; /* delay the optional PCB query a bit after turn on */
    lastCompressorState = 0;
    lastDHWState = 0;
    lastStatusResult = 0;
    checksum = 0;
    commandWriteState = 0;
    lastDemandControlCommand = 0;
    lastSGCommand = 0;
    inhibition_control = true;
    power_control = true;
    inhibition_control_inhibit = false;
    inhibition_control_low_power = false;
    inhibition_control_temperature_trigger = false;
    initDone = false;
    temperature_defrost_condition_fulfilled_count = 0;
    temperature_defrost_control = true;
    compressorTurnedOnAt = timestamp - (120 * 60000);
    compressorTurnedOffAt = timestamp - (120 * 60000);
    last_defrost_active_at = timestamp;
}
/* flow target temp for demand 77, 90, 100, 110, 120, 130, 141, 234 */
uint8_t AquareaH::power_control_demand[10] = {77, 90, 100, 110, 120, 130, 140, 150, 160, 234};
int8_t AquareaH::power_control_temperatures[10] =  {33, 34, 35,  36,  37,  39,  40,  41, 42, 43};
#define AQUAREAH_MAX_FLOW_TARGET_FOR_INHIBITION 31

void AquareaH::init_optional_pcb_settings(struct optional_query *optionalQuery) {
    optionalQuery->heat_cool = 0;
    optionalQuery->compressor_enable = 1;
    optionalQuery->sg_mode = 0;
    optionalQuery->thermostat_1 = 0;
    optionalQuery->thermostat_2 = 0;
    optionalQuery->pool_temp = 0xFF;
    optionalQuery->buffer_temp = 0xFF;
    optionalQuery->z1_room_temp = 0xFF;
    optionalQuery->z2_room_temp = 0xFF;
    optionalQuery->solar_temp = 0xFF;
    optionalQuery->demand_control = 0xEA;
    optionalQuery->z2_water_temp = 0xFF;
    optionalQuery->z1_water_temp = 0xFF;
}

uint8_t AquareaH::process(uint8_t c) {
    uint8_t res = 0;
    if (rxBufPos == AQUAREA_RX_BUF_SIZE - 1) {
        /* packet too long, however we ended up in this situation */
        rxBufPos = 0;
    }
    rxBuf[rxBufPos++] = c;
    if (rxBuf[0] != 0x71) {
        /* header not matching -> skip byte */
        rxBufPos = 0;
    }
    if (rxBufPos == 2 && rxBuf[1] > AQUAREA_RX_BUF_SIZE - 3) {
        /* packet potentially too long -> likely an error */
        rxBufPos = 0;
    }
    if (rxBufPos == 3 && rxBuf[2] != 0x01) {
        /* header not matching - error */
    }
    if (rxBufPos == 4 && rxBuf[3] != 0x10 && rxBuf[3] != 0x50) {
        /* neither normal nor optional response -> error */
    }
    if (rxBufPos == rxBuf[1] + 3) {
        /* RX done */
        uint8_t sum = 0;
        for (uint8_t i = 0; i < rxBufPos; ++i) {
            sum += rxBuf[i];
        }
        if (sum == 0) {
            send_lock = 0;
            /* checksum correct */    
            if (rxBufPos == 203) {
                bool compressorState;
                bool dhwState;
                uint32_t timestamp = _getMillis();
                decode_heatpump_data(rxBuf, &config, &status);
                compressorState = status.compressor_frequency > 0;
                dhwState = status.valve_dhw > 0;
                lastStatusResult = timestamp;
                if (!initDone) {
                    lastCompressorState = compressorState;
                    lastDHWState = dhwState;
                    initDone = true;
                }
                if (lastCompressorState && !compressorState && !status.defrost && !lastDHWState && !dhwState) {
                    compressorTurnedOffAt = timestamp;
                    inhibition_control_temperature_trigger = 
                        status.flow_temp > status.flow_target_temp &&
                        status.flow_target_temp <= AQUAREAH_MAX_FLOW_TARGET_FOR_INHIBITION &&
                        status.defrost == 0;
                }
                if (!lastCompressorState && compressorState) {
                    compressorTurnedOnAt = timestamp;
                }
                if (status.defrost) {
                    last_defrost_active_at = timestamp;
                }
                if (status.outdoor_pipe_temp < status.outdoor_temp - 8 && status.outdoor_temp < 6) {
                    temperature_defrost_condition_fulfilled_count ++;
                } else {
                    temperature_defrost_condition_fulfilled_count = 0;
                }
                lastFlowTargetTemperature = status.flow_target_temp;
                lastCompressorState = compressorState;
                lastDHWState = dhwState;
                res = 1;
            } else if (rxBufPos == 20) {
                decode_optional_heatpump_data(rxBuf);
            } else {
                /* different type answer package? */
            }
        } else {
            /* checksum incorrect */
        }
        rxBufPos = 0;
    }
    return res;
}

void AquareaH::write(uint8_t c) {
    checksum += c;
    _write(c);
}

void AquareaH::write_checksum() {
    write(256-checksum);
}

uint8_t AquareaH::acquire_send_lock() {
    if(send_lock != 0) {
        return 0;
    }
    send_lock = 1;
    checksum = 0;
    lastCommandSend = _getMillis();
    return 1;
}

void AquareaH::send_optional_pcb_packet(const struct optional_query *query) {
    if(!acquire_send_lock()) {
        return;
    }
    uint8_t effective_sg_mode = query->sg_mode;
    uint32_t effective_demand_control = query->demand_control;
    uint8_t effective_compressor_enable = query->compressor_enable;
    uint32_t timestamp = _getMillis();
    
    if (inhibition_control && inhibition_control_inhibit) {
        effective_sg_mode = AQUAREA_SG_OFF;        
        /* effective_compressor_enable = 0; */
    }
    if (initDone && power_control && (timestamp - lastStatusResult) < 30000) {
        int i;
        uint8_t power_control_result = 0;
        for (i = 0; i < 10; ++i) {
            if (lastFlowTargetTemperature <= power_control_temperatures[i]) {
                power_control_result = power_control_demand[i];
                break;
            }
        }
        if (power_control_result < effective_demand_control) {
            effective_demand_control = power_control_result;
        }
    }
    if (inhibition_control && inhibition_control_low_power) {
        effective_demand_control = 43;
    }
    lastOptPCBquery = timestamp;
    write(0xF1);
    write(0x11);
    write(0x01);
    write(0x50);
    write(0x00);
    write(0x00);
    write((query->heat_cool << 7) | (effective_compressor_enable << 6) | (effective_sg_mode << 4) | (query->thermostat_1 << 2) | (query->thermostat_2));
    write(query->pool_temp);
    write(query->buffer_temp);
    write(0xE5);
    write(query->z1_room_temp);
    write(query->z2_room_temp);
    write(0);
    write(query->solar_temp);
    write(effective_demand_control);
    write(query->z2_water_temp);
    write(query->z1_water_temp);
    write(0);
    write(0);
    write_checksum();
}

void AquareaH::send_request_continue() {
    switch (commandWriteState) {
        case 0:
        if (activeCommand == REQUEST) {
            write(0x71);
        } else {
            write(0xF1);
        }        
        break;
        case 1:
        write(0x6c);
        break;
        case 2:
        write(0x01);
        break;
        case 3:
        write(0x10);
        break;
        case 8:
        if (activeCommand == REQUEST_DEFROST) {
            write(0x02);
        } else {
            write(0x00);
        }
        break;
        case 110:
        write_checksum();
        commandWriteState = 0;
        return;
        break;
        default:
        write(0x00);
        break;
    }
    ++commandWriteState;
}

void AquareaH::send_defrost_request() {
    if (!acquire_send_lock()) {
        return;
    }
    activeCommand = REQUEST_DEFROST;
    commandWriteState = 0;
    send_request_continue();
}

void AquareaH::send_request() {
    if (!acquire_send_lock()) {
        return;
    }
    activeCommand = REQUEST;
    commandWriteState = 0;
    send_request_continue();
}

void AquareaH::set_demand_control(uint8_t v) {
    optSettings.demand_control = v;
    lastDemandControlCommand = _getMillis();
}

void AquareaH::set_sg(uint8_t v) {
    if (v <= AQUAREA_SG_SG2) {
        optSettings.sg_mode = v;
    }
    lastSGCommand = _getMillis();    
}

void AquareaH::enable_inhibition_control() {
    inhibition_control = true;
}

void AquareaH::disable_inhibition_control() {
    inhibition_control = false;
}

#define INHIBITION_CONTROL_TRIGGER_COMPRESSOR_MIN_OFF (90000)
#define INHIBITION_CONTROL_INHIBIT_FOR (90 * 60 * 1000)
#define INHIBITION_CONTROL_LIMIT_POWER_FOR (60 * 60 * 1000)
/* Do not send a defrost command when last defrost state is less than this ago to allow minimum heating time.
Default of heatpump is something around 30 minutes here which is too long, but I guess it's reasonable to keep
a certain minimum */
#define MIN_DEFROST_HEATING_INTERVAL (17 * 60 * 1000)

void AquareaH::tick() {
    uint32_t timestamp = _getMillis();
    if (initDone) {
        if (inhibition_control) {
            /* Try to have the heatpump stay off longer when it should operate below
            its lowest power output.
            a) when it turns off due to too high power output, keep it off for a
            certain time (1 hour?)
            b) when it turns back on, limit the power using demand control to the
            minimum for a certain time (1 hour?)
            */

           /* turn off (implemented via SG ready) when off for a minute and flow temp
            in the moment of turn off was too high */
           if (!inhibition_control_inhibit && !lastCompressorState && inhibition_control_temperature_trigger &&
           timestamp - compressorTurnedOffAt > INHIBITION_CONTROL_TRIGGER_COMPRESSOR_MIN_OFF &&
           timestamp - compressorTurnedOffAt < INHIBITION_CONTROL_TRIGGER_COMPRESSOR_MIN_OFF + 60000) {
               inhibition_control_inhibit = true;
               inhibition_control_inhibit_startAt = timestamp;
           }

          /* allow turning back on after time has elapsed */  
           if (inhibition_control_inhibit &&
           timestamp - inhibition_control_inhibit_startAt > INHIBITION_CONTROL_INHIBIT_FOR) {
               inhibition_control_inhibit = false;
               /* prevent immediate retriggering */
               inhibition_control_temperature_trigger = false;
           }

           inhibition_control_low_power = lastCompressorState &&
           lastFlowTargetTemperature < power_control_temperatures[1] &&
            (timestamp - compressorTurnedOnAt < INHIBITION_CONTROL_LIMIT_POWER_FOR);
        } else {
            inhibition_control_inhibit = false;
            inhibition_control_low_power = false;
            inhibition_control_temperature_trigger = false;
        }
    }
    
    if (timestamp - lastDemandControlCommand > AQUAREA_DEMAND_CONTROL_TIMEOUT) {
        /* reset demand control to 100% */
        optSettings.demand_control = 0xEA;
    }

    if (timestamp - lastSGCommand > AQUAREA_SG_TIMEOUT) {
        /* reset SG control to normal when there was no signalling */
        optSettings.sg_mode = AQUAREA_SG_NORMAL;
    }

    
    
    while (send_lock && commandWriteState && _availableForWrite()) {
        send_request_continue();
    }
    if (send_lock && timestamp - lastCommandSend > AQUAREA_COMMAND_REPLY_TIMEOUT) {
        /* timeout for reply - continue... */
        send_lock = 0;
    }
    if (temperature_defrost_control &&
    !send_lock &&
    timestamp - last_defrost_active_at > MIN_DEFROST_HEATING_INTERVAL &&
    timestamp - compressorTurnedOnAt > MIN_DEFROST_HEATING_INTERVAL &&
    lastCompressorState &&
    temperature_defrost_condition_fulfilled_count > 3) {
        temperature_defrost_condition_fulfilled_count = 0;
        send_defrost_request();
    }    
    
    if (optPcbEnabled && !send_lock && timestamp - lastOptPCBquery > AQUAREA_OPTIONAL_PCB_INTERVAL) {
        /* because we are using the send lock, the TX buffer should provide enough space for the whole opt. packet */
        send_optional_pcb_packet(&optSettings);
    }
    if (!send_lock && timestamp - lastStatusResult > AQUAREA_STATUS_INTERVAL) {
        send_request();
    }
}

static uint8_t decode_silent_mode(uint8_t raw) {
    uint8_t res = raw >> 3;
    if(res >= 9 && res <= 17) {
        return res - 9;
    } else {
        /* unknown value */
        return 0;
    }
}

static uint8_t decode_power_mode(uint8_t raw) {
    uint8_t res = raw & 0x7;
    if (res >= 1 && res <= 4) {
        return res - 1;
    } else {
        /* unknown value */
        return 0;
    }
}

void AquareaH::decode_heatpump_data(uint8_t *raw, struct hp_config *config, struct hp_status *status) {
    config->hp_state = decode_bool_78(raw[4]);
    config->force_dhw = decode_bool_12(raw[4]);
    config->holiday = decode_bool_34(raw[5]);
    config->week_timer = decode_bool_12(raw[5]);
    config->force_heater = decode_bool_56(raw[5]);
    config->silent_mode =  decode_silent_mode(raw[7]);
    config->power_mode = decode_power_mode(raw[7]);
    config->zone1_shift = raw[38] - 128;
    config->dhw_target = raw[42] - 128;
    config->sg1_heating = raw[71] - 1;
    config->sg1_dhw = raw[72] - 1;
    config->sg2_heating = raw[73] - 1;
    config->sg2_dhw = raw[74] - 1;
    status->valve_dhw = decode_bool_78(raw[111]);
    status->defrost = decode_bool_56(raw[111]);
    status->external_heater = decode_bool_78(raw[112]);
    status->internal_heater = decode_bool_56(raw[112]);
    status->error_type = raw[113] == 0xA1 ? 1 : 0;
    status->error_number = raw[114];
    status->zone1_actual_temp = raw[139] - 128;
    status->dhw_actual_temp = raw[141] - 128;
    status->outdoor_temp = raw[142] - 128;
    status->return_temp = raw[143] - 128;
    status->flow_temp = raw[144] - 128;
    status->zone1_flow_temp = raw[145] - 128;
    status->zone1_flow_target_temp = raw[147] - 128;
    status->flow_target_temp = raw[153] - 128;
    status->discharge_temp = raw[155] - 128;
    status->room_thermostat_temp = raw[156] - 128;
    status->indoor_pipe_temp = raw[157] - 128;
    status->outdoor_pipe_temp = raw[158] - 128;
    status->defrost_temp = raw[159] - 128;
    status->eva_outlet_temp = raw[160] - 128;
    status->bypass_outlet_temp = raw[161] - 128;
    status->ipm_temp = raw[162] - 128;
    status->high_pressure = raw[163] - 1;
    status->low_pressure = raw[164] - 1;
    status->compressor_current = raw[165] - 1;
    status->compressor_frequency = raw[166] - 1;
    status->pump_flow = (raw[170] << 8) + (raw[169] - 1);
    status->pump_speed = raw[171] - 1;
    status->pump_duty = raw[172] - 1;
    status->fan1_rpm = raw[173] - 1;
    status->fan2_rpm = raw[174] - 1;
    status->compressor_starts = (raw[180] << 8) + raw[179] - 1; /* maybe + raw[181] << 16 ? */
    status->compressor_hours = (raw[183] << 8) + raw[182] - 1; /* maybe + raw[184] << 16 ? */
    status->heat_hours = (raw[186] << 8) + raw[185] - 1;
    status->dhw_hours = (raw[189] << 8) + raw[188] - 1;
}

/*
{0xF1, 0x11, 0x01, 0x50, 0x00, 0x00, 0x40, 0xFF, 0xFF, 0xE5, 0xFF, 0xFF, 0x00, 0xFF, 0xEB, 0xFF, 0xFF, 0x00, 0x00};
*/


void AquareaH::decode_optional_heatpump_data(uint8_t *raw) {
    
}

uint8_t AquareaH::decode_bool_check(uint8_t val) {
    if (val != 1 && val != 2) {
        /* unknown state combination: TODO propagate error */
        return 0;
    }
    return 1;
}

uint8_t AquareaH::decode_bool_78(uint8_t raw) {
    uint8_t res = raw & 0x03;
    if (decode_bool_check(res)) {
        return res - 1;
    }
    return 0;    
}

uint8_t AquareaH::decode_bool_56(uint8_t raw) {
    uint8_t res = (raw >> 2) & 0x03;
    if (decode_bool_check(res)) {
        return res - 1;
    }
    return 0;    
}


uint8_t AquareaH::decode_bool_34(uint8_t raw) {
    uint8_t res = (raw >> 4) & 0x03;
    if (decode_bool_check(res)) {
        return res - 1;
    }
    return 0;    
}

uint8_t AquareaH::decode_bool_12(uint8_t raw) {
    uint8_t res = (raw >> 6) & 0x03;
    if (decode_bool_check(res)) {
        return res - 1;
    }
    return 0;    
}