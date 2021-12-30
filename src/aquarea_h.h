#include <stdint.h>
#include <functional>

#define AQUAREA_RX_BUF_SIZE 204
#define AQUAREA_COMMAND_REPLY_TIMEOUT 2000
#define AQUAREA_OPTIONAL_PCB_INTERVAL 5000
#define AQUAREA_OPT_QUERY_SIZE 20
#define AQUAREA_STATUS_INTERVAL 10000
#define AQUAREA_DEMAND_CONTROL_TIMEOUT 7200000
#define AQUAREA_SG_TIMEOUT 7200000

#define AQUAREA_SG_NORMAL 0
#define AQUAREA_SG_OFF 2
#define AQUAREA_SG_SG1 1
#define AQUAREA_SG_SG2 3

struct hp_config {
    unsigned hp_state: 1;
    unsigned force_dhw: 1;
    unsigned holiday: 1;
    unsigned week_timer: 1;
    unsigned force_heater: 1;
    unsigned silent_mode: 3; /* off, 1, 2, 3, timer enabled */
    unsigned power_mode: 2; /* off, 30/60/90 min */
    int8_t zone1_shift;
    int8_t dhw_target;
    uint8_t sg1_heating;
    uint8_t sg1_dhw;
    uint8_t sg2_heating;
    uint8_t sg2_dhw;
};

struct hp_status {
    unsigned valve_dhw: 1; /* 0: 3 way valve on heating, 1: 3 way valve on DHW */
    unsigned defrost: 1; /* 1: heatpump is in defrost cycle */
    unsigned external_heater: 1;
    unsigned internal_heater: 1;
    unsigned error_type: 1; /* 0: F Error, 1: H Error */
    uint8_t error_number; /* interpret/print as hex to match error codes from manual */
    int8_t zone1_actual_temp;
    int8_t dhw_actual_temp;
    int8_t outdoor_temp;
    int8_t return_temp;
    int8_t flow_temp;
    int8_t zone1_flow_temp;
    int8_t zone1_flow_target_temp;
    int8_t heat_shift_target_temp; /* See byte 152; unknown what it is actually for */
    int8_t flow_target_temp;
    int8_t discharge_temp; /* compressor discharge sensor */
    int8_t room_thermostat_temp; /* temperature of remote controller? */
    int8_t indoor_pipe_temp;
    int8_t outdoor_pipe_temp;
    int8_t defrost_temp;
    int8_t eva_outlet_temp;
    int8_t bypass_outlet_temp;
    int8_t ipm_temp;    /* what is this? -> transistor temperatures */
    uint8_t high_pressure; /* divide by 5 to get value in Kgf/cm² */
    uint8_t low_pressure; /* Kgf/cm² */
    uint8_t compressor_current; /* divide by 5 to get actual current in A */
    uint8_t compressor_frequency;
    uint16_t pump_flow; /* pump flow in l/min as 8.8 fixed point */
    uint8_t pump_speed; /* pump speed in rpm, multiply by 50 to get RPM */
    uint8_t pump_duty; /* pump duty cycle in % */
    uint8_t fan1_rpm; /* fan 1 rpm: multiply by 10 to get RPM */
    uint8_t fan2_rpm; /* fan 2 rpm: multiply by 10 to get RPM */
    uint16_t compressor_starts; /* likely, the heatpump provides this as 3 byte value */
    uint16_t compressor_hours;
    uint16_t heat_hours;
    uint16_t dhw_hours;
};

struct optional_query {
    unsigned thermostat_2: 2;
    unsigned thermostat_1: 2;   /* 0: off, 1: cool, 2: heat, 3: cool and heat */
    unsigned sg_mode: 2;        /* 0: normal, 2: off, 1: SG1, 3: SG2 */
    unsigned compressor_enable: 1;
    unsigned heat_cool: 1;  
    uint8_t pool_temp;
    uint8_t buffer_temp;
    uint8_t z1_room_temp;
    uint8_t z2_room_temp;
    uint8_t solar_temp;
    /* T-Cap 12 kW experience:
    There is also a weird hystersis active, likely as described in the datasheet so
    going upwards/downwards has different effects (of +/- 4 Hz):
    90 is 38 Hz
    100 is about 42 Hz (while minimum is 48 slowly modulating down to 37 over an hour)
    110, 115 is 54 Hz
    120 is 56 Hz
    130 is 60 Hz

    (A1/W40):
    High -> 115: 53/54 Hz
    115 -> 110: 48/49/50 Hz (2.85 kW)
    ... stable in single downward steps between
    102 -> 101:  47/48 Hz (2.45 kW)
    
    90 -> 77: 42 Hz (2.0 kW)
    91 -> 90: 40 Hz (2.1 kW)
    90 -> 100 46 Hz (2.45 kW)
    100 -> 110 50 Hz (2.85 kW)
    112 -> 120 53 Hz (3.25 kW)
    130 (3.57 kW)
    140 (4 kW)
    */
    uint8_t demand_control;     /* 43: 5% up to 234: 100% */
    uint8_t z1_water_temp;
    uint8_t z2_water_temp;
};

typedef std::function<uint32_t(void)> TimestampMillisecondsFunc;
typedef std::function<int(void)> WriteBufferAvailableFunc;
typedef std::function<size_t(uint8_t)> WriteByteFunc;

class AquareaH {
    public:
        AquareaH(uint8_t optPcbEnable, TimestampMillisecondsFunc getMillis, WriteBufferAvailableFunc availableForWrite, WriteByteFunc write);
        uint8_t process(uint8_t c);
        void tick();
        struct hp_config config;
        struct hp_status status;
        void set_demand_control(uint8_t v);
        void set_sg(uint8_t v);
        void enable_inhibition_control();
        void disable_inhibition_control();
        void enable_power_control();
        void disable_power_control();
        void enable_temperature_defrost_control();
        void disable_temperature_defrost_control();

    private:
        enum Command { REQUEST, REQUEST_DEFROST};
        uint8_t rxBufPos;
        /* buffer to store a whole RX packet to be able to check CRC before processing */
        uint8_t rxBuf[AQUAREA_RX_BUF_SIZE];
        uint8_t optPcbEnabled;
        uint32_t lastOptPCBquery;
        uint32_t lastStatusResult;
        uint32_t lastCommandSend;
        uint8_t send_lock;
        struct optional_query optSettings;
        TimestampMillisecondsFunc _getMillis;
        WriteBufferAvailableFunc _availableForWrite;
        WriteByteFunc _write;
        uint8_t checksum;
        uint8_t commandWriteState;
        enum Command activeCommand;
        uint32_t lastDemandControlCommand;
        uint32_t lastSGCommand;
        bool inhibition_control;
        bool lastCompressorState;
        /* gets true as soon as the first heatpump answer package has been received */
        bool initDone;
        uint32_t compressorTurnedOffAt;
        uint32_t compressorTurnedOnAt;
        bool inhibition_control_temperature_trigger;
        bool inhibition_control_inhibit;
        uint32_t inhibition_control_inhibit_startAt;
        bool inhibition_control_low_power;
        bool power_control;
        bool temperature_defrost_control;
        uint8_t temperature_defrost_condition_fulfilled_count;
        static int8_t power_control_temperatures[10];
        static uint8_t power_control_demand[10];
        uint8_t lastFlowTargetTemperature;
        uint32_t last_defrost_active_at;

        
        void init_optional_pcb_settings(struct optional_query *optionalQuery);
        void decode_heatpump_data(uint8_t *raw, struct hp_config *config, struct hp_status *status);
        void decode_optional_heatpump_data(uint8_t *raw);
        uint8_t decode_bool_78(uint8_t raw);
        uint8_t decode_bool_56(uint8_t raw);
        uint8_t decode_bool_34(uint8_t raw);
        uint8_t decode_bool_12(uint8_t raw);
        uint8_t decode_bool_check(uint8_t val);
        void send_optional_pcb_packet(const struct optional_query *query);
        void write(uint8_t c);
        void write_checksum();
        uint8_t acquire_send_lock();
        void send_request();
        void send_request_continue();
        void send_defrost_request();
};