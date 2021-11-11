#include <stdint.h>
#include <functional>

#define AQUAREA_RX_BUF_SIZE 204
#define AQUAREA_COMMAND_REPLY_TIMEOUT 2000
#define AQUAREA_OPTIONAL_PCB_INTERVAL 5000
#define AQUAREA_OPT_QUERY_SIZE 20
#define AQUAREA_STATUS_INTERVAL 10000
#define AQUAREA_DEMAND_CONTROL_TIMEOUT 7200000

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
    int8_t ipm_temp;    /* what is this? */
    int8_t high_pressure; /* divide by 5 to get value in Kgf/cm² */
    int8_t low_pressure; /* Kgf/cm² */
    int8_t compressor_current; /* divide by 5 to get actual current in A */
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

    private:
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
        uint32_t lastDemandControlCommand;

        
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
};