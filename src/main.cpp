#include <Arduino.h>
#include <ArduinoUAVCAN.h>
#include <d0_reader.h>
#include <aquarea_h.h>
#include <sensostar.h>
#include <can.h>
#include <bootloader_interface.h>
#include <wrappers/housebus/heating/heatmeter_1_0.hpp>
#include <wrappers/housebus/heating/heating_status_1_0.hpp>
#include <wrappers/housebus/electricity/obis_reading_1_0.hpp>

#define CMD_SG_OFF 1
#define CMD_SG_NORMAL 2
#define CMD_SG_1 3
#define CMD_SG_2 4
#define CMD_SET_DEMAND_CONTROL 5
#define CMD_SET_AUTO_DEMAND_CONTROL 6

#define UAVCAN_NODE_ID 31

using namespace uavcan::node;
using namespace housebus::heating;
using namespace housebus::electricity;

void    spi_select      ();
void    spi_deselect    ();
uint8_t spi_transfer    (uint8_t const);
void    onExternalEvent ();
bool    transmitCanFrame(CanardFrame const &);
HardwareSerial Serial1(PA10, PA9);
HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial3(PB11, PB10);
HardwareSerial Serial4(PA1, PA0);

void hp_meter_result (uint8_t obis_1, uint8_t obis_2, uint8_t obis_3, uint64_t val);
void    onExecuteCommand_1_1_Request_Received(CanardTransfer const &, ArduinoUAVCAN &);

static int hp_availableForWrite() {
  return Serial1.availableForWrite();
}

static size_t hp_write(uint8_t c) {
  return Serial1.write(c);
}

static size_t hm_write(uint8_t c) {
  return Serial3.write(c);
}

ArduinoUAVCAN uc(UAVCAN_NODE_ID, transmitCanFrame);
Heartbeat_1_0<> hb;
Heatmeter_1_0<1000> hm;
Obis_reading_1_0<1010> em_hp;
Heating_status_1_0<1001> heating_status;
D0Reader meter_heatpump(hp_meter_result);
AquareaH heatpump(0, millis, hp_availableForWrite, hp_write);
Sensostar heat_meter(hm_write, millis);

uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)


static void reload_watchdog() {
  /* Bootloader leaves watchdog enabled - watchdog library pulls in a lot of float stuff, so let's
  keep it very simple: */
  IWDG->KR = 0x0AAAA;
}

void setup() {
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN; //enable MCU debug module clock
  Serial1.end();

  /* Communication with heatpump: 9600,8,E,1 */

  Serial1.begin(9600, SERIAL_8E1);
  /* Stupidly, the driver turns on PULLUP on RX line PA10. This must not be done to be 5V-tolerant.
  Let's hope the µC is fine with it for a few µs and disable it again: */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR10;
  

  /* Main electric counter: 9600,7,E,1 RX only */
  Serial2.begin(9600, SERIAL_7E1);

  /* heat energy meter: 2400,8,E,1 */
  Serial3.begin(2400, SERIAL_8E1);

  /* Aux electric counter (electric heater): 9600,7,E,1 RX only */
  Serial4.begin(9600, SERIAL_7E1);

  /* Enable CAN driver: GPIOC13 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;           // Enable GPIOC clock 
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);

  //bool ret = CANInit(CAN_1000KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  bool ret = CANInit(CAN_125KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  //bool ret = CANInit(CAN_1000KBPS, 3);  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
  if (!ret) while(true);

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  uc.subscribe<ExecuteCommand_1_1::Request<>>(onExecuteCommand_1_1_Request_Received);
}

static void restartNode() {
  bootloader_interface.node_id = UAVCAN_NODE_ID;
  bootloader_interface.magic = BOOTLOADER_INTERFACE_VALID_MAGIC;
  NVIC_SystemReset();
}

void onExecuteCommand_1_1_Request_Received(CanardTransfer const & transfer, ArduinoUAVCAN & uc)
{
  ExecuteCommand_1_1::Request<> req = ExecuteCommand_1_1::Request<>::deserialize(transfer);
  ExecuteCommand_1_1::Response<> rsp;
  rsp = ExecuteCommand_1_1::Response<>::Status::BAD_COMMAND;

  switch(req.data.command)
  {
    case uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART:
    case uavcan_node_ExecuteCommand_Request_1_0_COMMAND_BEGIN_SOFTWARE_UPDATE:
      restartNode();
      break;

    case CMD_SET_DEMAND_CONTROL:
      heatpump.set_demand_control(req.data.parameter.elements[0]);
      rsp = ExecuteCommand_1_1::Response<>::Status::SUCCESS;
      break;
  }
  uc.respond(rsp, transfer.remote_node_id, transfer.transfer_id);
}


void hp_meter_result (uint8_t obis_1, uint8_t obis_2, uint8_t obis_3, uint64_t val) {
  em_hp.data.code[0] = obis_1;
  em_hp.data.code[1] = obis_2;
  em_hp.data.code[3] = obis_3;
  em_hp.data.value = val;
  em_hp.data.unit = 0; /* todo */
  uc.publish(em_hp);
}

static void copy_heatpump_status_data(housebus_heating_heating_status_1_0 *dst, struct hp_status *src) {
    dst->valve_position_dhw = src->valve_dhw;
    dst->defrost_running = src->defrost;
    dst->external_heater = src->external_heater;
    dst->internal_heater = src->internal_heater;
    dst->error_type_h = src->error_type;
    dst->error_number = src->error_number;
    dst->zone1_actual_temp = src->zone1_actual_temp;
    dst->dhw_actual_temp = src->dhw_actual_temp;
    dst->outdoor_temp = src->outdoor_temp;
    dst->return_temp = src->return_temp;
    dst->flow_temp = src->flow_temp;
    dst->zone1_flow_temp = src->zone1_flow_temp;
    dst->zone1_flow_target_temp = src->zone1_flow_target_temp;
    dst->heat_shift_target_temp = src->heat_shift_target_temp;
    dst->flow_target_temp = src->flow_target_temp;
    dst->discharge_temp = src->discharge_temp;
    dst->room_thermostat_temp = src->room_thermostat_temp;
    dst->indoor_pipe_temp = src->indoor_pipe_temp;
    dst->outdoor_pipe_temp = src->outdoor_pipe_temp;
    dst->defrost_temp = src->defrost_temp;
    dst->eva_outlet_temp = src->eva_outlet_temp;
    dst->bypass_outlet_temp = src->bypass_outlet_temp;
    dst->ipm_temp = src->ipm_temp;
    dst->high_pressure = src->high_pressure;
    dst->low_pressure = src->low_pressure;
    dst->compressor_current = src->compressor_current;
    dst->compressor_frequency = src->compressor_frequency;
    dst->pump_flow = src->pump_flow;
    dst->pump_speed = src->pump_speed;
    dst->pump_duty = src->pump_duty;
    dst->fan1_rpm = src->fan1_rpm;
    dst->fan2_rpm = src->fan2_rpm;
    dst->compressor_starts = src->compressor_starts;
    dst->compressor_hours = src->compressor_hours;
    dst->heat_hours = src->heat_hours;
    dst->dhw_hours = src->dhw_hours;
}

void loop() {

  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
    uc.publish(hb);
    prev = now;
  }

  /* Transmit all enqeued CAN frames when there is TX capacity */
  while(CANTxCapacityAvail() && uc.transmitCanFrame()) { }
  
  /* Receive CAN messages */
  if(CANMsgAvail()) {
    uint8_t data[8];
    CanardFrame frame;
    frame.payload = data;
    frame.extended_can_id = 0;
    frame.payload_size = 0;
    CANReceive(&frame);
    uc.onCanFrameReceived(frame);
  }

  reload_watchdog();

  while(Serial4.available()) {
    char c = Serial4.read();
    meter_heatpump.process(c);
  }
  
  while(Serial1.available()) {
    char c = Serial1.read();
    if (heatpump.process(c)) {
      /* new data available */
      copy_heatpump_status_data(&heating_status.data, &heatpump.status);
      uc.publish(heating_status);
    }
  }

  while(Serial3.available()) {
    char c = Serial3.read();
    struct sensostar_data result;
    uint8_t res;
    res = heat_meter.process(c, &result);
    if (res) {
      hm.data.absolute_energy = result.total_heat_energy;
      hm.data.power = result.power;
      hm.data.flow_temp = result.flow_temperature;
      hm.data.return_temp = result.return_temperature;
      hm.data.delta_temp = result.flow_return_difference_temperature;
      hm.data.error_code = result.error;
      hm.data.flow_rate = result.flow_speed;
      uc.publish(hm);
    }
  }

  heatpump.tick();
  {
    static uint32_t time;
    if (millis() > time + 5000) {
      heat_meter.request();
      time = millis();
    }
    
  }
}

void crit_sec_enter(void) {

}

void crit_sec_leave(void) {

}