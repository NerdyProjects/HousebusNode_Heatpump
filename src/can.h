#include <ArduinoUAVCAN.h>
#include <stdint.h>

/* Symbolic names for bit rate of CAN message                                */
typedef enum {CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS} BITRATE;

bool transmitCanFrame(CanardFrame const & frame);
bool CANInit(BITRATE bitrate, int remap);
bool CANTxCapacityAvail(void);
uint8_t CANMsgAvail(void);
void CANReceive(CanardFrame* frame);