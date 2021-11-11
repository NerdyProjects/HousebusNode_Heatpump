#include <Arduino.h>
#include <can.h>

#define AF4   0x04
#define AF0   0x00

#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR               (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE               (1U << 2U)  // Bit 2: Identifier Extension

#define CAN_EXT_ID_MASK                 0x1FFFFFFFU
#define CAN_STD_ID_MASK                 0x000007FFU

typedef const struct
{
  uint8_t TS2;
  uint8_t TS1;
  uint8_t BRP;
} CAN_bit_timing_config_t;

CAN_bit_timing_config_t can_configs[6] = {{2, 13, 60}, {2, 13, 30}, {2, 13, 24}, {2, 13, 12}, {2, 13, 6}, {2, 13, 3}};

bool CANTxCapacityAvail(void) {
  return CAN->TSR & CAN_TSR_TME_Msk;
}

bool transmitCanFrame(CanardFrame const &frame)
{
  uint8_t mailbox = 0;
  switch ((CAN->TSR & CAN_TSR_TME_Msk) >> CAN_TSR_TME_Pos) {
    case 1:
    case 3:
    case 5:
    case 7:
      mailbox = 0;
      break;
    case 2:
    case 6:
      mailbox = 1;
      break;
    case 4:
      mailbox = 2;
      break;
    default:
      return 0;
      break;
  }
  uint32_t out = ((frame.extended_can_id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  const uint8_t *payload = (const uint8_t *)frame.payload;
  CAN->sTxMailBox[mailbox].TDTR &= ~(0xF);
  CAN->sTxMailBox[mailbox].TDTR |= frame.payload_size & 0xFUL;
  
  CAN->sTxMailBox[mailbox].TDLR  = (((uint32_t) payload[3] << 24) |
                               ((uint32_t)payload[2] << 16) |
                               ((uint32_t)payload[1] <<  8) |
                               ((uint32_t)payload[0]      ));
  CAN->sTxMailBox[mailbox].TDHR  = (((uint32_t) payload[7] << 24) |
                               ((uint32_t)payload[6] << 16) |
                               ((uint32_t)payload[5] <<  8) |
                               ((uint32_t)payload[4]      ));

  // Send Go
  CAN->sTxMailBox[mailbox].TIR = out | STM32_CAN_TIR_TXRQ;

  return true;
}

/**
 * Initializes the CAN GPIO registers.
 *
 * @params: addr    - Specified GPIO register address.
 * @params: index   - Specified GPIO index.
 * @params: afry    - Specified Alternative function selection AF0-AF15.
 * @params: speed   - Specified OSPEEDR register value.(Optional)
 *
 */
void CANSetGpio(GPIO_TypeDef * addr, uint8_t index, uint8_t afry, uint8_t speed = 3) {
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7) {
      _index4 = (index - 8) * 4;
      ofs = 1;
    }

    uint32_t mask;
    
    mask = 0xF << _index4;
    addr->AFR[ofs]  &= ~mask;         // Reset alternate function
    //setting = 0x9;                    // AF9
    setting = afry;                   // Alternative function selection
    mask = setting << _index4;
    addr->AFR[ofs]  |= mask;          // Set alternate function
    
    mask = 0x3 << _index2;
    addr->MODER   &= ~mask;           // Reset mode
    setting = 0x2;                    // Alternate function mode
    mask = setting << _index2;
    addr->MODER   |= mask;            // Set mode
    
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;           // Reset speed
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;            // Set speed
    
    mask = 0x1 << index;
    addr->OTYPER  &= ~mask;           // Reset Output push-pull
    
    mask = 0x3 << _index2;
    addr->PUPDR   &= ~mask;           // Reset port pull-up/pull-down
}


/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 27) return;

  CAN->FA1R &= ~(0x1UL<<index);               // Deactivate filter

  if (scale == 0) {
    CAN->FS1R &= ~(0x1UL<<index);             // Set filter to Dual 16-bit scale configuration
  } else {
    CAN->FS1R |= (0x1UL<<index);              // Set filter to single 32 bit configuration
  }
  if (mode == 0) {
    CAN->FM1R &= ~(0x1UL<<index);             // Set filter to Mask mode
  } else {
    CAN->FM1R |= (0x1UL<<index);              // Set filter to List mode
  }

  if (fifo == 0) {
    CAN->FFA1R &= ~(0x1UL<<index);            // Set filter assigned to FIFO 0
  } else {
    CAN->FFA1R |= (0x1UL<<index);             // Set filter assigned to FIFO 1
  }

  CAN->sFilterRegister[index].FR1 = bank1;    // Set filter bank registers1
  CAN->sFilterRegister[index].FR2 = bank2;    // Set filter bank registers2

  CAN->FA1R |= (0x1UL<<index);                // Activate filter

}

    
/**
 * Initializes the CAN controller with specified bit rate.
 *
 * @params: bitrate - Specified bitrate. If this value is not one of the defined constants, bit rate will be defaulted to 125KBS
 * @params: remap   - Select CAN port. 
 *                    =0:CAN_RX mapped to PA11, CAN_TX mapped to PA12
 *                    =1:Not used
 *                    =2:CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)
 *                    =3:CAN_RX mapped to PD0, CAN_TX mapped to PD1 (available on 100-pin and 144-pin package)
 *
 */
bool CANInit(BITRATE bitrate, int remap)
{
  // Reference manual
  // https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

  RCC->APB1ENR |= 0x2000000UL;          // Enable CAN clock 

  if (remap == 0) {
    RCC->AHBENR |= 0x20000UL;           // Enable GPIOA clock 
    CANSetGpio(GPIOA, 11, AF4);         // Set PA11 to AF4
    CANSetGpio(GPIOA, 12, AF4);         // Set PA12 to AF4
  }

  if (remap == 2) {
    RCC->AHBENR |= 0x40000UL;           // Enable GPIOB clock 
    CANSetGpio(GPIOB, 8, AF4);          // Set PB8 to AF4
    CANSetGpio(GPIOB, 9, AF4);          // Set PB9 to AF4
  }

  if (remap == 3) {
    RCC->AHBENR |= 0x100000UL;          // Enable GPIOD clock 
    CANSetGpio(GPIOD, 0, AF0);          // Set PD0 to AF0
    CANSetGpio(GPIOD, 1, AF0);          // Set PD1 to AF0
  }

  CAN->MCR |= 0x1UL;                   // Set CAN to Initialization mode 
  while (!(CAN->MSR & 0x1UL));         // Wait for Initialization mode

  //CAN->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
  CAN->MCR = 0x41UL | 0x04;                   // Hardware initialization(With automatic retransmission)
  // and fifo priority tx queue TXFP (0x04)

  // Set bit rates 
  //CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
  //CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
  CAN->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF)); 
  CAN->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x3FF);
      
  // Configure Filters to default values
  CAN->FMR  |=   0x1UL;                // Set to filter initialization mode
  CAN->FMR  &= 0xFFFFC0FF;             // Clear CAN2 start bank

  // bxCAN has 28 filters.
  // These filters are used for both CAN1 and CAN2.
  // STM32F072 has only CAN1, so all 28 are used for CAN1
  CAN->FMR  |= 0x1C << 8;              // Assign all filters to CAN1

  // Set fileter 0
  // Single 32-bit scale configuration 
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0 
  // Filter bank register to all 0
  CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL); 

  CAN->FMR   &= ~(0x1UL);              // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;
  bool can1 = false;
  CAN->MCR   &= ~(0x1UL);              // Require CAN1 to normal mode 

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //Serial.print("can1=");
  //Serial.println(can1);
  if (can1) {
    //Serial.println("CAN1 initialize ok");
  } else {
    //Serial.println("CAN1 initialize fail!!");
    return false;
  }
  return true; 
}
 
/**
 * Decodes CAN messages from the data registers and populates a 
 * CAN message struct with the data fields.
 * 
 * @preconditions     - A valid CAN message is received
 * @params CAN_rx_msg - CAN message structure for reception
 * 
 */
void CANReceive(CanardFrame* frame)
{
  uint8_t *payload = (uint8_t *)frame->payload;
    uint32_t id = CAN->sFIFOMailBox[0].RIR;
    if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
        frame->extended_can_id = (CAN_STD_ID_MASK & (id >> 21U));
    } 
    else {                               // Extended frame format
        frame->extended_can_id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    frame->timestamp_usec = micros();
    frame->payload_size = (CAN->sFIFOMailBox[0].RDTR) & 0xFUL;
    
    payload[0] = 0xFFUL &  CAN->sFIFOMailBox[0].RDLR;
    payload[1] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 8);
    payload[2] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 16);
    payload[3] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 24);
    payload[4] = 0xFFUL &  CAN->sFIFOMailBox[0].RDHR;
    payload[5] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 8);
    payload[6] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 16);
    payload[7] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 24);

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    CAN->RF0R |= 0x20UL;
}

 /**
 * Returns whether there are CAN messages available.
 *
 * @returns If pending CAN messages are in the CAN controller
 *
 */
uint8_t CANMsgAvail(void)
{
  // Check for pending FIFO 0 messages
  return CAN->RF0R & 0x3UL;
}
