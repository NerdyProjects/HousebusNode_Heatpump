/**
 * This software is distributed under the terms of the MIT License.
 */

#ifndef ARDUINO_UAVCAN_TYPES_HOUSEBUS_ELECTRICITY_OBIS_READING_1_0_HPP_
#define ARDUINO_UAVCAN_TYPES_HOUSEBUS_ELECTRICITY_OBIS_READING_1_0_HPP_

#include <libcanard/canard.h>

#include <types/housebus/electricity/obis_reading_1_0.h>


namespace housebus {
namespace electricity {

template <CanardPortID ID>
class Obis_reading_1_0
{

public:

  housebus_electricity_obis_reading_1_0 data;

  static constexpr CanardPortID       PORT_ID          = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = housebus_electricity_obis_reading_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND    = CanardTransferKindMessage;

  Obis_reading_1_0()
  {
    housebus_electricity_obis_reading_1_0_initialize_(&data);
  }

  static Obis_reading_1_0 deserialize(CanardTransfer const & transfer)
  {
    Obis_reading_1_0<ID> i;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    housebus_electricity_obis_reading_1_0_deserialize_(&i.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
    return i;
  }

  size_t serialize(uint8_t * payload) const
  {
    size_t inout_buffer_size_bytes = Obis_reading_1_0<ID>::MAX_PAYLOAD_SIZE;
    return (housebus_electricity_obis_reading_1_0_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }
};

}
}

#endif