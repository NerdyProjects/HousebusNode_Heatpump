/**
 * This software is distributed under the terms of the MIT License.
 */

#ifndef ARDUINO_UAVCAN_TYPES_HOUSEBUS_HEATING_HEATMETER_1_0_HPP_
#define ARDUINO_UAVCAN_TYPES_HOUSEBUS_HEATING_HEATMETER_1_0_HPP_

#include <libcanard/canard.h>

#include <types/housebus/heating/heatmeter_1_0.h>


namespace housebus {
namespace heating {

template <CanardPortID ID>
class Heatmeter_1_0
{

public:

  housebus_heating_heatmeter_1_0 data;

  static constexpr CanardPortID       PORT_ID          = ID;
  static constexpr size_t             MAX_PAYLOAD_SIZE = housebus_heating_heatmeter_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
  static constexpr CanardTransferKind TRANSFER_KIND    = CanardTransferKindMessage;

  Heatmeter_1_0()
  {
    housebus_heating_heatmeter_1_0_initialize_(&data);
  }

  static Heatmeter_1_0 deserialize(CanardTransfer const & transfer)
  {
    Heatmeter_1_0<ID> i;
    size_t inout_buffer_size_bytes = transfer.payload_size;
    housebus_heating_heatmeter_1_0_deserialize_(&i.data, (uint8_t *)(transfer.payload), &inout_buffer_size_bytes);
    return i;
  }

  size_t serialize(uint8_t * payload) const
  {
    size_t inout_buffer_size_bytes = Heatmeter_1_0<ID>::MAX_PAYLOAD_SIZE;
    return (housebus_heating_heatmeter_1_0_serialize_(&data, payload, &inout_buffer_size_bytes) < NUNAVUT_SUCCESS) ? 0 : inout_buffer_size_bytes;
  }
};

}
}

#endif