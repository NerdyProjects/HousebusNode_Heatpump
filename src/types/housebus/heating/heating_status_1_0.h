// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://uavcan.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.5.1 (serialization was enabled)
// Source file:   /home/user/Documents/PlatformIO/Projects/stm32 test/dsdl/housebus/heating/heating_status.1.0.uavcan
// Generated at:  2021-11-10 16:37:42.882453 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     housebus.heating.heating_status
// Version:       1.0
//
// Platform
//     python_implementation:  CPython
//     python_version:  3.9.7
//     python_release_level:  final
//     python_build:  ('default', 'Aug 31 2021 13:28:12')
//     python_compiler:  GCC 11.1.0
//     python_revision:
//     python_xoptions:  {}
//     runtime_platform:  Linux-5.13.19-2-MANJARO-x86_64-with-glibc2.33
//
// Language Options
//     target_endianness:  any
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  True
//     enable_override_variable_array_capacity:  False

#ifndef HOUSEBUS_HEATING_HEATING_STATUS_1_0_INCLUDED_
#define HOUSEBUS_HEATING_HEATING_STATUS_1_0_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdint.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 1693710260,
              "/home/user/Documents/PlatformIO/Projects/stm32 test/dsdl/housebus/heating/heating_status.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/user/Documents/PlatformIO/Projects/stm32 test/dsdl/housebus/heating/heating_status.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 1,
              "/home/user/Documents/PlatformIO/Projects/stm32 test/dsdl/housebus/heating/heating_status.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 0,
              "/home/user/Documents/PlatformIO/Projects/stm32 test/dsdl/housebus/heating/heating_status.1.0.uavcan is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.uavcan.org/t/choosing-message-and-service-ids/889
#define housebus_heating_heating_status_1_0_HAS_FIXED_PORT_ID_ false

#define housebus_heating_heating_status_1_0_FULL_NAME_             "housebus.heating.heating_status"
#define housebus_heating_heating_status_1_0_FULL_NAME_AND_VERSION_ "housebus.heating.heating_status.1.0"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define housebus_heating_heating_status_1_0_EXTENT_BYTES_                    37UL
#define housebus_heating_heating_status_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_ 37UL
static_assert(housebus_heating_heating_status_1_0_EXTENT_BYTES_ >= housebus_heating_heating_status_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    /// saturated uint1 valve_position_dhw
    uint8_t valve_position_dhw;

    /// saturated uint1 defrost_running
    uint8_t defrost_running;

    /// saturated uint1 external_heater
    uint8_t external_heater;

    /// saturated uint1 internal_heater
    uint8_t internal_heater;

    /// saturated uint1 error_type_h
    uint8_t error_type_h;

    /// saturated uint8 error_number
    uint8_t error_number;

    /// saturated int8 zone1_actual_temp
    int8_t zone1_actual_temp;

    /// saturated int8 dhw_actual_temp
    int8_t dhw_actual_temp;

    /// saturated int8 outdoor_temp
    int8_t outdoor_temp;

    /// saturated int8 return_temp
    int8_t return_temp;

    /// saturated int8 flow_temp
    int8_t flow_temp;

    /// saturated int8 zone1_flow_temp
    int8_t zone1_flow_temp;

    /// saturated int8 zone1_flow_target_temp
    int8_t zone1_flow_target_temp;

    /// saturated int8 heat_shift_target_temp
    int8_t heat_shift_target_temp;

    /// saturated int8 flow_target_temp
    int8_t flow_target_temp;

    /// saturated int8 discharge_temp
    int8_t discharge_temp;

    /// saturated int8 room_thermostat_temp
    int8_t room_thermostat_temp;

    /// saturated int8 indoor_pipe_temp
    int8_t indoor_pipe_temp;

    /// saturated int8 outdoor_pipe_temp
    int8_t outdoor_pipe_temp;

    /// saturated int8 defrost_temp
    int8_t defrost_temp;

    /// saturated int8 eva_outlet_temp
    int8_t eva_outlet_temp;

    /// saturated int8 bypass_outlet_temp
    int8_t bypass_outlet_temp;

    /// saturated int8 ipm_temp
    int8_t ipm_temp;

    /// saturated int8 high_pressure
    int8_t high_pressure;

    /// saturated int8 low_pressure
    int8_t low_pressure;

    /// saturated int8 compressor_current
    int8_t compressor_current;

    /// saturated uint8 compressor_frequency
    uint8_t compressor_frequency;

    /// saturated uint16 pump_flow
    uint16_t pump_flow;

    /// saturated uint8 pump_speed
    uint8_t pump_speed;

    /// saturated uint8 pump_duty
    uint8_t pump_duty;

    /// saturated uint8 fan1_rpm
    uint8_t fan1_rpm;

    /// saturated uint8 fan2_rpm
    uint8_t fan2_rpm;

    /// saturated uint16 compressor_starts
    uint16_t compressor_starts;

    /// saturated uint16 compressor_hours
    uint16_t compressor_hours;

    /// saturated uint16 heat_hours
    uint16_t heat_hours;

    /// saturated uint16 dhw_hours
    uint16_t dhw_hours;
} housebus_heating_heating_status_1_0;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see housebus_heating_heating_status_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t housebus_heating_heating_status_1_0_serialize_(
    const housebus_heating_heating_status_1_0* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 296UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;

    {   // saturated uint1 valve_position_dhw
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 1ULL) <= (capacity_bytes * 8U));
        uint8_t _sat0_ = obj->valve_position_dhw;
        if (_sat0_ > 1U)
        {
            _sat0_ = 1U;
        }
        buffer[offset_bits / 8U] = (uint8_t)(_sat0_);  // C std, 6.3.1.3 Signed and unsigned integers
        offset_bits += 1U;
    }

    {   // saturated uint1 defrost_running
        NUNAVUT_ASSERT((offset_bits + 1ULL) <= (capacity_bytes * 8U));
        uint8_t _sat1_ = obj->defrost_running;
        if (_sat1_ > 1U)
        {
            _sat1_ = 1U;
        }
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat1_, 1U);
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += 1U;
    }

    {   // saturated uint1 external_heater
        NUNAVUT_ASSERT((offset_bits + 1ULL) <= (capacity_bytes * 8U));
        uint8_t _sat2_ = obj->external_heater;
        if (_sat2_ > 1U)
        {
            _sat2_ = 1U;
        }
        const int8_t _err1_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat2_, 1U);
        if (_err1_ < 0)
        {
            return _err1_;
        }
        offset_bits += 1U;
    }

    {   // saturated uint1 internal_heater
        NUNAVUT_ASSERT((offset_bits + 1ULL) <= (capacity_bytes * 8U));
        uint8_t _sat3_ = obj->internal_heater;
        if (_sat3_ > 1U)
        {
            _sat3_ = 1U;
        }
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat3_, 1U);
        if (_err2_ < 0)
        {
            return _err2_;
        }
        offset_bits += 1U;
    }

    {   // saturated uint1 error_type_h
        NUNAVUT_ASSERT((offset_bits + 1ULL) <= (capacity_bytes * 8U));
        uint8_t _sat4_ = obj->error_type_h;
        if (_sat4_ > 1U)
        {
            _sat4_ = 1U;
        }
        const int8_t _err3_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, _sat4_, 1U);
        if (_err3_ < 0)
        {
            return _err3_;
        }
        offset_bits += 1U;
    }

    {   // saturated uint8 error_number
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err4_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->error_number, 8U);
        if (_err4_ < 0)
        {
            return _err4_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 zone1_actual_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err5_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->zone1_actual_temp, 8U);
        if (_err5_ < 0)
        {
            return _err5_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 dhw_actual_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err6_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->dhw_actual_temp, 8U);
        if (_err6_ < 0)
        {
            return _err6_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 outdoor_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err7_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->outdoor_temp, 8U);
        if (_err7_ < 0)
        {
            return _err7_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 return_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err8_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->return_temp, 8U);
        if (_err8_ < 0)
        {
            return _err8_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 flow_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err9_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->flow_temp, 8U);
        if (_err9_ < 0)
        {
            return _err9_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 zone1_flow_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err10_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->zone1_flow_temp, 8U);
        if (_err10_ < 0)
        {
            return _err10_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 zone1_flow_target_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err11_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->zone1_flow_target_temp, 8U);
        if (_err11_ < 0)
        {
            return _err11_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 heat_shift_target_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err12_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->heat_shift_target_temp, 8U);
        if (_err12_ < 0)
        {
            return _err12_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 flow_target_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err13_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->flow_target_temp, 8U);
        if (_err13_ < 0)
        {
            return _err13_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 discharge_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err14_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->discharge_temp, 8U);
        if (_err14_ < 0)
        {
            return _err14_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 room_thermostat_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err15_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->room_thermostat_temp, 8U);
        if (_err15_ < 0)
        {
            return _err15_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 indoor_pipe_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err16_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->indoor_pipe_temp, 8U);
        if (_err16_ < 0)
        {
            return _err16_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 outdoor_pipe_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err17_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->outdoor_pipe_temp, 8U);
        if (_err17_ < 0)
        {
            return _err17_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 defrost_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err18_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->defrost_temp, 8U);
        if (_err18_ < 0)
        {
            return _err18_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 eva_outlet_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err19_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->eva_outlet_temp, 8U);
        if (_err19_ < 0)
        {
            return _err19_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 bypass_outlet_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err20_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->bypass_outlet_temp, 8U);
        if (_err20_ < 0)
        {
            return _err20_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 ipm_temp
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err21_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->ipm_temp, 8U);
        if (_err21_ < 0)
        {
            return _err21_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 high_pressure
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err22_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->high_pressure, 8U);
        if (_err22_ < 0)
        {
            return _err22_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 low_pressure
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err23_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->low_pressure, 8U);
        if (_err23_ < 0)
        {
            return _err23_;
        }
        offset_bits += 8U;
    }

    {   // saturated int8 compressor_current
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err24_ = nunavutSetIxx(&buffer[0], capacity_bytes, offset_bits, obj->compressor_current, 8U);
        if (_err24_ < 0)
        {
            return _err24_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint8 compressor_frequency
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err25_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->compressor_frequency, 8U);
        if (_err25_ < 0)
        {
            return _err25_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint16 pump_flow
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err26_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->pump_flow, 16U);
        if (_err26_ < 0)
        {
            return _err26_;
        }
        offset_bits += 16U;
    }

    {   // saturated uint8 pump_speed
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err27_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->pump_speed, 8U);
        if (_err27_ < 0)
        {
            return _err27_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint8 pump_duty
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err28_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->pump_duty, 8U);
        if (_err28_ < 0)
        {
            return _err28_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint8 fan1_rpm
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err29_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->fan1_rpm, 8U);
        if (_err29_ < 0)
        {
            return _err29_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint8 fan2_rpm
        NUNAVUT_ASSERT((offset_bits + 8ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err30_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->fan2_rpm, 8U);
        if (_err30_ < 0)
        {
            return _err30_;
        }
        offset_bits += 8U;
    }

    {   // saturated uint16 compressor_starts
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err31_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->compressor_starts, 16U);
        if (_err31_ < 0)
        {
            return _err31_;
        }
        offset_bits += 16U;
    }

    {   // saturated uint16 compressor_hours
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err32_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->compressor_hours, 16U);
        if (_err32_ < 0)
        {
            return _err32_;
        }
        offset_bits += 16U;
    }

    {   // saturated uint16 heat_hours
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err33_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->heat_hours, 16U);
        if (_err33_ < 0)
        {
            return _err33_;
        }
        offset_bits += 16U;
    }

    {   // saturated uint16 dhw_hours
        NUNAVUT_ASSERT((offset_bits + 16ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- native representation matches the serialized representation.
        const int8_t _err34_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, obj->dhw_hours, 16U);
        if (_err34_ < 0)
        {
            return _err34_;
        }
        offset_bits += 16U;
    }

    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err35_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err35_ < 0)
        {
            return _err35_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits == 296ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t housebus_heating_heating_status_1_0_deserialize_(
    housebus_heating_heating_status_1_0* const out_obj, const uint8_t* const buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }

    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;

    // saturated uint1 valve_position_dhw
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    if ((offset_bits + 1U) <= capacity_bits)
    {
        out_obj->valve_position_dhw = buffer[offset_bits / 8U] & 1U;
    }
    else
    {
        out_obj->valve_position_dhw = 0U;
    }
    offset_bits += 1U;

    // saturated uint1 defrost_running
    out_obj->defrost_running = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 1);
    offset_bits += 1U;

    // saturated uint1 external_heater
    out_obj->external_heater = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 1);
    offset_bits += 1U;

    // saturated uint1 internal_heater
    out_obj->internal_heater = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 1);
    offset_bits += 1U;

    // saturated uint1 error_type_h
    out_obj->error_type_h = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 1);
    offset_bits += 1U;

    // saturated uint8 error_number
    out_obj->error_number = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 zone1_actual_temp
    out_obj->zone1_actual_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 dhw_actual_temp
    out_obj->dhw_actual_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 outdoor_temp
    out_obj->outdoor_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 return_temp
    out_obj->return_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 flow_temp
    out_obj->flow_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 zone1_flow_temp
    out_obj->zone1_flow_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 zone1_flow_target_temp
    out_obj->zone1_flow_target_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 heat_shift_target_temp
    out_obj->heat_shift_target_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 flow_target_temp
    out_obj->flow_target_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 discharge_temp
    out_obj->discharge_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 room_thermostat_temp
    out_obj->room_thermostat_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 indoor_pipe_temp
    out_obj->indoor_pipe_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 outdoor_pipe_temp
    out_obj->outdoor_pipe_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 defrost_temp
    out_obj->defrost_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 eva_outlet_temp
    out_obj->eva_outlet_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 bypass_outlet_temp
    out_obj->bypass_outlet_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 ipm_temp
    out_obj->ipm_temp = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 high_pressure
    out_obj->high_pressure = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 low_pressure
    out_obj->low_pressure = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated int8 compressor_current
    out_obj->compressor_current = nunavutGetI8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint8 compressor_frequency
    out_obj->compressor_frequency = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint16 pump_flow
    out_obj->pump_flow = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;

    // saturated uint8 pump_speed
    out_obj->pump_speed = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint8 pump_duty
    out_obj->pump_duty = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint8 fan1_rpm
    out_obj->fan1_rpm = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint8 fan2_rpm
    out_obj->fan2_rpm = nunavutGetU8(&buffer[0], capacity_bytes, offset_bits, 8);
    offset_bits += 8U;

    // saturated uint16 compressor_starts
    out_obj->compressor_starts = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;

    // saturated uint16 compressor_hours
    out_obj->compressor_hours = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;

    // saturated uint16 heat_hours
    out_obj->heat_hours = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;

    // saturated uint16 dhw_hours
    out_obj->dhw_hours = nunavutGetU16(&buffer[0], capacity_bytes, offset_bits, 16);
    offset_bits += 16U;

    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void housebus_heating_heating_status_1_0_initialize_(housebus_heating_heating_status_1_0* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = housebus_heating_heating_status_1_0_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}

#ifdef __cplusplus
}
#endif
#endif // HOUSEBUS_HEATING_HEATING_STATUS_1_0_INCLUDED_
