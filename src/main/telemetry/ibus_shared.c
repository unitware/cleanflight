/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */

/*
 * iBus Telemetry is a half-duplex serial protocol. It shares 1 line for
 * both TX and RX. It runs at a fixed baud rate of 115200. Queries are sent
 * every 7ms by the iBus receiver. Multiple sensors can be daisy chained with
 * ibus but this is implemented but not tested because i don't have one of the
 * sensors to test with
 *
 *     _______
 *    /       \                                             /---------\
 *    | STM32 |--UART TX-->[Bi-directional @ 115200 baud]<--| IBUS RX |
 *    |  uC   |--UART RX--x[not connected]                  \---------/
 *    \_______/
 *
 *
 * The protocol is driven by the iBus receiver, currently either an IA6B or
 * IA10. All iBus traffic is little endian. It begins with the iBus rx
 * querying for a sensor on the iBus:
 *
 *
 *  /---------\
 *  | IBUS RX | > Hello sensor at address 1, are you there?
 *  \---------/     [ 0x04, 0x81, 0x7A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0x81       - bits 7-4 Command (1000 = discover sensor)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x7A, 0xFF - Checksum, 0xFFFF - (0x04 + 0x81)
 *
 *
 * Due to the daisy-chaining, this hello also serves to inform the sensor
 * of its address (position in the chain). There are 16 possible addresses
 * in iBus, however address 0 is reserved for the rx's internally measured
 * voltage leaving 0x1 to 0xF remaining.
 *
 * Having learned it's address, the sensor simply echos the message back:
 *
 *
 *                                                      /--------\
 *                              Yes, i'm here, hello! < | Sensor |
 *                       [ 0x04, 0x81, 0x7A, 0xFF ]     \--------/
 *
 *     0x04, 0x81, 0x7A, 0xFF - Echo back received packet
 *
 *
 * On receipt of a response, the iBus rx next requests the sensor's type:
 *
 *
 *  /---------\
 *  | IBUS RX | > Sensor at address 1, what type are you?
 *  \---------/     [ 0x04, 0x91, 0x6A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0x91       - bits 7-4 Command (1001 = request sensor type)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x6A, 0xFF - Checksum, 0xFFFF - (0x04 + 0x91)
 *
 *
 * To which the sensor responds with its details:
 *
 *
 *                                                      /--------\
 *                              Yes, i'm here, hello! < | Sensor |
 *                [ 0x06 0x91 0x00 0x02 0x66 0xFF ]     \--------/
 *
 *     0x06       - Packet Length
 *     0x91       - bits 7-4 Command (1001 = request sensor type)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x00       - Measurement type (0 = internal voltage)
 *     0x02       - Unknown, always 0x02
 *     0x66, 0xFF - Checksum, 0xFFFF - (0x06 + 0x91 + 0x00 + 0x02)
 *
 *
 * The iBus rx continues the discovery process by querying the next
 * address. Discovery stops at the first address which does not respond.
 *
 * The iBus rx then begins a continual loop, requesting measurements from
 * each sensor discovered:
 *
 *
 *  /---------\
 *  | IBUS RX | > Sensor at address 1, please send your measurement
 *  \---------/     [ 0x04, 0xA1, 0x5A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0xA1       - bits 7-4 Command (1010 = request measurement)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x5A, 0xFF - Checksum, 0xFFFF - (0x04 + 0xA1)
 *
 *
 *                                                      /--------\
 *                                I'm reading 0 volts < | Sensor |
 *                [ 0x06 0xA1 0x00 0x00 0x5E 0xFF ]     \--------/
 *
 *     0x06       - Packet Length
 *     0xA1       - bits 7-4 Command (1010 = request measurement)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x00, 0x00 - The measurement
 *     0x58, 0xFF - Checksum, 0xFFFF - (0x06 + 0xA1 + 0x00 + 0x00)
 *
 *
 * Due to the limited telemetry data types possible with ibus, we
 * simply send everything which can be represented. Currently this
 * is voltage and temperature.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
// #include <string.h>

#include "platform.h"
//#include "common/utils.h"
#include "telemetry/telemetry.h"
#include "telemetry/ibus_shared.h"

static uint16_t calculateChecksum(const uint8_t *ibusPacket, size_t packetLength);


#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "sensors/battery.h"
#include "fc/rc_controls.h"

#ifdef BARO
#include "sensors/barometer.h"
#else
#include "fc/cleanflight_fc.h"
#endif


#define IBUS_TEMPERATURE_OFFSET  (400)

typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

typedef enum {
    IBUS_SENSOR_TYPE_TEMPERATURE      = 0x01,
    IBUS_SENSOR_TYPE_RPM              = 0x02,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE = 0x03
} ibusSensorType_e;

/* Address lookup relative to the sensor base address which is the lowest address seen by the FC 
   The actual lowest value is likely to change when sensors are daisy chained */
static const uint8_t sensorAddressTypeLookup[] = {
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
    IBUS_SENSOR_TYPE_TEMPERATURE,
    IBUS_SENSOR_TYPE_RPM
};

static serialPort_t *ibusSerialPort = NULL;

#define INVALID_IBUS_ADDRESS 0
static ibusAddress_t ibusBaseAddress = INVALID_IBUS_ADDRESS;



static uint8_t transmitIbusPacket(uint8_t *ibusPacket, size_t payloadLength)
{
    uint16_t checksum = calculateChecksum(ibusPacket, payloadLength + IBUS_CHECKSUM_SIZE);
    for (size_t i = 0; i < payloadLength; i++) {
        serialWrite(ibusSerialPort, ibusPacket[i]);
    }
    serialWrite(ibusSerialPort, checksum & 0xFF);
    serialWrite(ibusSerialPort, checksum >> 8);
    return payloadLength + IBUS_CHECKSUM_SIZE;
}

static uint8_t sendIbusDiscoverSensorReply(ibusAddress_t address)
{
    uint8_t sendBuffer[] = { 0x04, IBUS_COMMAND_DISCOVER_SENSOR | address};
    return transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static uint8_t sendIbusSensorType(ibusAddress_t address)
{
    uint8_t sendBuffer[] = {0x06,
                            IBUS_COMMAND_SENSOR_TYPE | address,
                            sensorAddressTypeLookup[address - ibusBaseAddress],
                            0x02
                           };
    return transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static uint8_t sendIbusMeasurement(ibusAddress_t address, uint16_t measurement)
{
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_MEASUREMENT | address, measurement & 0xFF, measurement >> 8};
    return transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static bool isCommand(ibusCommand_e expected, const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0x0F);
}

static uint8_t dispatchMeasurementReply(ibusAddress_t address)
{
    int value;
    
    switch (sensorAddressTypeLookup[address - ibusBaseAddress]) {
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        value = getVbat() * 10;
        if (telemetryConfig()->report_cell_voltage) {
            value /= batteryCellCount;
        }
        return sendIbusMeasurement(address, value);

    case IBUS_SENSOR_TYPE_TEMPERATURE:
        #ifdef BARO
            value = (baro.baroTemperature + 5) / 10; // +5 to make integer division rounding correct
        #else
            value = telemTemperature1 * 10;
        #endif
        return sendIbusMeasurement(address, value + IBUS_TEMPERATURE_OFFSET);

    case IBUS_SENSOR_TYPE_RPM:
        return sendIbusMeasurement(address, (uint16_t) rcCommand[THROTTLE]);
    }
    return 0;
}

static void autodetectFirstReceivedAddressAsBaseAddress(ibusAddress_t returnAddress)
{
    if ((INVALID_IBUS_ADDRESS == ibusBaseAddress) &&
        (INVALID_IBUS_ADDRESS != returnAddress)) {
        ibusBaseAddress = returnAddress;
    }
}

static bool theAddressIsWithinOurRange(ibusAddress_t returnAddress)
{
    return (returnAddress >= ibusBaseAddress) &&
           (ibusAddress_t)(returnAddress - ibusBaseAddress) < ARRAYLEN(sensorAddressTypeLookup);
}

uint8_t respondToIbusRequest(uint8_t const * const ibusPacket)
{
    ibusAddress_t returnAddress = getAddress(ibusPacket);
    autodetectFirstReceivedAddressAsBaseAddress(returnAddress);

    if (theAddressIsWithinOurRange(returnAddress)) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            return sendIbusDiscoverSensorReply(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            return sendIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            return dispatchMeasurementReply(returnAddress);
        }
    }

    return 0;
}


void initSharedIbusTelemetry(serialPort_t *port)
{
    ibusSerialPort = port;
    ibusBaseAddress = INVALID_IBUS_ADDRESS;
}


#endif //defined(TELEMETRY) && defined(TELEMETRY_IBUS)

static uint16_t calculateChecksum(const uint8_t *ibusPacket, size_t packetLength)
{
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < packetLength - IBUS_CHECKSUM_SIZE; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

bool isChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = calculateChecksum(ibusPacket, length);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}

