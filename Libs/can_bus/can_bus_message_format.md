# CAN Bus Message Format

This document defines the 11-bit CAN identifier format used for project CAN messages.

## 11-bit CAN ID Layout

The CAN ID is split into three fields:

```text
Bit index:  10  9  8   7  6  5  4   3  2  1  0
            +---------+-----------+-------------+
Field:      |Priority | Msg Type  | Destination |
            +---------+-----------+-------------+
Width:      | 3 bits  |  4 bits   |   4 bits    |
```

| Bits    | Width | Field                 | Description |
|---------|-------|-----------------------|-------------|
| 10..8   | 3     | Priority              | Message priority. Lower numeric value means higher priority on CAN arbitration. |
| 7..4    | 4     | Message Type          | Defines the type/purpose of the message. Up to 16 message types are possible. |
| 3..0    | 4     | Destination Device ID | Target device ID. Up to 16 destination IDs are possible. |

## CAN ID Formula

```c
CAN_ID = ((priority & 0x7U) << 8) |
         ((message_type & 0xFU) << 4) |
         ((destination_id & 0xFU) << 0);
```

## Field Masks

```c
#define CAN_ID_PRIORITY_MASK      0x700U
#define CAN_ID_MESSAGE_TYPE_MASK  0x0F0U
#define CAN_ID_DESTINATION_MASK   0x00FU

#define CAN_ID_PRIORITY_SHIFT     8U
#define CAN_ID_MESSAGE_TYPE_SHIFT 4U
#define CAN_ID_DESTINATION_SHIFT  0U
```

## Broadcast ID

Destination device ID `0xF` is reserved as the broadcast destination.

```c
#define CAN_DEST_BROADCAST_ID 0xFU
```

Any message with destination field `0xF` should be treated as a broadcast message by all devices.

Example broadcast CAN ID:

```c
// Priority: VERY_HIGH = 0x0
// Message Type: HEARTBEAT = 0x1
// Destination: BROADCAST = 0xF
// CAN ID = 0x01F
```

Note: The raw destination broadcast ID is `0xF`. A complete 11-bit CAN ID containing a broadcast destination depends on the priority and message type fields.

## Priority Levels

CAN arbitration gives priority to lower numeric CAN IDs. Since the priority field is stored in bits 10..8, lower priority values win arbitration first.

```c
typedef enum
{
    CAN_Priority_VERY_HIGH = 0x0U,
    CAN_Priority_HIGH      = 0x1U,
    CAN_Priority_MEDIUM    = 0x2U,
    CAN_Priority_LOW       = 0x3U,
    CAN_Priority_VERY_LOW  = 0x4U
} CAN_Priority_t;
```

Available priority field range: `0x0` to `0x7`.

Currently defined priority levels use `0x0` to `0x4`; values `0x5` to `0x7` are reserved.

## Message Types

The message type field is 4 bits wide, so up to 16 message types are possible: `0x0` to `0xF`.

Currently assigned message types:

```c
typedef enum
{
    CAN_ID_HEARTBEAT   = 0x1U,
    CAN_ID_STATUS      = 0x2U,
    CAN_ID_COMMAND     = 0x3U,
    CAN_ID_LED_COMMAND = 0x4U,
    CAN_ID_ADC_REPORT  = 0x5U,
    CAN_ID_DEBUG       = 0x6U
} CAN_MessageType_t;
```

| Value | Message Type        | Description |
|-------|---------------------|-------------|
| 0x0   | Reserved            | Reserved for future use or invalid/null message type. |
| 0x1   | HEARTBEAT           | Periodic node-alive message. |
| 0x2   | STATUS              | Device status or health report. |
| 0x3   | COMMAND             | General command message. |
| 0x4   | LED_COMMAND         | LED control command. |
| 0x5   | ADC_REPORT          | ADC measurement/report message. |
| 0x6   | DEBUG               | Debug or diagnostic message. |
| 0x7   | TBD                 | Reserved. |
| 0x8   | TBD                 | Reserved. |
| 0x9   | TBD                 | Reserved. |
| 0xA   | TBD                 | Reserved. |
| 0xB   | TBD                 | Reserved. |
| 0xC   | TBD                 | Reserved. |
| 0xD   | TBD                 | Reserved. |
| 0xE   | TBD                 | Reserved. |
| 0xF   | TBD                 | Reserved. |

## Destination Device IDs

The destination field is 4 bits wide, so up to 16 destination values are possible: `0x0` to `0xF`.

Recommended usage:

| Value | Destination |
|-------|-------------|
| 0x0   | Reserved or master/controller node |
| 0x1   | Device 1 |
| 0x2   | Device 2 |
| 0x3   | Device 3 |
| 0x4   | Device 4 |
| 0x5   | Device 5 |
| 0x6   | Device 6 |
| 0x7   | Device 7 |
| 0x8   | Device 8 |
| 0x9   | Device 9 |
| 0xA   | Device 10 |
| 0xB   | Device 11 |
| 0xC   | Device 12 |
| 0xD   | Device 13 |
| 0xE   | Device 14 |
| 0xF   | Broadcast to all devices |

## Helper Macros

```c
#define CAN_BUILD_ID(priority, message_type, destination_id) \
    ((((uint16_t)(priority)      & 0x7U) << CAN_ID_PRIORITY_SHIFT) | \
     (((uint16_t)(message_type)  & 0xFU) << CAN_ID_MESSAGE_TYPE_SHIFT) | \
     (((uint16_t)(destination_id)& 0xFU) << CAN_ID_DESTINATION_SHIFT))

#define CAN_GET_PRIORITY(can_id) \
    (((uint16_t)(can_id) & CAN_ID_PRIORITY_MASK) >> CAN_ID_PRIORITY_SHIFT)

#define CAN_GET_MESSAGE_TYPE(can_id) \
    (((uint16_t)(can_id) & CAN_ID_MESSAGE_TYPE_MASK) >> CAN_ID_MESSAGE_TYPE_SHIFT)

#define CAN_GET_DESTINATION(can_id) \
    (((uint16_t)(can_id) & CAN_ID_DESTINATION_MASK) >> CAN_ID_DESTINATION_SHIFT)

#define CAN_IS_BROADCAST(can_id) \
    (CAN_GET_DESTINATION(can_id) == CAN_DEST_BROADCAST_ID)
```

## Example CAN IDs

| Priority | Message Type | Destination | CAN ID |
|----------|--------------|-------------|--------|
| VERY_HIGH `0x0` | HEARTBEAT `0x1` | Broadcast `0xF` | `0x01F` |
| HIGH `0x1`      | STATUS `0x2`    | Device 1 `0x1`  | `0x121` |
| MEDIUM `0x2`    | COMMAND `0x3`   | Device 2 `0x2`  | `0x232` |
| LOW `0x3`       | LED_COMMAND `0x4` | Device 3 `0x3` | `0x343` |
| VERY_LOW `0x4`  | ADC_REPORT `0x5` | Device 4 `0x4` | `0x454` |
| VERY_HIGH `0x0` | DEBUG `0x6`     | Device 1 `0x1`  | `0x061` |

## Recommended Message Payload Notes

The 11-bit CAN ID only identifies priority, message type, and destination. The data payload should be defined separately for each message type.

Recommended payload rules:

1. Use standard CAN data length from 0 to 8 bytes, unless CAN FD is explicitly used.
2. Define byte order for multi-byte values. Recommended: little-endian unless the project requires otherwise.
3. Keep payload layouts fixed per message type where possible.
4. Include a sequence counter or timestamp in payloads only if required by the application.
5. For command messages, define whether an acknowledgement/status response is required.
6. For broadcast messages, avoid responses from all nodes at the same time unless a response delay or polling scheme is used.

## Suggested Payload Definitions

These are suggested starting points and can be changed as needed.

### HEARTBEAT

Purpose: Periodic alive message from a device.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | Status flags |
| 2..3 | Uptime counter or heartbeat counter, uint16 |

### STATUS

Purpose: Report device state, error flags, or health information.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | Device state |
| 2..3 | Error flags, uint16 |
| 4..5 | Optional measurement or diagnostic value |

### COMMAND

Purpose: General command message.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | Command ID |
| 2..7 | Command parameters |

### LED_COMMAND

Purpose: Control LEDs on the destination device.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | LED index or channel |
| 2    | LED mode |
| 3    | Brightness, 0 to 255 |
| 4    | Red value, 0 to 255 |
| 5    | Green value, 0 to 255 |
| 6    | Blue value, 0 to 255 |
| 7    | Reserved |

### ADC_REPORT

Purpose: Report ADC reading from a device.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | ADC channel |
| 2..3 | ADC raw value, uint16 |
| 4..5 | Optional converted value, uint16 |
| 6..7 | Optional timestamp/counter, uint16 |

### DEBUG

Purpose: Debug or diagnostic data.

Suggested payload:

| Byte | Field |
|------|-------|
| 0    | Source device ID |
| 1    | Debug code |
| 2..7 | Debug data |

## Implementation Notes

1. Use standard 11-bit CAN IDs, not extended 29-bit CAN IDs.
2. All CAN IDs must be less than or equal to `0x7FF`.
3. Lower numeric CAN IDs have higher bus arbitration priority.
4. Since priority is placed in the most significant bits of the 11-bit ID, priority affects arbitration strongly.
5. Destination ID `0xF` is reserved for broadcast messages.
6. Message type `0x0` is recommended to remain reserved.
7. Priority values `0x5`, `0x6`, and `0x7` are currently reserved.
8. Define source device ID in the payload if receivers need to know who sent the message.
9. Avoid many devices replying immediately to the same broadcast command, because that can cause bus congestion.
10. Keep message periods documented to prevent CAN bus overload.

## Quick Reference

```c
// Build CAN ID
uint16_t id = CAN_BUILD_ID(CAN_Priority_HIGH, CAN_ID_STATUS, 0x1U);

// Decode CAN ID
uint8_t priority    = CAN_GET_PRIORITY(id);
uint8_t msg_type    = CAN_GET_MESSAGE_TYPE(id);
uint8_t destination = CAN_GET_DESTINATION(id);

// Check broadcast
if (CAN_IS_BROADCAST(id))
{
    // Handle broadcast message
}
```

## Copy-Paste Header Template

```c
#ifndef CAN_MESSAGE_FORMAT_H
#define CAN_MESSAGE_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_ID_PRIORITY_MASK      0x700U
#define CAN_ID_MESSAGE_TYPE_MASK  0x0F0U
#define CAN_ID_DESTINATION_MASK   0x00FU

#define CAN_ID_PRIORITY_SHIFT     8U
#define CAN_ID_MESSAGE_TYPE_SHIFT 4U
#define CAN_ID_DESTINATION_SHIFT  0U

#define CAN_DEST_BROADCAST_ID     0xFU

typedef enum
{
    CAN_Priority_VERY_HIGH = 0x0U,
    CAN_Priority_HIGH      = 0x1U,
    CAN_Priority_MEDIUM    = 0x2U,
    CAN_Priority_LOW       = 0x3U,
    CAN_Priority_VERY_LOW  = 0x4U
} CAN_Priority_t;

typedef enum
{
    CAN_ID_HEARTBEAT   = 0x1U,
    CAN_ID_STATUS      = 0x2U,
    CAN_ID_COMMAND     = 0x3U,
    CAN_ID_LED_COMMAND = 0x4U,
    CAN_ID_ADC_REPORT  = 0x5U,
    CAN_ID_DEBUG       = 0x6U
} CAN_MessageType_t;

#define CAN_BUILD_ID(priority, message_type, destination_id) \
    ((((uint16_t)(priority)       & 0x7U) << CAN_ID_PRIORITY_SHIFT) | \
     (((uint16_t)(message_type)   & 0xFU) << CAN_ID_MESSAGE_TYPE_SHIFT) | \
     (((uint16_t)(destination_id) & 0xFU) << CAN_ID_DESTINATION_SHIFT))

#define CAN_GET_PRIORITY(can_id) \
    (((uint16_t)(can_id) & CAN_ID_PRIORITY_MASK) >> CAN_ID_PRIORITY_SHIFT)

#define CAN_GET_MESSAGE_TYPE(can_id) \
    (((uint16_t)(can_id) & CAN_ID_MESSAGE_TYPE_MASK) >> CAN_ID_MESSAGE_TYPE_SHIFT)

#define CAN_GET_DESTINATION(can_id) \
    (((uint16_t)(can_id) & CAN_ID_DESTINATION_MASK) >> CAN_ID_DESTINATION_SHIFT)

#define CAN_IS_BROADCAST(can_id) \
    (CAN_GET_DESTINATION(can_id) == CAN_DEST_BROADCAST_ID)

#endif // CAN_MESSAGE_FORMAT_H
```
