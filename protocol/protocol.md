# HILT communication protocol
## Message flow
![Message flow](hilt_message_flow.png)

HILT has a simple straight forward message flow. All communication with HILT is asynchronous.
The protocol defines several types of messages:
* action
* response
* event
* acknowledge

An action has a unique sequence number and is acknowledged with the same
sequence number. When the action requires a response the response is given with the same 
sequence number. HILT also send events. These events have a unique identifier
that indicates what the event is about.

The physical format of the messages is described in the following sections.

## Frame
![Framing](hilt_frame.png)

A frame consists of a header, a payload and a CRC.
Where:

*SYNC:* 4 byte ASCII representation of "hilt".
*LEN:* Payload length in bytes.
*Sequence:* A sequence number for the frame. It shall be incremented for each new frame that is sent. It shall wrap around from 0xFFFFFFFF to 0x00000000. 
*Protocol Version:* A field combining the fields Major Version and Minor Version.
*Message Type:* The type of message:

| value | message type |
| ----- | ------------ |
| 0     | action       |
| 1     | response     |
| 2     | event        |
| 3     | acknowledge  |

*CRC:* CRC16 with polynomial "xyz" calculated over the header + payload.

## Message
![Message](hilt_message.png)


## GPIO Messages

### GPIO set pin
![set pin](gpio_set_pin.png)

### GPIO reset pin
![reset pin](gpio_reset_pin.png)

### GPIO configure pin
![configure pin](gpio_configure_pin.png)

## I2C Messages

### Write register

### Read register

### Configure I2C bus
