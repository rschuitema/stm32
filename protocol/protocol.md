# HILT communication protocol

## Frame
![Framing](hilt_frame.png)

A frame consists of a message header, a message payload and a CRC.

**SYNC:** 4 byte ASCII representation of "hilt".

**LEN:** Payload length in bytes.

**Sequence:** A sequence number for the frame. It shall be incremented for each new frame that is sent. It shall wrap around from 0xFFFFFFFF to 0x00000000. 

**Protocol Version:** A field combining the fields Major Version and Minor Version.

**Major Version:** The major version number of the hilt protocol.

**Minor Version:** The minor version number of the hilt protocol.

**Message Type:** The type of message:

| value | message type |
| ----- | ------------ |
| 0     | action       |
| 1     | response     |
| 2     | event        |

**CRC:** CRC16 with polynomial "xyz" calculated over the header + payload.

## Message
![Message](hilt_message.png)


## GPIO Messages

### GPIO set pin
![set pin](gpio_set_pin.png)

### GPIO reset pin
![reset pin](gpio_reset_pin.png)

### GPIO read pin

### GPIO configure pin
![configure pin](gpio_configure_pin.png)

## I2C Messages

### Write register

### Read register

### Configure I2C bus
