# Three-circle vision protocol

UART wiring and baud rate are unchanged:

- MaixCam A16 TX -> STM32 PE7 RX
- MaixCam A17 RX <- STM32 PE8 TX
- 115200 baud, 8-N-1

## Commands

All commands are four bytes.

| Frame | Meaning |
| --- | --- |
| `AA 00 F2 BB` | Select the dedicated three-circle vision mode. No reply. |
| `AA 00 31 BB` | Request the latest three-target result. |

## Result packet

The response is always 30 bytes. Multi-byte values are little-endian.

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 1 | Header `AA` |
| 1 | 1 | Type `31` |
| 2 | 2 | Camera frame sequence |
| 4 | 1 | Flags: bit 0 valid, bit 1 contains inference, bit 2 contains fusion |
| 5 | 1 | Overall board quality, 0-100 |
| 6 | 7 | Target 1 record |
| 13 | 7 | Target 2 record |
| 20 | 7 | Target 3 record |
| 27 | 2 | Modbus CRC16 over bytes 1 through 26 |
| 29 | 1 | Tail `BB` |

Each target record is `number:u8, source:u8, x:i16, y:i16, confidence:u8`.
Signed coordinates are intentional: an inferred target may lie outside the
current 320 x 240 image while still being usable by the calibrated arm model.

Source values:

| Value | Meaning |
| ---: | --- |
| 0 | Missing/invalid |
| 1 | Directly measured numbered circle |
| 2 | Inferred from two reliable numbered circles |
| 3 | Direct fragment fused with the two-circle geometric prediction |

The controller must require `valid=1`, quality at least the configured
threshold, target numbers exactly `1,2,3`, a valid CRC, and a fresh timestamp
before allowing any arm motion.
