# MaixCam Pro visual detector

Deploy `main.py` to the MaixCam Pro and run it as the camera application.  It
starts in the original colour-block mode. UART0 remains configured as in the
supplied script: A17 is RX, A16 is TX, and the baud rate is 115200.

## UART protocol

The request keeps the original four-byte layout:

```
AA | reserved | command | BB
```

Colour responses keep the original seven-byte layout:

```
AA | result_type | center_x low | center_x high | center_y low | center_y high | BB
```

Colour `center_x` and `center_y` are unsigned 16-bit little-endian values in
the 320 x 240 camera image coordinate system. A no-result colour response is
`AA 00 00 00 00 00 BB`. Circle mode deliberately retains the existing
16-byte `ColorRecognize` response format: `AA | 10 | flags | frame_id(4B) |
cx(2B) | cy(2B) | radius(2B) | confidence | XOR | BB`.

| STM32 request | Meaning | Response |
| --- | --- | --- |
| `AA 00 F0 BB` | Switch to dashed-circle mode | No response. |
| `AA 00 F1 BB` | Switch to legacy colour mode | No response; this is the startup default. |
| `AA 00 10 BB` | Read outermost dashed-circle centre | 16-byte `ColorRecognize` response, type `10`. |
| `AA xx 01..06 BB` | Read legacy colour result | Unchanged from the supplied program; only valid in colour mode. |

In circle mode the program runs `find_circles` once every three display frames,
chooses the largest detected radius, and filters the centre/radius against the
previous result. It keeps a reliable result briefly when the circle segments
are covered or momentarily lost. The tuning constants at the top of `main.py`
are the intended adjustment points for a different lens distance or lighting.
