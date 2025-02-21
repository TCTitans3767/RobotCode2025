# Arduino Code

## BasicPromotionalLights
Solid green lights for testing and simple promotional if main code doesn't work. Pushes the green to 4 LED strips that are each 128 LEDs long. Meant for WS2811 12V LED strips.

### Config
| Arduino Pin | Connection |
|:---|:---|
| GND | Robot ground |
| VIN | 5V supply |
| 2 | Data in on a light strip |
| 3 | Data in on a light strip |
| 4 | Data in on a light strip |
| 5 | Data in on a light strip |

**Also connect all grounds and 12Vs of the light strips as needed.**

### Requires
- FastLED (3.9.13) - Daniel Garcia