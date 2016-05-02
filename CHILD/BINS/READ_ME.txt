Use TI Flash Programmer 2 to flash firmwares.
Stack firmware is added for convenience. Both kind of nodes (CHILD and MASTER) uses the same stack.
The child node id is kept if the only application is flashed, if the both firmwares (stack and application) are flashed the id is reset to 0 and must be reassigned.

To reassign an Id press for more than 3 seconds both buttons, the device will become connectible. Connect it using a standard ble scanner (such as: nRF master control panel), discover all services/characteristics and write "01-xx" (without quotes, where xx is the id in hex) in PICO characteristic 