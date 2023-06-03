# Programming the configuration for SX1272 module

Firstly we send a combination of the command and the register
- if the address width is smaller than 8, we can combine the command and the register into one SPI transfer
- if the address width is greater than 8, we have to send two SPI transfer commands

If the action is read, then we send 0x00 to read each byte
If the action is write, then we send each byte to be written

The LORA/FSK mode can be changed only in the sleep mode (this is probably the reason why they are firstly reading the state and then modifying only the lower bits).