# ESP32-MAYHEM 
An alternative implementation designed to address compatibility issues with older hardware.

The primary goal is to leverage a TTGO device, utilizing its built-in Wi-Fi, GPS, and LoRaWAN capabilities. Additionally, the project aims to explore hardware modifications using ESP32 gadgets in conjunction with PortaPack.

This project is currently in its early stages and requires significant development. It serves as a proof-of-concept, featuring a custom implementation of an I2C slave driver. 

The project also involves reverse engineering to understand the commands and responses of the device, as the original documentation is lacking. This project aims to provide clarity and insights into the device's operation. Pure in C code.

## Status

-   Proof of Concept
-   Basic I2C slave communication established
-   Partial data display implemented
-   Includes a dummy application (a fully functional application will require compilation with the firmware)

## License

Published by @bitstuffing under the GPLv3 license for exclusive research and exploration purposes.