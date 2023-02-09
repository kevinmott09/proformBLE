# proformBLE
ESP32 BLE connectivity for the Proform Tour de France exercise bike.  I used it with Zwift and set resistance based on incline provided by Zwift.

Interfacing an Seeed XIAO Esp32-C3 board with the lower control board of a Proform Le Tour de France exercise bike (Model#PFEX0118C.0).  

Gears are adjusted manually using new buttons that interface with the ESP32.  Resistance (gears) can be set automatically by external software such as Zwift to give an impression of increased difficulty when cycling uphill.  Incline of the bike is also automatically adjusted based on the incline provided by Zwift. Cadence is read from the bike's control board and with the current resistance setting a basic power (Watts) is calculated.  Cadence is also sent to Zwift.
