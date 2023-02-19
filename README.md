# proformBLE
ESP32 BLE connectivity for the Proform Tour de France exercise bike.  I use it with Zwift and set cycling resistance based on the incline provided by Zwift.

Seeed XIAO Esp32-C3 board interfacing with the lower control board of a Proform Le Tour de France exercise bike (Model#PFEX0118C.0).  

Instantaneous Power (watts) and Instantaneous Cadence are sent via BLE from the ESP32 to a paired BLE phone or computer app.  Resistance can be set automatically by external software such as Zwift to give an impression of increased difficulty when cycling uphill.  The ESP32 receives the incline value and adjusts the flywheel magnets to increase or decrease resistance.  Resistance is adjusted manually using four buttons that interface with the ESP32 allowing the rider to "change" gears on the bike and make the effort easier or harder.  Incline or tilt of the bike is also automatically adjusted based on the incline provided by Zwift. Cadence is read from the bike's control board and using the current resistance setting a basic power (Watts) is calculated. The power calculation provides similar values to those calculated by the exercise bike at cadences from 60 to 100 rpm. 

The BLE connection can also be tested in NRF Connect to display power and cadence.

The ESP32 was programmed using the Arduino IDE. 
