uint32 timestamp_nanoseconds #Timestamp Nanoseconds
uint32 timestamp_seconds #Timestamp Seconds
uint8 timestamp_syncstatus #Timestamp Sync Status
uint8 swversion_major #Software version (major)
uint8 swversion_minor #Software version (minor)
uint8 swversion_patch #Software version (patch)
float32 longitudinal #Longitudinal sensor position (AUTOSAR)
float32 lateral #Lateral sensor position (AUTOSAR)
float32 vertical #Vertical sensor position (AUTOSAR)
float32 yaw #Sensor yaw angle (AUTOSAR)
float32 pitch #Sensor pitch angle (AUTOSAR)
uint8 plugorientation #Orientation of plug
float32 length #Vehicle length
float32 width #Vehicle width
float32 height #Vehicle height
float32 wheelbase #Vehicle wheelbase
uint16 maximundistance #Maximum detection distance
uint8 frequencyslot #Center frequency
uint8 cycletime #Cycle time
uint8 timeslot #Cycle offset
uint8 hcc #Country code
uint8 powersave_standstill #Power saving in standstill
uint32 sensoripaddress_0 #Sensor IP address
uint32 sensoripaddress_1 #Reserved
uint8 configurationcounter #Counter that counts up if new configuration has been received and accepted
uint8 status_longitudinalvelocity #Signals if current VDY is OK or timed out
uint8 status_longitudinalacceleration #Signals if current VDY is OK or timed out
uint8 status_lateralacceleration #Signals if current VDY is OK or timed out
uint8 status_yawrate #Signals if current VDY is OK or timed out
uint8 status_steeringangle #Signals if current VDY is OK or timed out
uint8 status_drivingdirection #Signals if current VDY is OK or timed out
uint8 status_characteristicspeed #(Unused) Signals if current VDY is OK or timed out.
uint8 status_radarstatus #Signals if Radar Status is OK
uint8 status_voltagestatus # Bitfield to report under- and overvoltage errors
uint8 status_temperaturestatus #Bitfield to report under- and overtemperature errors
uint8 status_blockagestatus #Current blockage state and blockage self test state.