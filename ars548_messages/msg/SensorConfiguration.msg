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
uint16 maximumdistance #Maximum detection distance
uint8 frequencslot #Center frequency (if MaximumDistance < 190 m only Mid can be selected)
uint8 cycletime #Cycle time
uint8 timeslot #Cycle offset
uint8 hcc #Country code
uint8 powersave_standstill #Power saving in standstill
uint32 sensoripaddress_0 #Sensor IP address
uint32 sensoripaddress_1 #Reserved (Configure as 2852025457 or 169.254.116.113)
uint8 newsensormounting #Flag if new sensor mounting position shall be configured
uint8 newvehicleparameters #Flag if new vehicle parameters position shall be configured
uint8 newradarparameters #Flag if new radar parameter shall be configured
uint8 newnetworkconfiguration # Flag if new IP address shall be configured
