std_msgs/Header header
uint64 crc #Checksum (E2E Profile 7) (Reserved)
uint32 length #Len (E2E Profile 7) (Reserved)
uint32 sqc #SQC (E2E Profile 7) (Reserved)
uint32 dataid #Data ID (E2E Profile 7) (Reserved)
uint32 timestamp_nanoseconds #Timestamp Nanoseconds
uint32 timestamp_seconds #Timestamp Seconds
uint8 timestamp_syncstatus #Timestamp Sync Status
uint32 eventdataqualifier #Event Data Qualifier (unused)
uint8 extendedqualifier #Extended Qualifier (unused)
uint16 origin_invalidflags #Sensor Position Invalid flags (unused)
float32 origin_xpos #Sensor X Position with reference to rear axle
float32 origin_xstd #Sensor X Position STD (unused)
float32 origin_ypos #Sensor Y Position with reference to rear axle
float32 origin_ystd #Sensor Y Position STD (unused)
float32 origin_zpos #Sensor Z Position with reference to rear axle
float32 origin_zstd #Sensor Z Position STD (unused)
float32 origin_roll #Sensor Roll Angle (unused)
float32 origin_rollstd #Sensor Roll Angle STD (unused)
float32 origin_pitch #Sensor Pitch Angle with alignment correction
float32 origin_pitchstd #Sensor Pitch Angle STD
float32 origin_yaw #Sensor Yaw Angle with alignment correction
float32 origin_yawstd #Sensor Yaw Angle STD
uint8 list_invalidflags #Invalid flags (unused)
Detection[800] list_detections #Detection Array
float32 list_radveldomain_min #Ambiguity free Doppler velocity range Min
float32 list_radveldomain_max #Ambiguity free Doppler velocity range Max
uint32 list_numofdetections #Number of Detections
float32 aln_azimuthcorrection #Azimuth Alignment Correction
float32 aln_elevationcorrection #Elevation Alignment Correction
uint8 aln_status #Status of alignment