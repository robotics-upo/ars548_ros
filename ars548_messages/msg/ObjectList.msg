std_msgs/Header header
uint64 crc #Checksum (E2E Profile 7) (Reserved)
uint32 length #Len (E2E Profile 7) (Reserved)
uint32 sqc #SQC (E2E Profile 7) (Reserved)
uint32 dataid #Data ID (E2E Profile 7) (Reserved)
uint32 timestamp_nanoseconds #Timestamp Nanoseconds
uint32 timestamp_seconds #Timestamp Seconds
uint8 timestamp_syncstatus #Timestamp Sync Status
uint32 eventdataqualifier #(unused) Event Data Qualifier
uint8 extendedqualifier #(unused) Extended Qualifier 
uint8 objectlist_numofobjects #Number of Objects
Object[50] objectlist_objects #Object Array