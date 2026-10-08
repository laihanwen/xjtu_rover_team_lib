std_msgs/Header header

uint8 IDLE=0
uint8 WAITING_FOR_ARM=1
uint8 RUNNING=2
uint8 COMPLETE=3
uint8 FAULT=4

uint8 state
string detail
uint32 route_revision
uint32 waypoint_index
int8 target_row
int8 target_col
float32 row_error
float32 col_error
