std_msgs/Header header

uint8 STATE_UNCALIBRATED=0
uint8 STATE_STOPPED=1
uint8 STATE_MOVING_CLOSE=2
uint8 STATE_CLOSED=3
uint8 STATE_MOVING_OPEN=4
uint8 STATE_OPENED=5
uint8 STATE_FAULT=6

bool connected
bool calibrated
uint8 state
uint16 current_pulse_us
uint16 target_pulse_us
uint32 error_flags
uint32 last_command_sequence
