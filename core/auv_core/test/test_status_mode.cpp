#include "auv_core/status_decoder.hpp"
#include "auv_stm32_bridge/protocol.h"
#include <stdexcept>
static void require(bool b){if(!b)throw std::runtime_error("mode compatibility check failed");}
int main(){
  std::vector<std::uint8_t> bytes(30,0);auv_core::Stm32Status s;
  require(auv_core::decode_status(bytes,s)&&!s.dual_mode&&!s.autonomous_mode);
  bytes[4]=16;require(auv_core::decode_status(bytes,s)&&s.dual_mode&&!s.autonomous_mode&&!s.armed);
  bytes[4]=48;require(auv_core::decode_status(bytes,s)&&s.dual_mode&&s.autonomous_mode&&!s.armed);
  bytes[4]=49;require(auv_core::decode_status(bytes,s)&&s.armed);
  const std::uint8_t payload[5]={1,0,0,0,1};std::uint8_t output[64]{};
  require(auv_protocol_encode_frame(AUV_PROTOCOL_MSG_SELECT_MODE,payload,5,output,64)==12);
  require(output[3]==9&&output[9]==1);
}
