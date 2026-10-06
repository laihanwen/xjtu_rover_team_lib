#include "../src/csi_capture.hpp"
#include <cassert>
int main() {
  MjpegFrames parser;
  const std::uint8_t first[]={1,2,0xff};
  const std::uint8_t second[]={0xd8,3,0xff,0xd9,0xff,0xd8,4,0xff,0xd9};
  parser.feed(first,sizeof(first)); assert(parser.take().empty());
  parser.feed(second,sizeof(second));
  assert(parser.take()==std::vector<std::uint8_t>({0xff,0xd8,4,0xff,0xd9}));
  assert(parser.take().empty());
  std::vector<std::uint8_t> oversized(MjpegFrames::limit+1,0xff);
  parser.feed(oversized.data(),oversized.size());
  parser.feed(second+4,5);
  assert(parser.take()==std::vector<std::uint8_t>({0xff,0xd8,4,0xff,0xd9}));
}
