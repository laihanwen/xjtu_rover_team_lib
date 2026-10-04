#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
int main(int argc, char** argv) {
  if (argc < 2 || argc > 6) { std::cerr << "usage: auvctl [--socket PATH] status|start|pause|resume|abort|reset|disarm|arm --confirm SAFE_TO_ARM\n"; return 2; }
  std::string path = "/run/auv-runtime/control.sock";
  int arg = 1;
  if (std::string(argv[arg]) == "--socket" && argc > 3) { path = argv[2]; arg = 3; }
  std::string cmd = argv[arg];
  if (cmd == "arm") {
    if (argc != arg+3 || std::string(argv[arg+1]) != "--confirm" || std::string(argv[arg+2]) != "SAFE_TO_ARM") {
      std::cerr << "ARM requires --confirm SAFE_TO_ARM\n"; return 2;
    }
    cmd = "arm SAFE_TO_ARM";
  } else if (argc != arg+1) { std::cerr << "unexpected arguments\n"; return 2; }
  int fd = ::socket(AF_UNIX,SOCK_STREAM,0);
  sockaddr_un addr{}; addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) { std::cerr << "socket path too long\n"; return 2; }
  std::strncpy(addr.sun_path,path.c_str(),sizeof(addr.sun_path)-1);
  if (fd < 0 || ::connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))) { perror("control socket"); return 1; }
  ::write(fd,cmd.data(),cmd.size());
  char b[4096]{}; auto n = ::read(fd,b,sizeof(b));
  if (n <= 0) { std::cerr << "no response\n"; return 1; }
  std::string response(b,static_cast<std::size_t>(n)); std::cout << response;
  return response.rfind("ERR",0) == 0 ? 1 : 0;
}
