#pragma once
#include <algorithm>
#include <vector>
#include <cstdint>
#include <cerrno>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <csignal>
#include <stdexcept>
#include <chrono>
#include <thread>

// Bounded newest-frame parser: a stalled consumer never accumulates video.
class MjpegFrames {
 public:
  void feed(const std::uint8_t* bytes, std::size_t size) {
    if (size > limit || buffer_.size() + size > limit) buffer_.clear();
    if (size > limit) return;
    buffer_.insert(buffer_.end(), bytes, bytes + size);
    for (;;) {
      auto start = marker(0xd8, 0);
      if (start == buffer_.size()) {
        if (buffer_.size() > 1) buffer_.erase(buffer_.begin(), buffer_.end()-1);
        return;
      }
      buffer_.erase(buffer_.begin(), buffer_.begin()+start);
      auto end = marker(0xd9, 2);
      if (end == buffer_.size()) return;
      latest_.assign(buffer_.begin(), buffer_.begin()+end+2);
      buffer_.erase(buffer_.begin(), buffer_.begin()+end+2);
    }
  }
  std::vector<std::uint8_t> take() { auto result=std::move(latest_); latest_.clear(); return result; }
  static constexpr std::size_t limit=4U*1024U*1024U;
 private:
  std::size_t marker(std::uint8_t second, std::size_t offset) const {
    for (auto i=offset;i+1<buffer_.size();++i)
      if (buffer_[i]==0xff && buffer_[i+1]==second) return i;
    return buffer_.size();
  }
  std::vector<std::uint8_t> buffer_,latest_;
};

class CsiCapture {
 public:
  ~CsiCapture() { close(); }
  CsiCapture()=default;
  CsiCapture(const CsiCapture&)=delete;
  CsiCapture& operator=(const CsiCapture&)=delete;
  void open(int index, int width, int height, int fps) {
    close();
    const auto id=std::to_string(index), w=std::to_string(width),
               h=std::to_string(height), rate=std::to_string(fps);
    int pipefd[2];
    if (::pipe2(pipefd,O_CLOEXEC)) throw std::runtime_error("CSI pipe failed");
    child_=::fork();
    if (child_==0) {
      ::dup2(pipefd[1],STDOUT_FILENO); ::close(pipefd[0]); ::close(pipefd[1]);
      ::execlp("rpicam-vid","rpicam-vid","--camera",id.c_str(),
        "--codec","mjpeg","--width",w.c_str(),"--height",h.c_str(),
        "--framerate",rate.c_str(),"--nopreview","--flush","--denoise","cdn_fast",
        "--quality","70","--timeout","0","--output","-",static_cast<char*>(nullptr));
      _exit(127);
    }
    ::close(pipefd[1]);
    if (child_<0) { ::close(pipefd[0]); throw std::runtime_error("CSI fork failed"); }
    fd_=pipefd[0];
    ::fcntl(fd_,F_SETFL,::fcntl(fd_,F_GETFL)|O_NONBLOCK);
    parser_=MjpegFrames{};
  }
  bool is_open() const { return fd_>=0; }
  bool read(cv::Mat& image) {
    pollfd p{fd_,POLLIN,0};
    const int ready=::poll(&p,1,100);
    if (ready<0 && errno!=EINTR) throw std::runtime_error("CSI poll failed");
    if (ready<=0) return false;
    std::uint8_t bytes[65536];
    for (int i=0;i<8;++i) {
      auto size=::read(fd_,bytes,sizeof(bytes));
      if (size==0) throw std::runtime_error("CSI process exited");
      if (size<0) {
        if (errno==EAGAIN || errno==EINTR) break;
        throw std::runtime_error("CSI read failed");
      }
      parser_.feed(bytes,static_cast<std::size_t>(size));
    }
    auto jpeg=parser_.take();
    if (jpeg.empty()) return false;
    image=cv::imdecode(jpeg,cv::IMREAD_COLOR);
    return !image.empty();
  }
  void close() {
    if (fd_>=0) { ::close(fd_); fd_=-1; }
    if (child_>0) {
      int status;
      ::kill(child_,SIGTERM);
      for (int i=0;i<50;++i) {
        if (::waitpid(child_,&status,WNOHANG)!=0) { child_=-1; return; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      ::kill(child_,SIGKILL); ::waitpid(child_,&status,0); child_=-1;
    }
  }
 private:
  int fd_{-1}; pid_t child_{-1}; MjpegFrames parser_;
};
