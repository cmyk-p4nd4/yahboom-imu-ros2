#pragma once

#include "circular_buffer.h"

#include <atomic>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>
#include <map>

class Serial {
public:
  using ReadCallback = std::function<void(const std::vector<uint8_t>&)>;
  using ErrorCallback = std::function<void(const std::string&)>;

  using CallbackID = std::size_t;

  Serial();
  Serial(const std::string_view& port_name, int baudrate);
  Serial(Serial &&) = delete; // Serial port cannot be transferred
  ~Serial();

  // Event Listeners
  CallbackID subscribeOnReadMatches(const uint8_t byte, const ReadCallback cb);
  CallbackID subscribeOnRead(ReadCallback cb);
  void unsubscribeRead(CallbackID callbackId);
  void subscribeOnError(ErrorCallback cb);

  // Explicitly open and configure the port
  bool openPort(const std::string_view& port_name, int baudrate = 115200);
  void closePort(void);

  // Queue data for epoll to write asynchronously
  void writeData(const std::vector<uint8_t>& data);

  bool opened(void) const;

  // start serial event loop
  void begin(void);

  // stop serial event loop. Note that callback will be called one last time before exiting
  void end(void);

  int getDeviceFD(void) const;
  

protected:
  void extractPackets(std::vector<std::vector<uint8_t>>& packets);
  void processCallbacks(const std::vector<uint8_t>& packet);
  
  // Internal helper functions
  void outputError(const std::string& msg);
  void epollOut(bool enable);
  bool configurePort(int baudrate);

  // this holds a list of callbacks based on pattern matching
  std::map<uint8_t, std::map<uint32_t, ReadCallback>> read_callback_map_;
    
  ErrorCallback error_cb_;

private:
  // Background functions
  void _threadFunc() noexcept;
  void _serialRead() noexcept;
  void _serialWrite() noexcept;

  Serial(const Serial & ) = delete; // Serial port is non-copyable
  Serial& operator=(const Serial &) = delete; // Serial port is non-copyable
  
  int fd_;
  int epoll_fd_;
  std::error_code error_;

  
  std::atomic<bool> running_;
  std::thread worker_thread_;

  CallbackID nextId_ = 0;

  circular_buffer<uint8_t, 512> read_buffer_;
  std::queue<std::vector<uint8_t>> write_queue_;
  std::mutex write_mutex_;

};
