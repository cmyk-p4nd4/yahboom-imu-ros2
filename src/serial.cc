// custom header
#include "yahboom_imu_ros2/serial.h"
#include "yahboom_imu_ros2/circular_buffer.h"

// standard header
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <ios>
#include <iterator>
#include <list>
#include <numeric>
#include <system_error>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>

// Linux specific headers hidden from the rest of the application
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/epoll.h>

static std::string getString(std::vector<uint8_t> data);

static inline bool isChecksumValid(std::vector<uint8_t>& data) {
  const int chksum = std::reduce(data.cbegin(), data.cend() - 1, 0, std::plus<>());
  return static_cast<uint8_t>(chksum & 0xFF) == data.back();
}

Serial::Serial() : fd_(-1), epoll_fd_(-1), running_(false) {}

Serial::Serial(const std::string_view& port_name, int baudrate) : Serial() {
  if (!this->openPort(port_name, baudrate)) {  // canot open and config
    std::error_code error_code = std::make_error_code(std::errc(errno));
    throw std::system_error(error_code, error_code.message());
  }
}

Serial::~Serial() { this->closePort(); }

Serial::CallbackID Serial::subscribeOnReadMatches(const uint8_t byte, const Serial::ReadCallback cb) {
  CallbackID id = ++nextId_;
  read_callback_map_[byte][nextId_] = cb;
  return id;
}

Serial::CallbackID Serial::subscribeOnRead(Serial::ReadCallback cb) { return this->subscribeOnReadMatches(0xFF, cb); }

void Serial::subscribeOnError(Serial::ErrorCallback cb) { error_cb_ = std::move(cb); }

bool Serial::openPort(const std::string_view& port_name, int baudrate) {
  if (fd_ >= 0) return true;  // Already open

  // Open in non-blocking mode
  fd_ = open(port_name.data(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    this->outputError("Failed to open serial port: " + std::string(port_name));
    return false;
  }

  if (!this->configurePort(baudrate)) {
    this->closePort();
    return false;
  }

  // Setup epoll
  epoll_fd_ = epoll_create1(0);
  if (epoll_fd_ < 0) {
    this->outputError("Failed to create epoll instance.");
    this->closePort();
    return false;
  }

  // Initially, we only want to listen for incoming data (EPOLLIN)
  struct epoll_event ev;
  ev.events = EPOLLIN | EPOLLET;  // Using Edge-Triggered mode
  ev.data.fd = fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd_, &ev) < 0) {
    this->outputError("Failed to add fd to epoll.");
    this->closePort();
    return false;
  }

  return true;
}

void Serial::begin(void) {
  // Start the background event loop
  running_ = true;
  worker_thread_ = std::thread(&Serial::_threadFunc, this);
}

void Serial::end(void) {
  running_ = false;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  this->closePort();

  // try invoking callback one last time
  std::vector<std::vector<uint8_t>> packets;
  this->extractPackets(packets);
  for (const auto& packet : packets) {
    this->processCallbacks(packet);
  }

  // unbind all callbacks
  this->read_callback_map_.clear();
}

int Serial::getDeviceFD(void) const { return this->fd_; }

void Serial::closePort(void) {
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
  epoll_fd_ = -1;

  if (fd_ >= 0) {
    close(fd_);
  }
  fd_ = -1;

  // Clear pending writes
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::queue<std::vector<uint8_t>> empty;
  std::swap(write_queue_, empty);
}

void Serial::writeData(const std::vector<uint8_t>& data) {
  if (data.empty()) {
    return;
  }
  if (this->error_) {
    throw std::system_error(this->error_);
  }
  if (fd_ < 0) {
    throw std::system_error(std::make_error_code(std::errc::not_connected));
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  bool was_empty = write_queue_.empty();
  write_queue_.push(data);

  // If the queue was empty, we need to tell epoll we want to write now
  if (was_empty) {
    this->epollOut(true);
  }
}

bool Serial::opened(void) const { return fd_ > 0 && !this->error_; }

void Serial::extractPackets(std::vector<std::vector<uint8_t>>& packets) {
  const size_t PACKET_LENGTH{11};
  constexpr uint8_t SYNC_BYTE{0x55};

  // keep looping until we consume the whole buffer or are waiting for more
  while (!read_buffer_.empty()) {
    // 1st: check if there is a response header
    auto pos = read_buffer_.find(SYNC_BYTE);

    if (pos == read_buffer_.npos) {
      read_buffer_.reset();
      break;  // nothing to extract
    }

    // 2nd: check if this is complete frame
    size_t left_size = read_buffer_.size() - pos;
    if (left_size < PACKET_LENGTH) {
      return;  // frame incomplete, wait for more
    }

    // found complete frame
    if (pos > 0) {
      read_buffer_.discard_n(pos - 1);  // discard elements before it
    }

    // 3rd: check if checksum is correct
    std::vector<uint8_t> packet(PACKET_LENGTH);
    // extract the packet
    for (auto it = packet.begin(); it != packet.end(); it++) {
      *it = read_buffer_.pop_front();
    }

    if (!isChecksumValid(packet)) {
      std::printf("Checksum invalid!\n");
      continue;  // check sum invalid, discard the whole packet
    }

    // 4th: push valid packet
    packets.emplace_back(std::move(packet));
  }

  return;  // placeholder
}

void Serial::outputError(const std::string& msg) {
  if (error_cb_) {
    error_cb_(msg);
  }

  std::cout << msg << std::endl;
}

void Serial::epollOut(bool enable) {
  if (epoll_fd_ < 0 || fd_ < 0) {
    return;
  }
  struct epoll_event ev;
  // always read, add EPOLLOUT if we are writing
  ev.events = EPOLLIN | EPOLLET;
  if (enable) {
    ev.events |= EPOLLOUT;
  }
  ev.data.fd = fd_;
  epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd_, &ev);
}

bool Serial::configurePort(int baudrate) {
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    this->outputError("Error from tcgetattr");
    return false;
  }

  // Map standard integer baud rates to termios macros
  speed_t speed = B4800;
  switch (baudrate) {
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    case 460800:
      speed = B460800;
      break;
    case 921600:
      speed = B921600;
      break;
    default:
      this->outputError("Baudrate not supported");
      return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 Mode
  tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tty.c_cflag |= CS8 | CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    this->outputError("Error from tcsetattr");
    return false;
  }

  return true;
}

void Serial::processCallbacks(const std::vector<uint8_t>& packet) {
  if (packet.empty()) {
    this->outputError("Packet is invalid! Somethings wrong!");
    return;
  }

  // clang-format: off
  // The Yahboom IMU has the following Response Format
  // 0x55	TYPE DATA1L[7:0] DATA1H[15:8] DATA2L[7:0] DATA2H[15:8] DATA3L[7:0] DATA3H[15:8]
  // DATA4L[7:0] DATA4H[15:8] SUMCRC clang-fromat: on

  // std::cout << std::quoted(__FUNCTION__) << ": " << getString(packet) << "\r\n";

  // get callbacks based on response TYPE
  uint8_t p_type = packet.at(1);
  auto iter = read_callback_map_.find(p_type);
  if (iter != read_callback_map_.end()) {
    // invoke callbacks
    auto callbacks = iter->second;
    for (auto& cb : callbacks) {
      cb.second(packet);
    }
  }

  // however, all callbacks with type 0xFF will always be invoked
  iter = read_callback_map_.find(0xFF);
  if (iter != read_callback_map_.end()) {
    for (auto& cb : iter->second) {
      cb.second(packet);
    }
  }
}

void Serial::_threadFunc() noexcept {
  const int MAX_EVENTS = 10;
  struct epoll_event events[MAX_EVENTS];

  while (running_) {
    int n = epoll_wait(epoll_fd_, events, MAX_EVENTS, 100);

    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      this->outputError("epoll_wait failed");
      break;
    }

    for (int i = 0; i < n; ++i) {
      if (events[i].events & EPOLLIN) {
        this->_serialRead();
      }

      if (events[i].events & EPOLLOUT) {
        this->_serialWrite();
      }

      if (events[i].events & (EPOLLERR | EPOLLHUP)) {
        this->outputError("Serial port disconnected or hardware error.");
        running_ = false;
        error_ = std::error_code{errno, std::generic_category()};
        break;
      }
    }

    std::vector<std::vector<uint8_t>> packets;
    this->extractPackets(packets);
    for (const auto& packet : packets) {
      this->processCallbacks(packet);
    }
  }

  this->outputError("Thread Exitting ...");
  return;
}

[[maybe_unused]] static std::string getString(std::vector<uint8_t> data) {
  std::stringstream ss;
  for (size_t i = 0; i < data.size(); ++i) {
    ss << std::hex << std::uppercase;
    ss << std::setfill('0') << std::setw(2);
    ss << (int)data.at(i) << " ";
  }
  ss << '\b';
  return ss.str();
}

void Serial::_serialRead() noexcept {  // never throw
  uint8_t buffer[128];
  while (true) {
    ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
    if (bytes_read > 0 && !read_callback_map_.empty()) {
      // std::printf("Pushing %ld bytes: ", bytes_read);

      // std::cout << getString(
      //                  std::vector<uint8_t>{std::begin(buffer), std::begin(buffer) + bytes_read})
      //           << "\n\r";
      read_buffer_.push_back(std::begin(buffer), std::begin(buffer) + bytes_read);
    } else if (bytes_read == -1) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      } else {
        this->outputError("Read error occurred.");
        break;
      }
    } else {
      break;
    }
  }
}

void Serial::_serialWrite() noexcept {
  std::lock_guard<std::mutex> lock(write_mutex_);

  if (write_queue_.empty()) {
    this->epollOut(false);
    return;
  }

  auto& data = write_queue_.front();
  // std::cout << getString(data) << "\r\n";
  ssize_t bytes_written = write(fd_, data.data(), data.size());

  if (bytes_written > 0) {
    if (static_cast<size_t>(bytes_written) == data.size()) {
      write_queue_.pop();
    } else {
      data.erase(data.begin(), data.begin() + bytes_written);
    }
  } else if (bytes_written == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
    this->outputError("Write error occurred.");
  }

  if (write_queue_.empty()) {
    this->epollOut(false);
  }
}
