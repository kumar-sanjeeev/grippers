// Copyright (c) 2023 PickNik, Inc.
// Copyright (c) 2026 Robotiq, Inc. (libserialport rewrite)
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <Robotiq/detail/default_serial.hpp>

#include <libserialport.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <string>
#include <utility>

#ifdef __linux__
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <Robotiq/detail/default_logger.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

namespace Robotiq::detail {
namespace {
//! Render a libserialport return code as a message and free any error state.
std::string describeSpError(sp_return result)
{
   if(result == SP_ERR_FAIL)
   {
      char* message = sp_last_error_message();
      std::string text = message != nullptr ? message : "unknown OS error";
      sp_free_error_message(message);
      return text;
   }
   switch(result)
   {
   case SP_ERR_ARG:
      return "invalid argument";
   case SP_ERR_MEM:
      return "memory allocation failure";
   case SP_ERR_SUPP:
      return "operation not supported";
   default:
      return "error " + std::to_string(result);
   }
}

//! Throw SerialIOException when a libserialport call fails.
void check(sp_return result, const std::string& context)
{
   if(result < SP_OK)
   {
      throw SerialIOException(context + ": " + describeSpError(result));
   }
}

#ifdef __linux__
std::string describeErrno()
{
   return std::strerror(errno);
}

bool isLinuxPty(const std::string& path)
{
   char resolved[PATH_MAX];
   if(realpath(path.c_str(), resolved) == nullptr)
   {
      return false;
   }
   constexpr const char prefix[] = "/dev/pts/";
   return std::strncmp(resolved, prefix, sizeof(prefix) - 1) == 0;
}

void configurePty(int fd, const std::string& path)
{
   termios settings{};
   if(tcgetattr(fd, &settings) != 0)
   {
      throw SerialIOException("reading PTY settings for '" + path + "': " + describeErrno());
   }
   cfmakeraw(&settings);
   settings.c_cflag |= CLOCAL | CREAD;
   settings.c_cflag &= ~CSTOPB;
   settings.c_cflag &= ~PARENB;
   settings.c_cflag &= ~CSIZE;
   settings.c_cflag |= CS8;
   settings.c_cc[VMIN] = 0;
   settings.c_cc[VTIME] = 0;
   if(tcsetattr(fd, TCSANOW, &settings) != 0)
   {
      throw SerialIOException("configuring PTY '" + path + "': " + describeErrno());
   }
}

int pollFd(int fd, short events, std::chrono::milliseconds timeout, const std::string& operation)
{
   pollfd descriptor{fd, events, 0};
   int result;
   do
   {
      result = poll(&descriptor, 1, static_cast<int>(timeout.count()));
   } while(result < 0 && errno == EINTR);
   if(result < 0)
   {
      throw SerialIOException(operation + ": " + describeErrno());
   }
   if(result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
   {
      throw SerialIOException(operation + ": PTY disconnected");
   }
   return result;
}
#endif

} // namespace

std::string deviceBasename(const std::string& port)
{
   const auto slash = port.find_last_of('/');
   return slash == std::string::npos ? port : port.substr(slash + 1);
}

DefaultSerial::DefaultSerial(SerialConfig config, std::shared_ptr<Logger> logger)
   : _config(std::move(config))
   , _logger(logger ? std::move(logger) : makeDefaultLogger())
{
}

DefaultSerial::~DefaultSerial()
{
   DefaultSerial::close();
}

void DefaultSerial::open()
{
   if(isOpen())
   {
      _logger->log(Logger::Level::Debug,
                   "open() called on " + _config.port + " but the port is already open; ignoring.");
      return;
   }
   if(_config.port.empty())
   {
      throw SerialIOException("Cannot open serial port: port path is empty");
   }
   _logger->log(Logger::Level::Debug,
                "opening serial port '" + _config.port + "' at " + std::to_string(_config.baudrate) + " baud");

   struct sp_port* handle = nullptr;
   if(sp_get_port_by_name(_config.port.c_str(), &handle) != SP_OK)
   {
#ifdef __linux__
      if(isLinuxPty(_config.port))
      {
         const int fd = ::open(_config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
         if(fd < 0)
         {
            throw SerialIOException("opening PTY '" + _config.port + "': " + describeErrno());
         }
         try
         {
            configurePty(fd, _config.port);
         }
         catch(...)
         {
            ::close(fd);
            throw;
         }
         _ptyFd = fd;
         _logger->log(Logger::Level::Info,
                      "opened Linux PTY '" + _config.port + "' (serial line settings are managed by its bridge).");
         return;
      }
#endif
      throw SerialIOException("no serial port named '" + _config.port + "' (is the device connected?)");
   }
   _portHandle = handle;
   try
   {
      check(sp_open(_portHandle, SP_MODE_READ_WRITE), "opening serial port '" + _config.port + "'");
      check(sp_set_baudrate(_portHandle, static_cast<int>(_config.baudrate)), "setting baud rate");
      check(sp_set_bits(_portHandle, 8), "setting data bits");
      check(sp_set_parity(_portHandle, SP_PARITY_NONE), "setting parity");
      check(sp_set_stopbits(_portHandle, 1), "setting stop bits");
      check(sp_set_flowcontrol(_portHandle, SP_FLOWCONTROL_NONE), "setting flow control");
   }
   catch(...)
   {
      sp_free_port(_portHandle);
      _portHandle = nullptr;
      throw;
   }

#ifdef __linux__
   if(_config.latencyTimerMs > 0)
   {
      if(applyLatencyTimer())
      {
         _logger->log(Logger::Level::Info,
                      "FTDI latency_timer set to " + std::to_string(_config.latencyTimerMs) + " ms on " + _config.port
                         + ".");
      }
      else
      {
         _logger->log(Logger::Level::Warn,
                      "Could not set FTDI latency_timer to " + std::to_string(_config.latencyTimerMs) + " ms on "
                         + _config.port
                         + ". Modbus cycle latency may be ~3x higher than expected (the kernel default "
                           "is 16 ms). Either run with permissions to write "
                           "/sys/bus/usb-serial/devices/<dev>/latency_timer, or ship a udev rule that "
                           "sets it at plug time.");
      }
   }
#elif defined(__APPLE__)
   if(_config.latencyTimerMs > 0)
   {
      _logger->log(Logger::Level::Warn,
                   "FTDI latency timer left at the macOS default (~16 ms) on " + _config.port
                      + "; the exchange rate is capped near 60 Hz. macOS has no API to set it from here — "
                        "configure the FTDI driver instead (see the README's macOS serial-port notes).");
   }
#endif
}

bool DefaultSerial::isOpen() const
{
#ifdef __linux__
   return _portHandle != nullptr || _ptyFd >= 0;
#else
   return _portHandle != nullptr;
#endif
}

void DefaultSerial::close()
{
   if(_portHandle != nullptr)
   {
      sp_close(_portHandle);
      sp_free_port(_portHandle);
      _portHandle = nullptr;
   }
#ifdef __linux__
   if(_ptyFd >= 0)
   {
      ::close(_ptyFd);
      _ptyFd = -1;
   }
#endif
}

std::vector<uint8_t> DefaultSerial::read(size_t size, std::chrono::milliseconds timeout)
{
#ifdef __linux__
   if(_ptyFd >= 0)
   {
      std::vector<uint8_t> data(size);
      if(size == 0 || pollFd(_ptyFd, POLLIN, timeout, "waiting to read from PTY") == 0)
      {
         data.clear();
         return data;
      }
      ssize_t result;
      do
      {
         result = ::read(_ptyFd, data.data(), data.size());
      } while(result < 0 && errno == EINTR);
      if(result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
         data.clear();
         return data;
      }
      if(result < 0)
      {
         throw SerialIOException("reading from PTY: " + describeErrno());
      }
      data.resize(static_cast<size_t>(result));
      return data;
   }
#endif
   if(_portHandle == nullptr)
   {
      throw SerialIOException("read called on closed port");
   }

   std::vector<uint8_t> data(size);
   const sp_return result =
      timeout.count() == 0
         ? sp_nonblocking_read(_portHandle, data.data(), size)
         : sp_blocking_read(_portHandle, data.data(), size, static_cast<unsigned int>(timeout.count()));
   check(result, timeout.count() == 0 ? "sp_nonblocking_read" : "sp_blocking_read");
   data.resize(static_cast<size_t>(result));
   return data;
}

void DefaultSerial::write(const std::vector<uint8_t>& data)
{
#ifdef __linux__
   if(_ptyFd >= 0)
   {
      const auto deadline = std::chrono::steady_clock::now() + _config.timeout;
      size_t written = 0;
      while(written < data.size())
      {
         const auto now = std::chrono::steady_clock::now();
         if(now >= deadline
            || pollFd(_ptyFd,
                      POLLOUT,
                      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now),
                      "waiting to write to PTY")
                  == 0)
         {
            throw SerialIOException("write timeout: wrote " + std::to_string(written) + " of "
                                    + std::to_string(data.size()) + " bytes");
         }
         ssize_t result;
         do
         {
            result = ::write(_ptyFd, data.data() + written, data.size() - written);
         } while(result < 0 && errno == EINTR);
         if(result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
         {
            continue;
         }
         if(result < 0)
         {
            throw SerialIOException("writing to PTY: " + describeErrno());
         }
         written += static_cast<size_t>(result);
      }
      return;
   }
#endif
   if(_portHandle == nullptr)
   {
      throw SerialIOException("write called on closed port");
   }

   const sp_return result =
      sp_blocking_write(_portHandle, data.data(), data.size(), static_cast<unsigned int>(_config.timeout.count()));
   check(result, "sp_blocking_write");
   if(static_cast<size_t>(result) < data.size())
   {
      throw SerialIOException("write timeout: wrote " + std::to_string(result) + " of " + std::to_string(data.size())
                              + " bytes");
   }
   check(sp_drain(_portHandle), "sp_drain");
}

std::chrono::milliseconds DefaultSerial::getTimeout() const
{
   return _config.timeout;
}

const SerialConfig& DefaultSerial::getConfig() const
{
   return _config;
}

bool DefaultSerial::applyLatencyTimer() const
{
#ifdef __linux__
   const std::string base = deviceBasename(_config.port);
   if(base.empty())
   {
      return false;
   }
   const std::string path = "/sys/bus/usb-serial/devices/" + base + "/latency_timer";

   // Read-before-write: sysfs is root-owned, so an unprivileged caller can't
   // write to it. If a privileged init (udev rule, container entrypoint) has
   // already pinned the value, skip the write — otherwise we'd emit a
   // spurious WARN about a device that's already configured.
   {
      std::ifstream in(path);
      int current = -1;
      if(in >> current && current == _config.latencyTimerMs)
      {
         return true;
      }
   }

   std::ofstream out(path);
   if(!out.is_open())
   {
      return false;
   }
   out << _config.latencyTimerMs;
   return out.good();
#else
   // No sysfs on this platform; the FTDI latency timer is a driver setting
   // (e.g. Windows device manager). Nothing to enforce programmatically.
   return false;
#endif
}
} // namespace Robotiq::detail
