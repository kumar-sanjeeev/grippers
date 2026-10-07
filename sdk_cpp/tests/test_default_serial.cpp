// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>

#ifdef __linux__
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#endif

#include <Robotiq/detail/default_serial.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>

namespace Robotiq::test {
using detail::DefaultSerial;

namespace {
DefaultSerial makeSerial(SerialConfig config = {})
{
   return DefaultSerial(std::move(config), std::make_shared<NullLogger>());
}

#ifdef __linux__
class LinuxPty
{
public:
   LinuxPty()
   {
      _master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
      if(_master < 0 || grantpt(_master) != 0 || unlockpt(_master) != 0)
      {
         throw std::runtime_error("could not create test PTY");
      }
      char* name = ptsname(_master);
      if(name == nullptr)
      {
         throw std::runtime_error("could not get test PTY name");
      }
      slave = name;
   }

   ~LinuxPty()
   {
      if(_master >= 0)
      {
         ::close(_master);
      }
   }

   LinuxPty(const LinuxPty&) = delete;
   LinuxPty& operator=(const LinuxPty&) = delete;

   int master() const { return _master; }

   std::string slave;

private:
   int _master = -1;
};
#endif
} // namespace

TEST(TestDefaultSerial, throws_when_port_does_not_exist)
{
   SerialConfig config;
   config.port = "/dev/this_should_not_exist";
   auto serial = makeSerial(config);
   EXPECT_THROW(serial.open(), SerialIOException);
   EXPECT_FALSE(serial.isOpen());
}

TEST(TestDefaultSerial, throws_when_port_is_empty)
{
   auto serial = makeSerial();
   EXPECT_THROW(serial.open(), SerialIOException);
}

TEST(TestDefaultSerial, read_and_write_throw_on_closed_port)
{
   auto serial = makeSerial();
   EXPECT_THROW((void)serial.read(1, std::chrono::milliseconds{10}), SerialIOException);
   EXPECT_THROW(serial.write({0x01}), SerialIOException);
}

TEST(TestDefaultSerial, close_is_idempotent_on_a_never_opened_port)
{
   auto serial = makeSerial();
   EXPECT_NO_THROW(serial.close());
   EXPECT_NO_THROW(serial.close());
   EXPECT_FALSE(serial.isOpen());
}

TEST(TestDefaultSerial, construction_config_round_trip)
{
   SerialConfig config;
   config.port = "/dev/ttyUSB0";
   config.baudrate = 230400;
   config.timeout = std::chrono::milliseconds{200};
   config.latencyTimerMs = 1;
   auto serial = makeSerial(config);

   EXPECT_EQ(serial.getConfig().port, "/dev/ttyUSB0");
   EXPECT_EQ(serial.getConfig().baudrate, 230400U);
   EXPECT_EQ(serial.getConfig().timeout, std::chrono::milliseconds{200});
   EXPECT_EQ(serial.getConfig().latencyTimerMs, 1);
   EXPECT_EQ(serial.getTimeout(), std::chrono::milliseconds{200});
}

#ifdef __linux__
TEST(TestDefaultSerial, opens_linux_pty_through_symlink)
{
   LinuxPty pty;
   const auto link = std::filesystem::temp_directory_path() / ("robotiq-tty-" + std::to_string(getpid()));
   std::filesystem::create_symlink(pty.slave, link);

   SerialConfig config;
   config.port = link.string();
   auto serial = makeSerial(config);
   EXPECT_NO_THROW(serial.open());
   EXPECT_TRUE(serial.isOpen());

   serial.close();
   std::filesystem::remove(link);
}

TEST(TestDefaultSerial, exchanges_bytes_over_linux_pty)
{
   LinuxPty pty;
   SerialConfig config;
   config.port = pty.slave;
   auto serial = makeSerial(config);
   serial.open();

   const std::vector<uint8_t> outbound{0x09, 0x03, 0x07};
   serial.write(outbound);
   std::vector<uint8_t> observed(outbound.size());
   ASSERT_EQ(::read(pty.master(), observed.data(), observed.size()), static_cast<ssize_t>(observed.size()));
   EXPECT_EQ(observed, outbound);

   const std::vector<uint8_t> inbound{0x09, 0x17, 0x00, 0x03};
   ASSERT_EQ(::write(pty.master(), inbound.data(), inbound.size()), static_cast<ssize_t>(inbound.size()));
   EXPECT_EQ(serial.read(inbound.size(), std::chrono::milliseconds{100}), inbound);
}

TEST(TestDefaultSerial, linux_pty_read_honors_timeout)
{
   LinuxPty pty;
   SerialConfig config;
   config.port = pty.slave;
   auto serial = makeSerial(config);
   serial.open();

   const auto start = std::chrono::steady_clock::now();
   EXPECT_TRUE(serial.read(1, std::chrono::milliseconds{20}).empty());
   const auto elapsed = std::chrono::steady_clock::now() - start;
   EXPECT_GE(elapsed, std::chrono::milliseconds{15});
}
#endif

TEST(TestDeviceBasename, strips_the_directory_part)
{
   EXPECT_EQ(detail::deviceBasename("/dev/ttyUSB0"), "ttyUSB0");
   EXPECT_EQ(detail::deviceBasename("/dev/serial/by-id/usb-FTDI_X-if00-port0"), "usb-FTDI_X-if00-port0");
   EXPECT_EQ(detail::deviceBasename("COM3"), "COM3");
   EXPECT_EQ(detail::deviceBasename(""), "");
}

} // namespace Robotiq::test
