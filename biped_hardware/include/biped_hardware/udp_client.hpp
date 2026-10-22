#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "biped_hardware/wbc_protocol.h"

namespace biped_hardware
{

/// UDP link to the NUCLEO-H755 leg bridge (WBC port 9999, see wbc_protocol.h).
///
/// The socket is connected to the board, so the kernel only delivers the board's datagrams,
/// and non-blocking, so read()/write() of the control loop never wait on the network.
class UdpClient
{
public:
  UdpClient() = default;
  ~UdpClient();

  UdpClient(const UdpClient &) = delete;
  UdpClient & operator=(const UdpClient &) = delete;

  /// Open and connect the socket. On failure, error() tells why.
  bool open(const std::string & board_ip, uint16_t port);
  void close();
  bool is_open() const {return sock_ >= 0;}

  bool send(const WbcCommandPacket & cmd);

  /// Drain the socket and keep the newest valid state packet.
  /// Returns false if no valid packet was waiting. Wrong size / magic / version are dropped
  /// and counted in dropped().
  bool recv_latest(WbcStatePacket & out);

  /// Block up to timeout_ms for the next datagram (used only during configure, not in the loop).
  bool wait_readable(int timeout_ms);

  /// Local address the kernel picked for the connected socket (shows which interface is used)
  std::string local_address() const;

  const std::string & error() const {return error_;}
  uint64_t dropped() const {return dropped_;}

private:
  int sock_{-1};
  std::string error_;
  uint64_t dropped_{0};
};

}  // namespace biped_hardware
