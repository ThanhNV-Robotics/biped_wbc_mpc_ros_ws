#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string>

namespace udp_interface {

// Minimal non-blocking UDP socket wrapper.
// send_socket: used to send state packets to the controller.
// recv_socket: bound to a local port, used to receive command packets.
class UdpSocket
{
public:
  UdpSocket() = default;
  ~UdpSocket();

  // Open a socket for sending datagrams to remote_ip:remote_port.
  bool open_sender(const std::string& remote_ip, uint16_t remote_port);

  // Open a socket bound to local_port for receiving datagrams.
  // recv_timeout_ms: SO_RCVTIMEO (0 = blocking).
  bool open_receiver(uint16_t local_port, int recv_timeout_ms = 100);

  // Send raw bytes. Returns true on success.
  bool send(const void* data, size_t len);

  // Receive raw bytes (blocks up to recv_timeout_ms set at open_receiver).
  // Returns number of bytes received, or -1 on error/timeout.
  ssize_t recv(void* buf, size_t max_len);

  bool is_open() const { return fd_ >= 0; }
  void close();

private:
  int fd_{-1};
  sockaddr_in remote_addr_{};
};

}  // namespace udp_interface
