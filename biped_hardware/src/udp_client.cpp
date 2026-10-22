#include "biped_hardware/udp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace biped_hardware
{

UdpClient::~UdpClient()
{
  close();
}

bool UdpClient::open(const std::string & board_ip, uint16_t port)
{
  close();
  sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0) {
    error_ = std::string("socket(): ") + std::strerror(errno);
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, board_ip.c_str(), &addr.sin_addr) != 1) {
    error_ = "invalid board address '" + board_ip + "'";
    close();
    return false;
  }

  // A connected UDP socket only receives datagrams from the board
  if (::connect(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    error_ = "connect(" + board_ip + "): " + std::strerror(errno);
    close();
    return false;
  }

  const int flags = fcntl(sock_, F_GETFL, 0);
  if (flags < 0 || fcntl(sock_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error_ = std::string("fcntl(O_NONBLOCK): ") + std::strerror(errno);
    close();
    return false;
  }

  error_.clear();
  dropped_ = 0;
  return true;
}

void UdpClient::close()
{
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
}

bool UdpClient::send(const WbcCommandPacket & cmd)
{
  if (sock_ < 0) {
    error_ = "socket not open";
    return false;
  }
  const ssize_t n = ::send(sock_, &cmd, sizeof(cmd), 0);
  if (n != static_cast<ssize_t>(sizeof(cmd))) {
    // ECONNREFUSED: an earlier datagram got ICMP port unreachable (nothing listens on the board)
    error_ = std::string("send(): ") + (n < 0 ? std::strerror(errno) : "short write");
    return false;
  }
  return true;
}

bool UdpClient::recv_latest(WbcStatePacket & out)
{
  if (sock_ < 0) {
    error_ = "socket not open";
    return false;
  }

  bool got = false;
  // One byte larger than a state packet, so an oversized datagram is detected, not truncated
  uint8_t buf[sizeof(WbcStatePacket) + 1];
  while (true) {
    const ssize_t n = ::recv(sock_, buf, sizeof(buf), 0);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        error_ = std::string("recv(): ") + std::strerror(errno);
      }
      break;
    }
    if (n != static_cast<ssize_t>(sizeof(WbcStatePacket))) {
      ++dropped_;
      continue;
    }
    WbcStatePacket st;
    std::memcpy(&st, buf, sizeof(st));
    if (st.magic != WBC_STATE_MAGIC || st.version != WBC_VERSION) {
      ++dropped_;
      continue;
    }
    out = st;
    got = true;
  }
  return got;
}

std::string UdpClient::local_address() const
{
  if (sock_ < 0) {
    return "";
  }
  sockaddr_in addr{};
  socklen_t len = sizeof(addr);
  if (::getsockname(sock_, reinterpret_cast<sockaddr *>(&addr), &len) < 0) {
    return "";
  }
  char buf[INET_ADDRSTRLEN] = {};
  ::inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf));
  return buf;
}

bool UdpClient::wait_readable(int timeout_ms)
{
  if (sock_ < 0) {
    return false;
  }
  pollfd pfd{sock_, POLLIN, 0};
  int ready;
  do {
    ready = ::poll(&pfd, 1, timeout_ms);
  } while (ready < 0 && errno == EINTR);
  return ready > 0;
}

}  // namespace biped_hardware
