#include "udp_interface/UdpSocket.hpp"

#include <unistd.h>
#include <cstring>
#include <stdexcept>

namespace udp_interface {

UdpSocket::~UdpSocket()
{
  close();
}

bool UdpSocket::open_sender(const std::string& remote_ip, uint16_t remote_port)
{
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;

  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port   = htons(remote_port);
  if (::inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr) != 1) {
    close();
    return false;
  }
  return true;
}

bool UdpSocket::open_receiver(uint16_t local_port, int recv_timeout_ms)
{
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;

  // Allow address reuse so the node can restart quickly
  int opt = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (recv_timeout_ms > 0) {
    timeval tv;
    tv.tv_sec  = recv_timeout_ms / 1000;
    tv.tv_usec = (recv_timeout_ms % 1000) * 1000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(local_port);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }
  return true;
}

bool UdpSocket::send(const void* data, size_t len)
{
  return ::sendto(fd_, data, len, 0,
                  reinterpret_cast<const sockaddr*>(&remote_addr_),
                  sizeof(remote_addr_)) == static_cast<ssize_t>(len);
}

ssize_t UdpSocket::recv(void* buf, size_t max_len)
{
  return ::recvfrom(fd_, buf, max_len, 0, nullptr, nullptr);
}

void UdpSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace udp_interface
