// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sentry_behavior/viz/state_viz_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <utility>

namespace sentry_behavior
{
namespace
{
constexpr size_t kMaxTransitions = 64;

void set_nonblock(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
}

// 非阻塞发一行 NDJSON; 慢客户端 (EAGAIN) 丢弃本帧而非阻塞; 断开/错误返回 false.
// MSG_NOSIGNAL: 对端关闭时不触发 SIGPIPE 杀进程.
bool send_line(int fd, const std::string & line)
{
  std::string buf = line;
  buf.push_back('\n');
  size_t off = 0;
  while (off < buf.size()) {
    ssize_t n = ::send(fd, buf.data() + off, buf.size() - off, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n > 0) {
      off += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    return false;
  }
  return true;
}
}  // namespace

StateVizServer::StateVizServer(uint16_t port, std::string graph_json)
: graph_json_(std::move(graph_json))
{
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    return;
  }
  int yes = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
    ::listen(listen_fd_, 4) < 0)
  {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return;
  }
  set_nonblock(listen_fd_);
  running_ = true;
  thread_ = std::thread(&StateVizServer::run, this);
}

StateVizServer::~StateVizServer()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  for (int fd : clients_) {
    ::close(fd);
  }
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
  }
}

void StateVizServer::update_state(const std::string & state_json)
{
  std::lock_guard<std::mutex> lk(mtx_);
  latest_state_ = state_json;
}

void StateVizServer::push_transition(const std::string & transition_json)
{
  std::lock_guard<std::mutex> lk(mtx_);
  transitions_.push_back(transition_json);
  while (transitions_.size() > kMaxTransitions) {
    transitions_.pop_front();
  }
}

void StateVizServer::accept_clients()
{
  for (;;) {
    int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) {
      break;
    }
    set_nonblock(fd);
    std::string state_copy;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      state_copy = latest_state_;
    }
    bool ok = send_line(fd, graph_json_);
    if (ok && !state_copy.empty()) {
      ok = send_line(fd, state_copy);
    }
    if (ok) {
      clients_.push_back(fd);
    } else {
      ::close(fd);
    }
  }
}

void StateVizServer::broadcast(const std::string & line)
{
  std::vector<int> alive;
  alive.reserve(clients_.size());
  for (int fd : clients_) {
    if (send_line(fd, line)) {
      alive.push_back(fd);
    } else {
      ::close(fd);
    }
  }
  clients_.swap(alive);
}

void StateVizServer::run()
{
  while (running_) {
    if (listen_fd_ >= 0) {
      struct pollfd pfd;
      pfd.fd = listen_fd_;
      pfd.events = POLLIN;
      ::poll(&pfd, 1, 100);
      accept_clients();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::string state_copy;
    std::deque<std::string> trans;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      state_copy = latest_state_;
      trans.swap(transitions_);
    }
    if (clients_.empty()) {
      continue;
    }
    for (const auto & t : trans) {
      broadcast(t);
    }
    if (!state_copy.empty()) {
      broadcast(state_copy);
    }
  }
}

}  // namespace sentry_behavior
