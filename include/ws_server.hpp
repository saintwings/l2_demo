#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <set>
#include <string>

namespace fastlio2 {

// Minimal WS server that broadcasts binary frames to all connected clients.
class WsServer {
public:
  WsServer();
  ~WsServer();

  // Start/stop on given port (e.g., 8081). Returns true if listening started.
  bool start(uint16_t port);
  void stop();

  // Broadcast a binary blob to all connected clients (thread-safe).
  void broadcast(const void* data, std::size_t size);

  // Optional: simple logger callback
  void setLogFn(std::function<void(const std::string&)> fn);

private:
  struct Impl;
  Impl* impl_ = nullptr;
};

} // namespace fastlio2
