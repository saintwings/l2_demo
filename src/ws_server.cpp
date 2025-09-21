#include "fastlio2/ws_server.hpp"

// Boost.Asio-backed websocketpp (no TLS)
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <boost/asio.hpp>   // ensure Boost.Asio is available
#include <iostream>

namespace fastlio2 {

struct WsServer::Impl {
  using Server = websocketpp::server<websocketpp::config::asio>;

  Server server;
  std::thread th;
  std::mutex mtx;
  std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> conns;
  std::function<void(const std::string&)> log;
  std::atomic<bool> running{false};

  void logMsg(const std::string& s){ if (log) log(s); }

  void run(uint16_t port){
    try{
      server.clear_access_channels(websocketpp::log::alevel::all); // quiet by default
      server.init_asio();                      // uses Boost.Asio by default in this config
      server.set_reuse_addr(true);

      server.set_open_handler([this](websocketpp::connection_hdl hdl){
        std::lock_guard<std::mutex> lk(mtx);
        conns.insert(hdl);
        logMsg("WS client connected");
      });

      server.set_close_handler([this](websocketpp::connection_hdl hdl){
        std::lock_guard<std::mutex> lk(mtx);
        conns.erase(hdl);
        logMsg("WS client disconnected");
      });

      server.set_message_handler([](websocketpp::connection_hdl, Server::message_ptr){ /*ignore*/ });

      server.listen(port);
      server.start_accept();
      running = true;
      server.run();  // blocking
    } catch (const std::exception& e) {
      logMsg(std::string("WS run exception: ") + e.what());
    }
    running = false;
  }
};

WsServer::WsServer() : impl_(new Impl()) {}
WsServer::~WsServer(){ stop(); delete impl_; }

bool WsServer::start(uint16_t port){
  if (impl_->running) return true;
  impl_->th = std::thread([this, port](){ impl_->run(port); });
  // Let ASIO bind before returning (simple delay)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

void WsServer::stop(){
  if (!impl_) return;
  // First, signal stop without holding the mutex while stopping/joining
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    impl_->running = false;
  }

  // Stop accepting and stop the ASIO loop
  try { impl_->server.stop_listening(); } catch (...) {}
  try { impl_->server.stop(); } catch (...) {}

  // Join the thread if running
  if (impl_->th.joinable()) {
    try { impl_->th.join(); } catch (...) {}
  }

  // Now it is safe to clear any remaining connection handles
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    impl_->conns.clear();
  }
}

void WsServer::broadcast(const void* data, std::size_t size){
  if (!impl_->running) return;
  std::lock_guard<std::mutex> lk(impl_->mtx);
  if (!impl_->running || impl_->conns.empty()) return;
  for (auto h : impl_->conns){
    try {
      impl_->server.send(h, data, size, websocketpp::frame::opcode::binary);
    } catch (...) {
      // ignore per-connection send errors
    }
  }
}

void WsServer::setLogFn(std::function<void(const std::string&)> fn){
  impl_->log = std::move(fn);
}

} // namespace fastlio2
