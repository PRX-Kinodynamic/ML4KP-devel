#include "prx/utilities/communication/zmq_communication.hpp"

#ifdef ZMQ_BUILT

namespace prx 
{

zmq_communication_t::zmq_communication_t() 
{
}

zmq_communication_t::~zmq_communication_t() 
{
    stop_async_receive();
    socket_.reset();
}

bool zmq_communication_t::initialize_socket(socket_type type, const std::string& endpoint, bool is_server) 
{
    try {
        socket_type_ = type;
        endpoint_ = endpoint;
        is_server_ = is_server;
        
        socket_ = std::make_unique<zmq::socket_t>(context_, convert_socket_type(type));
        
        if (is_server) {
            socket_->bind(endpoint);
            // PRX_DEBUG_COLOR("ZMQ server bound to " + endpoint, PRX_TEXT_GREEN);
        } else {
            socket_->connect(endpoint);
            // PRX_DEBUG_COLOR("ZMQ client connected to " + endpoint, PRX_TEXT_GREEN);
        }
        
        // Subscribe to all messages if this is a SUB socket
        if (type == socket_type::SUB) {
            socket_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
        }
        
        return true;
    } catch (const zmq::error_t& e) {
        // PRX_ERROR_S("ZMQ error: " << e.what());
        return false;
    }
}

bool zmq_communication_t::send_message(const std::string& message, bool nonblocking) 
{
    if (!socket_) return false;
    
    try {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        zmq::message_t zmq_message(message.size());
        memcpy(zmq_message.data(), message.data(), message.size());
        
        int flags = nonblocking ? ZMQ_DONTWAIT : 0;
        return socket_->send(zmq_message, flags);
    } catch (const zmq::error_t& e) {
        // PRX_ERROR_S("ZMQ send error: " << e.what());
        return false;
    }
}

bool zmq_communication_t::send_data(const void* data, size_t size, bool nonblocking) 
{
    if (!socket_) return false;
    
    try {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        zmq::message_t zmq_message(size);
        memcpy(zmq_message.data(), data, size);
        
        int flags = nonblocking ? ZMQ_DONTWAIT : 0;
        return socket_->send(zmq_message, flags);
    } catch (const zmq::error_t& e) {
        // PRX_ERROR_S("ZMQ send error: " << e.what());
        return false;
    }
}

std::string zmq_communication_t::receive_message(bool nonblocking) 
{
    if (!socket_) return "";
    
    try {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        zmq::message_t message;
        int flags = nonblocking ? ZMQ_DONTWAIT : 0;
        
        bool received = socket_->recv(&message, flags);
        if (!received) {
            return "";
        }
        
        return std::string(static_cast<char*>(message.data()), message.size());
    } catch (const zmq::error_t& e) {
        // PRX_ERROR_S("ZMQ receive error: " << e.what());
        return "";
    }
}

std::vector<uint8_t> zmq_communication_t::receive_data(bool nonblocking) 
{
    if (!socket_) return {};
    
    try {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        zmq::message_t message;
        int flags = nonblocking ? ZMQ_DONTWAIT : 0;
        
        bool received = socket_->recv(&message, flags);
        if (!received) {
            return {};
        }
        
        std::vector<uint8_t> data(static_cast<uint8_t*>(message.data()), 
                                  static_cast<uint8_t*>(message.data()) + message.size());
        return data;
    } catch (const zmq::error_t& e) {
        // PRX_ERROR_S("ZMQ receive error: " << e.what());
        return {};
    }
}

void zmq_communication_t::set_message_callback(std::function<void(const std::string&)> callback) 
{
    message_callback_ = callback;
}

void zmq_communication_t::start_async_receive() 
{
    if (!socket_ || running_ || !message_callback_) {
        return;
    }
    
    running_ = true;
    receive_thread_ = std::thread(&zmq_communication_t::receive_thread_func, this);
}

void zmq_communication_t::stop_async_receive() 
{
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

bool zmq_communication_t::is_connected() const 
{
    return socket_ != nullptr;
}

void zmq_communication_t::receive_thread_func() 
{
    while (running_) {
        std::string message = receive_message(true);
        if (!message.empty() && message_callback_) {
            message_callback_(message);
        }
        
        // Small delay to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int zmq_communication_t::convert_socket_type(socket_type type) const 
{
    switch (type) {
        case socket_type::PUB: return ZMQ_PUB;
        case socket_type::SUB: return ZMQ_SUB;
        case socket_type::REQ: return ZMQ_REQ;
        case socket_type::REP: return ZMQ_REP;
        case socket_type::PUSH: return ZMQ_PUSH;
        case socket_type::PULL: return ZMQ_PULL;
        default: return ZMQ_PAIR;
    }
}

} // namespace prx

#endif // ZMQ_BUILT