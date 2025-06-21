#pragma once

#include "prx/utilities/defs.hpp"

#ifdef ZMQ_BUILT
#include <zmq.hpp>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

namespace prx 
{

class zmq_communication_t 
{
public:
    enum class socket_type 
    {
        PUB,    // Publisher (one-to-many)
        SUB,    // Subscriber (receives from publisher)
        REQ,    // Request (sends requests, receives replies)
        REP,    // Reply (receives requests, sends replies)
        PUSH,   // Push (distributes messages, load balanced)
        PULL    // Pull (receives messages from pushers)
    };

    // Constructor
    zmq_communication_t();
    
    // Destructor - ensures clean shutdown
    ~zmq_communication_t();

    // Initialize a socket with specified type
    bool initialize_socket(socket_type type, const std::string& endpoint, bool is_server = true);
    
    // Send a message
    bool send_message(const std::string& message, bool nonblocking = false);
    
    // Send binary data
    bool send_data(const void* data, size_t size, bool nonblocking = false);
    
    // Receive a message (blocking or non-blocking)
    std::string receive_message(bool nonblocking = false);
    
    // Receive binary data (blocking or non-blocking)
    std::vector<uint8_t> receive_data(bool nonblocking = false);
    
    // Set a callback for message reception (for asynchronous reception)
    void set_message_callback(std::function<void(const std::string&)> callback);
    
    // Start asynchronous reception
    void start_async_receive();
    
    // Stop asynchronous reception
    void stop_async_receive();
    
    // Check if connected
    bool is_connected() const;

private:
    zmq::context_t context_{1};
    std::unique_ptr<zmq::socket_t> socket_;
    socket_type socket_type_;
    bool is_server_;
    std::string endpoint_;
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::mutex socket_mutex_;
    std::function<void(const std::string&)> message_callback_;
    
    // Thread function for asynchronous message reception
    void receive_thread_func();
    
    // Convert prx socket type to ZMQ socket type
    int convert_socket_type(socket_type type) const;
};

} // namespace prx

#endif // ZMQ_BUILT