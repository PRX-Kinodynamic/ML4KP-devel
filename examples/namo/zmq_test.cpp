// #include "prx/utilities/defs.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/communication/zmq_communication.hpp"

// #ifdef ZMQ_BUILT
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace prx;

int main(int argc, char* argv[])
{
    // prx::param_loader params("examples/namo/zmq_test.yaml", argc, argv);
    std::string endpoint = "tcp://arrakis.cs.rutgers.edu:5555"; // params["endpoint"].as<std::string>();
    
    // Create ZMQ communication client
    zmq_communication_t client;
    
    // Initialize as REQ socket (client)
    if (!client.initialize_socket(zmq_communication_t::socket_type::REQ, endpoint, false)) {
        std::cerr << "Failed to initialize client socket" << std::endl;
        return 1;
    }
    
    std::cout << "Connected to Python server at " << endpoint << std::endl;
    
    // Send requests and receive responses
    for (int i = 1; i <= 5; ++i) {
        // Create message
        std::string message = "Request " + std::to_string(i) + " from C++ client";
        
        // Send request
        std::cout << "Sending: " << message << std::endl;
        if (client.send_message(message)) {
            // Wait for response
            std::string response = client.receive_message();
            std::cout << "Received: " << response << std::endl;
        } else {
            std::cerr << "Failed to send message" << std::endl;
        }
        
        // Wait before next request
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
// #else
// int main() {
//     std::cerr << "ZMQ support not built. Please install ZeroMQ and rebuild with ZMQ support." << std::endl;
//     return 1;
// }
// #endif // ZMQ_BUILT