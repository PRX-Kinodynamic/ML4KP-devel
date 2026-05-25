#pragma once
// #include <foxglove/foxglove.hpp>
// #include <foxglove/context.hpp>
// #include <foxglove/error.hpp>
// #include <foxglove/mcap.hpp>
// #include <foxglove/server.hpp>

namespace prx
{
namespace visualization
{

// class visualizer_t
// {
//   visualizer_t()
//   {
//     foxglove::setLogLevel(foxglove::LogLevel::Debug);
//   }

//   foxglove::McapWriterOptions mcap_options = {};
//   mcap_options.path = "quickstart-cpp.mcap";
//   auto writer_result = foxglove::McapWriter::create(mcap_options);
//   if (!writer_result.has_value())
//   {
//     std::cerr << "Failed to create writer: " << foxglove::strerror(writer_result.error()) << '\n';
//     return 1;
//   }
//   auto writer = std::move(writer_result.value());

//   // Start a server to communicate with the Foxglove app.
//   foxglove::WebSocketServerOptions ws_options;
//   ws_options.host = "127.0.0.1";
//   ws_options.port = 8765;
//   auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
//   if (!server_result.has_value())
//   {
//     std::cerr << "Failed to create server: " << foxglove::strerror(server_result.error()) << '\n';
//     return 1;
//   }
//   auto server = std::move(server_result.value());
//   std::cerr << "Server listening on port " << server.port() << '\n';

//   // return 0;
// };

}  // namespace visualization
}  // namespace prx