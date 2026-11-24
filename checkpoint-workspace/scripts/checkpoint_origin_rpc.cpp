// Read-only research client for the server's checkpoint world-origin query.
#include <carla/rpc/Metadata.h>
#include <carla/rpc/Response.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: checkpoint_origin_rpc PORT\n"; return 2; }
  try {
    const auto port = std::stoul(argv[1]);
    if (port < 1 || port > 65535) return 2;
    rpc::client client("127.0.0.1", static_cast<uint16_t>(port));
    client.set_timeout(180000);
    auto response = client.call("get_checkpoint_world_origin", carla::rpc::Metadata::MakeSync())
      .as<carla::rpc::Response<std::vector<int32_t>>>();
    if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
    const auto &origin = response.Get();
    if (origin.size() != 3) return 3;
    std::cout << '[' << origin[0] << ',' << origin[1] << ',' << origin[2] << "]\n";
    return 0;
  } catch (rpc::rpc_error &e) {
    std::cerr << e.what() << ": " << e.get_error().get() << '\n'; return 3;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 3; }
}
