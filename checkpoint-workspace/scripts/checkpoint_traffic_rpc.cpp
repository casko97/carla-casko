// Research client for experimental portable traffic-light checkpoint RPCs.
#include <carla/rpc/Response.h>
#include <carla/rpc/Metadata.h>
#include <carla/rpc/PortableTrafficLightsCheckpoint.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc != 5) { std::cerr << "usage: checkpoint_traffic_rpc PORT export|import FILE BUILD_ID\n"; return 2; }
  try {
    const auto port = std::stoul(argv[1]);
    const std::string operation(argv[2]);
    if (port < 1 || port > 65535 || (operation != "export" && operation != "import")) return 2;
    rpc::client client("127.0.0.1", static_cast<uint16_t>(port));
    client.set_timeout(30000);
    if (operation == "export") {
      auto response = client.call("export_traffic_lights_checkpoint", carla::rpc::Metadata::MakeSync(),
        std::string(argv[4])).as<carla::rpc::Response<carla::rpc::PortableTrafficLightsCheckpoint>>();
      if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
      if (!response.Get().IsValid()) { std::cerr << "invalid exported checkpoint\n"; return 3; }
      clmdep_msgpack::sbuffer packed;
      clmdep_msgpack::pack(packed, response.Get());
      const int fd = ::open(argv[3], O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (fd < 0) { std::cerr << "cannot create checkpoint\n"; return 3; }
      size_t written = 0;
      while (written < packed.size()) {
        const auto n = ::write(fd, packed.data() + written, packed.size() - written);
        if (n <= 0) { ::close(fd); std::cerr << "write failed\n"; return 3; }
        written += static_cast<size_t>(n);
      }
      if (::close(fd) != 0) return 3;
      return 0;
    }
    std::ifstream in(argv[3], std::ios::binary | std::ios::ate);
    if (!in || in.tellg() <= 0 || in.tellg() > 4 * 1024 * 1024) { std::cerr << "invalid checkpoint size\n"; return 3; }
    auto size = static_cast<size_t>(in.tellg());
    std::vector<char> bytes(size);
    in.seekg(0);
    if (!in.read(bytes.data(), size)) return 3;
    size_t offset = 0;
    auto packed = clmdep_msgpack::unpack(bytes.data(), bytes.size(), offset);
    if (offset != bytes.size()) { std::cerr << "trailing data\n"; return 3; }
    auto data = packed.get().as<carla::rpc::PortableTrafficLightsCheckpoint>();
    if (!data.IsValid()) { std::cerr << "invalid checkpoint\n"; return 3; }
    auto response = client.call("import_traffic_lights_checkpoint", carla::rpc::Metadata::MakeSync(),
      data, std::string(argv[4])).as<carla::rpc::Response<bool>>();
    if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
    std::cout << (response.Get() ? "accepted" : "rejected") << '\n';
    return response.Get() ? 0 : 1;
  } catch (rpc::rpc_error &e) {
    std::cerr << e.what() << ": " << e.get_error().get() << '\n'; return 3;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 3; }
}
