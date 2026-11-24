// Narrow diagnostic client: capture and restore walker movement velocity and observer cache.
#include <carla/rpc/Response.h>
#include <carla/rpc/Metadata.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc != 5) { std::cerr << "usage: walker_velocity_rpc PORT capture|restore ACTOR_ID FILE\n"; return 2; }
  try {
    const auto port = std::stoul(argv[1]);
    const std::string operation(argv[2]);
    const auto id = std::stoul(argv[3]);
    if (port < 1 || port > 65535 || id == 0 || id > UINT32_MAX ||
        (operation != "capture" && operation != "restore")) return 2;
    rpc::client client("127.0.0.1", static_cast<uint16_t>(port));
    client.set_timeout(30000);
    if (operation == "capture") {
      auto response = client.call("capture_walker_velocity_probe", carla::rpc::Metadata::MakeSync(),
        static_cast<uint32_t>(id)).as<carla::rpc::Response<std::vector<float>>>();
      if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
      if (response.Get().size() != 6) return 3;
      clmdep_msgpack::sbuffer packed;
      clmdep_msgpack::pack(packed, response.Get());
      int fd = ::open(argv[4], O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (fd < 0) return 3;
      size_t n = 0;
      while (n < packed.size()) {
        auto count = ::write(fd, packed.data() + n, packed.size() - n);
        if (count <= 0) { ::close(fd); return 3; }
        n += static_cast<size_t>(count);
      }
      return ::close(fd) == 0 ? 0 : 3;
    }
    std::ifstream in(argv[4], std::ios::binary | std::ios::ate);
    if (!in || in.tellg() <= 0 || in.tellg() > 1024) return 3;
    std::vector<char> bytes(static_cast<size_t>(in.tellg()));
    in.seekg(0);
    if (!in.read(bytes.data(), bytes.size())) return 3;
    size_t offset = 0;
    auto packed = clmdep_msgpack::unpack(bytes.data(), bytes.size(), offset);
    if (offset != bytes.size()) return 3;
    auto values = packed.get().as<std::vector<float>>();
    if (values.size() != 6) return 3;
    auto response = client.call("restore_walker_velocity_probe", carla::rpc::Metadata::MakeSync(),
      static_cast<uint32_t>(id), values).as<carla::rpc::Response<bool>>();
    if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
    return response.Get() ? 0 : 1;
  } catch (rpc::rpc_error &e) {
    std::cerr << e.what() << ": " << e.get_error().get() << '\n'; return 3;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 3; }
}
