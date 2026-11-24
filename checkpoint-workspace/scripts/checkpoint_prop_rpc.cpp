// Minimal research client for experimental prop checkpoint RPCs.
#include <carla/rpc/Response.h>
#include <carla/rpc/Metadata.h>
#include <carla/rpc/PortableRigidPropCheckpoint.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc != 4 && argc != 6) {
    std::cerr << "usage: checkpoint_prop_rpc PORT save|restore ACTOR_ID | PORT export|import ACTOR_ID FILE BUILD_ID\n";
    return 2;
  }
  try {
    const unsigned long port = std::stoul(argv[1]);
    const std::string operation(argv[2]);
    const unsigned long id = std::stoul(argv[3]);
    if (port < 1 || port > 65535 || id == 0 || id > UINT32_MAX ||
        !((argc == 4 && (operation == "save" || operation == "restore")) ||
          (argc == 6 && (operation == "export" || operation == "import")))) return 2;
    rpc::client client("127.0.0.1", static_cast<uint16_t>(port));
    client.set_timeout(30000);
    if (operation == "export") {
      auto response = client.call("export_rigid_prop_checkpoint", carla::rpc::Metadata::MakeSync(),
          static_cast<uint32_t>(id), std::string(argv[5])).as<carla::rpc::Response<carla::rpc::PortableRigidPropCheckpoint>>();
      if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
      if (!response.Get().IsValid()) { std::cerr << "invalid exported checkpoint\n"; return 3; }
      clmdep_msgpack::sbuffer packed;
      clmdep_msgpack::pack(packed, response.Get());
      const int fd = ::open(argv[4], O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (fd < 0) { std::cerr << "cannot create checkpoint file\n"; return 3; }
      size_t written = 0;
      while (written < packed.size()) {
        const auto n = ::write(fd, packed.data() + written, packed.size() - written);
        if (n <= 0) { ::close(fd); std::cerr << "checkpoint write failed\n"; return 3; }
        written += static_cast<size_t>(n);
      }
      if (::close(fd) != 0) { std::cerr << "checkpoint close failed\n"; return 3; }
      std::cout << "exported\n";
      return 0;
    }
    if (operation == "import") {
      std::ifstream input(argv[4], std::ios::binary | std::ios::ate);
      if (!input || input.tellg() <= 0 || input.tellg() > 1024 * 1024) { std::cerr << "invalid file size\n"; return 3; }
      const auto size = static_cast<size_t>(input.tellg());
      std::vector<char> bytes(size);
      input.seekg(0);
      if (!input.read(bytes.data(), size)) { std::cerr << "checkpoint read failed\n"; return 3; }
      size_t offset = 0;
      auto object = clmdep_msgpack::unpack(bytes.data(), bytes.size(), offset);
      if (offset != bytes.size()) { std::cerr << "trailing checkpoint data\n"; return 3; }
      auto data = object.get().as<carla::rpc::PortableRigidPropCheckpoint>();
      if (!data.IsValid()) { std::cerr << "invalid checkpoint\n"; return 3; }
      auto response = client.call("import_rigid_prop_checkpoint", carla::rpc::Metadata::MakeSync(),
          static_cast<uint32_t>(id), data, std::string(argv[5])).as<carla::rpc::Response<bool>>();
      if (response.HasError()) { std::cerr << response.GetError().What() << '\n'; return 3; }
      std::cout << (response.Get() ? "accepted" : "rejected") << '\n';
      return response.Get() ? 0 : 1;
    }
    const auto method = operation == "save" ? "save_rigid_prop_checkpoint" : "restore_rigid_prop_checkpoint";
    auto response = client.call(method, carla::rpc::Metadata::MakeSync(), static_cast<uint32_t>(id)).as<carla::rpc::Response<bool>>();
    if (response.HasError()) {
      std::cerr << response.GetError().What() << '\n';
      return 3;
    }
    std::cout << (response.Get() ? "accepted" : "rejected") << '\n';
    return response.Get() ? 0 : 1;
  } catch (rpc::rpc_error &error) {
    std::cerr << error.what() << ": " << error.get_error().get() << '\n';
    return 3;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 3;
  }
}
