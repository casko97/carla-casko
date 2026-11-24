// Create an intentionally incompatible but structurally valid prop checkpoint.
// Used only by the portable import guard probe.
#include <carla/rpc/PortableRigidPropCheckpoint.h>
#include <rpc/msgpack.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  try {
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) return 2;
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)), {});
    size_t offset = 0;
    auto object = clmdep_msgpack::unpack(bytes.data(), bytes.size(), offset);
    if (offset != bytes.size()) return 2;
    auto state = object.get().as<carla::rpc::PortableRigidPropCheckpoint>();
    if (!state.IsValid()) return 2;
    state.mass *= 2.0f;
    if (!state.IsValid()) return 2;
    clmdep_msgpack::sbuffer packed;
    clmdep_msgpack::pack(packed, state);
    int fd = ::open(argv[2], O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return 2;
    size_t count = 0;
    while (count < packed.size()) {
      auto n = ::write(fd, packed.data() + count, packed.size() - count);
      if (n <= 0) { ::close(fd); return 2; }
      count += static_cast<size_t>(n);
    }
    return ::close(fd) == 0 ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
