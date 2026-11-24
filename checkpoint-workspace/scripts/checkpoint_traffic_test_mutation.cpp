// Produce a valid but topology-incompatible checkpoint for a rejection test.
#include <carla/rpc/PortableTrafficLightsCheckpoint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream in(argv[1], std::ios::binary | std::ios::ate);
  if (!in || in.tellg() <= 0 || in.tellg() > 4 * 1024 * 1024) return 2;
  std::vector<char> bytes(static_cast<size_t>(in.tellg()));
  in.seekg(0);
  if (!in.read(bytes.data(), bytes.size())) return 2;
  size_t offset = 0;
  auto obj = clmdep_msgpack::unpack(bytes.data(), bytes.size(), offset);
  auto data = obj.get().as<carla::rpc::PortableTrafficLightsCheckpoint>();
  if (offset != bytes.size() || !data.IsValid()) return 2;
  data.groups.back().controllers.back().id += "-mismatch";
  if (!data.IsValid()) return 2;
  clmdep_msgpack::sbuffer out;
  clmdep_msgpack::pack(out, data);
  int fd = ::open(argv[2], O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0) return 2;
  size_t n = 0;
  while (n < out.size()) {
    auto count = ::write(fd, out.data() + n, out.size() - n);
    if (count <= 0) { ::close(fd); return 2; }
    n += static_cast<size_t>(count);
  }
  return ::close(fd) == 0 ? 0 : 2;
}
