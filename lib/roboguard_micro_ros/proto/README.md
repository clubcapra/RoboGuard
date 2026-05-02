This folder contains the protobuf schema and nanopb options for RoboGuard telemetry and commands.

Files:
- `roboguard.proto` - protobuf definitions for `Telemetry` and `EstopCommand`.
- `roboguard.options` - nanopb generation options (max_count, max_size).

Generate nanopb sources with protoc + nanopb plugin. Example:

On Linux/macOS (with `protoc` and `protoc-gen-nanopb` on PATH):

```bash
protoc -I. --nanopb_out=../generated roboguard.proto
```

On Windows with PlatformIO environment you may need to provide full plugin path. After generation put the produced `.pb.c`/`.pb.h` into your project (for example `lib/roboguard_micro_ros/include`).

Notes:
- The field numbers in `roboguard.proto` match the constants used by the current encoder in `roboguard_micro_ros.cpp` so on-wire compatibility is preserved.
- Adjust `roboguard.options` `max_size` / `max_count` if you change array sizes.
