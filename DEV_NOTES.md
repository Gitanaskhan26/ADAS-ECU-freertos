## Development Environment Notes

### macOS Development with Linux Runtime

This project is designed to run in **Linux (Docker)** but can be edited on **macOS**.

#### Why you see warnings:

- **C++ Include Paths**: `/usr/include/eigen3` and `/usr/include/freertos` exist only in Docker
- **Python CAN Library**: `python-can` SocketCAN support requires Linux kernel
- **Compile Commands**: `build/compile_commands.json` generated only after building in Docker

#### IntelliSense Configuration:

Two configurations are provided in `.vscode/c_cpp_properties.json`:

1. **macOS (Development)** - For editing on macOS (default)
2. **Linux (Docker/Runtime)** - For when connected to Docker container

To switch: `Cmd+Shift+P` → "C/C++: Select a Configuration" → Choose "macOS (Development)"

#### Recommended Workflow:

**Option 1: Edit on macOS, Build in Docker (Current Setup)**
```bash
# On macOS: Edit code in VS Code
# Then build in Docker:
docker-compose run --rm adas_dev ./scripts/build.sh
```

**Option 2: Full Docker Development (Better IntelliSense)**
```bash
# Use VS Code Remote - Containers extension
# Open folder in container for full Linux environment
```

#### Install Remote Container Extension:

```bash
code --install-extension ms-vscode-remote.remote-containers
```

Then: `Cmd+Shift+P` → "Remote-Containers: Reopen in Container"

This gives you native Linux IntelliSense with all include paths resolved.

---

### Quick Reference

| Task | Command |
|------|---------|
| Build in Docker | `docker-compose run --rm adas_dev ./scripts/build.sh` |
| Run in Docker | `docker-compose run --rm adas_dev ./build/adas_ecu` |
| Generate Coverage | `./scripts/generate_coverage.sh` |
| View Coverage | `firefox build/coverage_html/index.html` |
| Run All Tests | `cd build/tests && for t in test_*; do ./$t; done` |
| Open in Container | VS Code: "Reopen in Container" |
| Switch C++ Config | VS Code: "C/C++: Select a Configuration" |

---

### Suppressing Warnings

The following are **expected and safe to ignore** on macOS:

- ✓ C++ include path warnings (`/usr/include/eigen3`, `/usr/include/freertos`)
- ✓ Missing `compile_commands.json` (generated after first Docker build)
- ✓ Python `can` module warnings (Linux-only library)

All code validation happens in Docker where all dependencies exist.
