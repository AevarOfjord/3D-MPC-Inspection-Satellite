# Contributing

## Development Setup

```bash
git clone <repo>
cd Cplusplus_build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

## Code Style

- **C++17** standard
- **4 spaces** indentation
- **snake_case** for functions/variables
- **PascalCase** for classes
- **UPPER_CASE** for constants

## Project Structure

```
include/         # Headers
src/             # Implementation
config/          # YAML configs
models/          # MuJoCo XML
assets/          # Meshes, textures
docs/            # Documentation
```

## Pull Request Process

1. Create feature branch: `git checkout -b feature/my-feature`
2. Make changes with tests
3. Ensure build passes: `make`
4. Update documentation if needed
5. Submit PR with description

## Testing

```bash
cd build
./sat_control ../config/mission_test.yaml
```

## Adding a New Module

1. Create header in `include/<module>/`
2. Create source in `src/<module>/`
3. Add to `CMakeLists.txt` SOURCES
4. Document in `docs/API_REFERENCE.md`
