# Flight Path Calculator

A dual-language implementation (C++ and Python) of pathfinding algorithms for aviation applications. This project demonstrates algorithm implementation, spatial computing, and software engineering best practices relevant to aerospace software development.

## Overview

This project implements A* and RRT (Rapidly-exploring Random Tree) algorithms for calculating optimal flight paths in 2D/3D space, with consideration for:

- Obstacle avoidance (restricted airspaces, no-fly zones)
- Minimum turn radius constraints for realistic aircraft maneuvering
- Altitude optimization for fuel efficiency
- Multiple waypoint support

## Repository Structure

```
flight-path-calculator/
├── cpp/                    # C++ implementation
│   ├── src/               # Source files
│   ├── include/           # Header files
│   └── tests/             # Unit tests
├── python/                # Python implementation
│   ├── src/               # Source modules
│   └── tests/             # Unit tests
├── docs/                  # Documentation
└── README.md
```

## Quick Start

### C++ Version

```bash
cd cpp
mkdir build && cd build
cmake ..
make
./flight_path_calculator
```

### Python Version

```bash
cd python
pip install -e .
python -m flightpath.cli
```

## Algorithms

### A* Pathfinding
- Optimal path guaranteed
- Best for known, relatively static environments
- Configurable heuristic (Euclidean, Manhattan, or aviation-specific)

### RRT (Rapidly-exploring Random Tree)
- Suitable for dynamic obstacle environments
- Provides feasible paths even with complex constraints
- Randomized exploration with bias toward goal

## Features

- 2D and 3D path planning
- Configurable aircraft performance parameters
- Minimum turn radius constraints
- Waypoint-based navigation
- Path smoothing and optimization
- JSON input/output for integration
- Unit test suite

## Testing

### C++
```bash
cd cpp/build
ctest --output-on-failure
```

### Python
```bash
cd python
pytest tests/
```

## Requirements

### C++
- C++17 compatible compiler
- CMake 3.10+
- GoogleTest (included as submodule)

### Python
- Python 3.8+
- NumPy
- pytest

## License

MIT License
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
# flight-path-calculator
