.PHONY: all cpp python test clean help

all: cpp python

cpp:
	@echo "Building C++ version..."
	@mkdir -p cpp/build
	@cd cpp/build && cmake .. -DBUILD_TESTS=ON && make

python:
	@echo "Setting up Python version..."
	@cd python && pip install -e . 2>/dev/null || pip install -e . --user

test: test-cpp test-python

test-cpp:
	@echo "Running C++ tests..."
	@cd cpp/build && ctest --output-on-failure

test-python:
	@echo "Running Python tests..."
	@cd python && python -m pytest tests/ -v

run:
	@cd cpp/build && ./flight_path_calculator

clean:
	@rm -rf cpp/build
	@cd python && rm -rf build dist *.egg-info

help:
	@echo "Flight Path Calculator - Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build both C++ and Python versions"
	@echo "  cpp        - Build C++ version"
	@echo "  python     - Install Python version"
	@echo "  test       - Run all tests"
	@echo "  test-cpp   - Run C++ tests"
	@echo "  test-python- Run Python tests"
	@echo "  run        - Run C++ executable"
	@echo "  clean      - Remove build artifacts"
