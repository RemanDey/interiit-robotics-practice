# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -Wall -I.

# Source files
SRC = sim/sim.cpp common/id.cpp

# Output executable (no .exe extension: Linux Files opens .exe as archive)
TARGET = simulator

# Default rule
all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

# Clean build files (also remove legacy sim.exe once)
clean:
	rm -f $(TARGET) sim.exe