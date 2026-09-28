# Compiler settings
CXX = clang++
CC  = clang

# C++ Flags (for your code)
#
#CXXFLAGS = -std=c++17 -g -O0 -Wall -Iheaders -I/opt/homebrew/include
CXXFLAGS = -std=c++17 -O3 -Wall -Iheaders -I/opt/homebrew/include
CXXFLAGS += -MMD -MP

# C Flags (specifically for miniz.c - removing -std=c++17)

CFLAGS = -O3 -Wall -Iheaders -I/opt/homebrew/include
CFLAGS += -MMD -MP

# Name of the executable
TARGET = raytracer

# Directories
SRC_DIR = src
OBJ_DIR = obj

# Find sources: Both .cpp (your code) and .c (miniz)
CPP_SRCS = $(wildcard $(SRC_DIR)/*.cpp)
C_SRCS   = $(wildcard $(SRC_DIR)/*.c)

# Create object lists for both
OBJS  = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_SRCS))
OBJS += $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(C_SRCS))

# Dependency files
DEPS = $(OBJS:.o=.d)

# Default target
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJS)
	@mkdir -p my_outputs 
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Rule: Compile C++ files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule: Compile C files (miniz.c)
# Uses $(CC) and $(CFLAGS) to avoid C++ standard errors on C code
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Include dependencies
-include $(DEPS)

# Clean
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean