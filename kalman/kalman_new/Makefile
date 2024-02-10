CC = g++
CFLAGS = -g -std=c++11 -Wall -Wextra -pedantic

# Source and object files
SRCS = csim_main.cpp csim_funcs.cpp 
OBJS = $(SRCS:.cpp=.o)

# Target executable
TARGET = csim

# Default target
all: $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Rule to build object files from source files
%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

# Clean up the project
clean:
	rm -f $(TARGET) $(OBJS)