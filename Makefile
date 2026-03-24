CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -O2 -I./include
LDFLAGS = -lm

SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
BIN_NAME = kalman_demo

SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))

.PHONY: all clean run

all: $(BUILD_DIR) $(BIN_NAME)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BIN_NAME): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(BIN_NAME)
	./$(BIN_NAME)

clean:
	rm -rf $(BUILD_DIR) $(BIN_NAME)

# Simple test target (could be expanded with CUnit or similar)
test: clean all
	@echo "Running basic sanity check..."
	./$(BIN_NAME) > /dev/null && echo "Test passed." || echo "Test failed."
