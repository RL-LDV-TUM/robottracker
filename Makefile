CMAKE_DIR := build
OBJ_DIR   := obj
BIN_DIR   := bin

default: debug

debug: $(CMAKE_DIR)/debug $(BIN_DIR) $(OBJ_DIR)
	cd $(CMAKE_DIR)/debug && make --no-print-directory

release: $(CMAKE_DIR)/release $(BIN_DIR) $(OBJ_DIR)
	cd $(CMAKE_DIR)/release && make --no-print-directory

$(CMAKE_DIR)/release:
	mkdir -p $(CMAKE_DIR)/release && cd $(CMAKE_DIR)/release && cmake -DCMAKE_BUILD_TYPE=Release ../..

$(CMAKE_DIR)/debug:
	mkdir -p $(CMAKE_DIR)/debug && cd $(CMAKE_DIR)/debug && cmake -DCMAKE_BUILD_TYPE=Debug ../..

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

purge: clean
	rm -rf $(CMAKE_DIR)

help:
	@echo "Usage: make <target>"
	@echo "Available targets:"
	@echo "  debug          to build in debug mode"
	@echo "  release        to build in release mode"
	@echo "  clean          to clean build output"
	@echo "  purge          to clean and remove cmake configuration"
	@echo "  help           to show this message"

