#!/bin/bash

# Source ESP-IDF environment
source /opt/esp/idf/export.sh

# Export additional environment variables for VS Code
export IDF_PATH="/opt/esp/idf"
export IDF_TOOLS_PATH="/opt/esp/tools"

echo "ESP-IDF environment configured successfully!"
echo "IDF_PATH: $IDF_PATH"
echo "Compiler path: $(which xtensa-esp32-elf-gcc)"
