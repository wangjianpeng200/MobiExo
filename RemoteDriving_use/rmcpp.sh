#! /bin/bash
find src -type f -name "*.h" -exec rm -rf {} \;
find src -type f -name "*.hpp" -exec rm -rf {} \;
find src -type f -name "*.c" -exec rm -rf {} \;
find src -type f -name "*.cpp" -exec rm -rf {} \;
