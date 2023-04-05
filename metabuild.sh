#!/bin/bash

# Check if a Makefile exists in the current directory
cd 3rdparty/svg2gcode/
if [ ! -f Makefile ]; then
  echo "Error: No Makefile found in the current directory."
  exit 1
fi

# Set the default build mode to release
BUILD_MODE="release"

# Parse command-line arguments
while [ "$#" -gt 0 ]; do
  case "$1" in
    release|-r)
      BUILD_MODE="release"
      shift
      ;;
    debug|-d)
      BUILD_MODE="debug"
      shift
      ;;
    *)
      echo "Error: Unknown option: $1"
      echo "Usage: $0 [release|-r] [debug|-d]"
      exit 1
      ;;
  esac
done

# Run the make command with the specified build mode
make BUILD_MODE="$BUILD_MODE"
