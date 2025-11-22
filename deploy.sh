#!/bin/sh

# Find the latest .cbun file by modification time
latest_cbun=$(ls -t build/*.cbun 2>/dev/null | head -n 1)

# Check if a file was found
if [ -z "$latest_cbun" ]; then
  echo "No .cbun files found in build/"
  exit 1
fi

# Copy the latest .cbun file to the rc_emulator docker container
docker cp "$latest_cbun" rc_emulator:/usr/share/cbuns