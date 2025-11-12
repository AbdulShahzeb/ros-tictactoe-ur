#!/bin/bash

angle=${1:-90}

# Validate input
if ! [[ "$angle" =~ ^[0-9]+$ ]] || [ "$angle" -lt 0 ] || [ "$angle" -gt 180 ]; then
  echo "Error: angle must be an integer between 0 and 180"
  exit 1
fi

echo "$angle" > /dev/ttyACM0
