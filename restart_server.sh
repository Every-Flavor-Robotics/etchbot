#!/bin/bash

export HOST_IP=$(
  if [[ "$OSTYPE" == "darwin"* ]]; then
    ipconfig getifaddr en0
  else
    hostname -I | awk '{print $1}'
  fi
)
docker-compose restart
