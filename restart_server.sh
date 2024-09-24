#!/bin/bash

export HOST_IP=$(hostname -I | awk '{print $1}')

docker-compose restart