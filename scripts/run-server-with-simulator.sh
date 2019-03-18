#!/bin/sh

# Delete previous container
docker rm swam-indoor-flight

./docker-swam-indoor-flight.sh "tmuxinator start simulator"
