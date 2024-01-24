#!/bin/bash

sudo mkswap /swap/swapfile  # start for each time
sudo swapon /swap/swapfile
sudo swapon --show
