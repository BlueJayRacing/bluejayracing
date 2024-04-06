#!/bin/bash
pkill -f data_logger
pkill -f receive_dispatcher
pkill -f transmit_prioritizer
pkill -f xbee_driver
pkill -f pit_commands