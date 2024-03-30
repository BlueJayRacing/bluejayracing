#!/bin/bash
pkill -f adc_driver
pkill -f mqtt_client
pkill -f transmit_prioritizer
pkill -f broker
pkill -f sd_writer
pkill -f xbee_driver
