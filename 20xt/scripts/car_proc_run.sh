#!/bin/bash

bash ./car_proc_kill.sh

bin/adc_driver             > $(date +logs/%Y-%m-%d_%H-%M-%S_adc_driver.txt) 2>&1 &
bin/mqtt_client            > $(date +logs/%Y-%m-%d_%H-%M-%S_mqtt_client.txt) 2>&1 &
bin/transmit_prioritizer   > $(date +logs/%Y-%m-%d_%H-%M-%S_transmit_prioritizer.txt) 2>&1 &
bin/broker                 > $(date +logs/%Y-%m-%d_%H-%M-%S_broker.txt) 2>&1 &
bin/sd_writer              > $(date +logs/%Y-%m-%d_%H-%M-%S_sd_writer.txt) 2>&1 &
