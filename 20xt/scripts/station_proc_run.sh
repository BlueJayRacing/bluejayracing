#!/bin/bash
bash ./station_proc_kill.sh
bin/data_logger          > $(date +logs/%Y-%m-%d_%H-%M-%S_data_logger.txt) 2>&1 &
bin/receive_dispatcher   > $(date +logs/%Y-%m-%d_%H-%M-%S_receive_dispatcher.txt) 2>&1 &
bin/transmit_prioritizer > $(date +logs/%Y-%m-%d_%H-%M-%S_transmit_prioritizer.txt) 2>&1 &
bin/xbee_driver          > $(date +logs/%Y-%m-%d_%H-%M-%S_xbee_driver.txt) 2>&1 &
bin/rtk_corrections      > $(date +logs/%Y-%m-%d_%H-%M-%S_rtk_corrections.txt) 2>&1 &
bin/pit_commands
