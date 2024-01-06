# Minimum required CMake version
cmake_minimum_required(VERSION 3.12)

# Project name
set(CMAKE_C_STANDARD 99)

# Add library source files
set(SOURCES
    src/mbedtls/aes.c 
    src/mbedtls/bignum.c 
    src/mbedtls/ctr_drbg.c 
    src/mbedtls/entropy_poll.c 
    src/mbedtls/entropy.c 
    src/mbedtls/mbedtls_util.c 
    src/mbedtls/sha256.c
    src/util/crc16buypass.c 
    src/util/hexdump.c 
    src/util/hexstrtobyte.c 
    src/util/jslong.c 
    src/util/memcheck.c 
    src/util/srp.c 
    src/util/swapbytes.c 
    src/util/swapcpy.c 
    src/util/xbee_random_mbedtls.c 
    src/util/xmodem_crc16.c 
    src/wpan/wpan_aps.c 
    src/wpan/wpan_types.c 
    src/xbee/pxbee_ota_client.c 
    src/xbee/pxbee_ota_server.c 
    src/xbee/xbee_atcmd.c 
    src/xbee/xbee_atmode.c 
    src/xbee/xbee_bl_gen3.c 
    src/xbee/xbee_cbuf.c 
    src/xbee/xbee_commissioning.c 
    src/xbee/xbee_delivery_status.c 
    src/xbee/xbee_device.c 
    src/xbee/xbee_discovery.c 
    src/xbee/xbee_ebl_file.c 
    src/xbee/xbee_ext_modem_status.c 
    src/xbee/xbee_file_system.c 
    src/xbee/xbee_firmware.c 
    src/xbee/xbee_gpm.c 
    src/xbee/xbee_io.c 
    src/xbee/xbee_ipv4.c 
    src/xbee/xbee_reg_descr.c 
    src/xbee/xbee_register_device.c 
    src/xbee/xbee_route.c 
    src/xbee/xbee_scan.c 
    src/xbee/xbee_secure_session.c 
    src/xbee/xbee_sms.c 
    src/xbee/xbee_socket_frames.c 
    src/xbee/xbee_socket.c 
    src/xbee/xbee_sxa.c 
    src/xbee/xbee_time.c 
    src/xbee/xbee_transparent_serial.c
    src/xbee/xbee_tx_status.c 
    src/xbee/xbee_user_data.c 
    src/xbee/xbee_wifi.c 
    src/xbee/xbee_wpan.c 
    src/xbee/xbee_xmodem.c
    src/zigbee/zcl_basic.c
    src/zigbee/zcl_client.c
    src/zigbee/zcl_commissioning.c
    src/zigbee/zcl_identify.c
    src/zigbee/zcl_onoff.c
    src/zigbee/zcl_ota_server.c
    src/zigbee/zcl_ota_upgrade.c
    src/zigbee/zcl_time.c
    src/zigbee/zcl_types.c
    src/zigbee/zigbee_zcl.c
    src/zigbee/zigbee_zdo.c
    ports/posix/xbee_platform_posix.c 
    ports/posix/xbee_readline.c 
    ports/posix/xbee_serial_posix.c
    # Add more source files here
)

# Add library header files
set(HEADERS
    include/mbedtls/aes.h 
    include/mbedtls/bignum.h 
    include/mbedtls/bn_mul.h 
    include/mbedtls/check_config.h 
    include/mbedtls/config.h 
    include/mbedtls/ctr_drbg.h 
    include/mbedtls/entropy_poll.h 
    include/mbedtls/entropy.h 
    include/mbedtls/platform_util.h 
    include/mbedtls/platform.h 
    include/mbedtls/sha256.h 
    include/util/crc16buypass.h 
    include/util/srp.h 
    include/wpan/aps.h 
    include/wpan/types.h 
    include/xbee/atcmd.h 
    include/xbee/atmode.h 
    include/xbee/bl_gen3.h 
    include/xbee/byteorder.h 
    include/xbee/cbuf.h 
    include/xbee/commissioning.h 
    include/xbee/delivery_status.h 
    include/xbee/device.h 
    include/xbee/discovery.h 
    include/xbee/ebl_file.h 
    include/xbee/ext_modem_status.h 
    include/xbee/file_system.h 
    include/xbee/firmware.h 
    include/xbee/gpm.h 
    include/xbee/io.h 
    include/xbee/ipv4.h 
    include/xbee/jslong_glue.h 
    include/xbee/jslong.h 
    include/xbee/platform.h 
    include/xbee/pxbee_ota_client.h 
    include/xbee/pxbee_ota_server.h 
    include/xbee/random.h 
    include/xbee/reg_descr.h 
    include/xbee/register_device.h 
    include/xbee/route.h 
    include/xbee/scan.h 
    include/xbee/secure_session.h 
    include/xbee/serial.h 
    include/xbee/sms.h 
    include/xbee/socket_frames.h 
    include/xbee/socket.h 
    include/xbee/sxa_socket.h 
    include/xbee/sxa.h 
    include/xbee/time.h 
    include/xbee/transparent_serial.h 
    include/xbee/tx_status.h
    include/xbee/user_data.h
    include/xbee/wifi.h
    include/xbee/wpan.h
    include/xbee/xmodem_crc16.h
    include/xbee/xmodem.h
    include/zigbee/zcl_bacnet.h
    include/zigbee/zcl_basic_attributes.h
    include/zigbee/zcl_basic.h
    include/zigbee/zcl_client.h
    include/zigbee/zcl_commissioning.h
    include/zigbee/zcl_identify.h
    include/zigbee/zcl_onoff.h
    include/zigbee/zcl_ota_server.h
    include/zigbee/zcl_ota_upgrade.h
    include/zigbee/zcl_time.h
    include/zigbee/zcl_types.h
    include/zigbee/zcl.h
    include/zigbee/zcl64.h
    include/zigbee/zdo.h
    ports/posix/platform_config.h
)

# Create library target
add_library(xbee_library STATIC ${SOURCES} ${HEADERS})

# This library has been modified so it is only compatible with POSIX devices
target_compile_definitions(xbee_library PUBLIC "POSIX")
target_include_directories(xbee_library PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ports/posix
)

target_link_libraries(xbee_library
    # Add any required libraries here
)