set(NANOPB_SRC "${COMPONENT_DIR}/../../../../common/nanopb/src")
set(NANOPB_INCLUDE "${COMPONENT_DIR}/../../../../common/nanopb/include")

file(COPY ${NANOPB_SRC} DESTINATION ${COMPONENT_DIR})
file(COPY ${NANOPB_INCLUDE} DESTINATION ${COMPONENT_DIR})

idf_component_register(SRCS "src/pb_common.c" "src/pb_encode.c" "src/pb_decode.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_system
                        )