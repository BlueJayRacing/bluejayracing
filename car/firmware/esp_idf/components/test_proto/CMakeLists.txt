set(PROTOC_PATH "${COMPONENT_DIR}/../nanopb/generator_bin/protoc")
set(PROTO_PATH "${COMPONENT_DIR}/../../../../proto/")
set(PROTO_OUT_PATH "${COMPONENT_DIR}/src")

list(APPEND CMAKE_MODULE_PATH "${COMPONENT_DIR}/cmake")
include(Functions)

get_directory_files(
    DIRECTORY ${PROTO_PATH}
    OUTPUT_VARIABLE PROTO_FILES
    PATTERN "*.proto"
)

add_prefix_postfix(FULL_PATH_PROTO_FILES "${PROTO_FILES}"
    PREFIX  "${PROTO_PATH}"
)

remove_prefix_postfix("${PROTO_FILES}" "" ".proto" PROTO_NAMES)

add_prefix_postfix( C_PROTO_FILES "${PROTO_NAMES}"
    PREFIX  "src/"
    POSTFIX ".pb.c"
)

clean_directory_preserve("${COMPONENT_DIR}/src")
write_files("${C_PROTO_FILES}" "${C_PROTO_FILES}" "${COMPONENT_DIR}/")

idf_component_register(SRCS ${C_PROTO_FILES}
                        INCLUDE_DIRS "src"
                        REQUIRES esp_system nanopb
                        )

add_custom_command(OUTPUT ${C_PROTO_FILES}
     COMMAND ${PROTOC_PATH} --proto_path=${PROTO_PATH} --nanopb_out=${PROTO_OUT_PATH} ${PROTO_FILES}
     DEPENDS ${FULL_PATH_PROTO_FILES}
     VERBATIM)

add_custom_target(protos DEPENDS ${C_PROTO_FILES})
add_dependencies(${COMPONENT_LIB} protos)

set_property(DIRECTORY "${COMPONENT_DIR}" APPEND PROPERTY
          ADDITIONAL_CLEAN_FILES ${C_PROTO_FILES})