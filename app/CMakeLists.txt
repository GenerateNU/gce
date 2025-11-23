cmake_minimum_required(VERSION 3.22)

set(ZEPHYR_TOOLCHAIN_VARIANT zephyr)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app)

target_sources(app PRIVATE 
    src/main.c
    src/system_states/system_states.c
    src/can_protocol/can_protocol.c
    src/board_roles/board_main.c
    src/board_roles/board_input.c
    src/board_roles/board_output.c
    src/ui/ui.c
    )
