# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd "--cmd-pre-load=set DEVICE riscv")
board_runner_args(openocd "--cmd-load-hal=set DEVICE riscv")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
