# Copyright (c) 2024 Infineon Technologies AG
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
    reset.S
    switch.S
    isr_wrapper.S
    irq_manage.c
    prep_c.c
    thread.c
    cpu_idle.c
    vectors.c
)

zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors vector_table.ld)
