set(simavr_SOURCES
    avr_acomp.c
    avr_acomp.h
    avr_adc.c
    avr_adc.h
    avr_bitbang.c
    avr_bitbang.h
    avr_eeprom.c
    avr_eeprom.h
    avr_extint.c
    avr_extint.h
    avr_flash.c
    avr_flash.h
    avr_ioport.c
    avr_ioport.h
    avr_lin.c
    avr_lin.h
    avr_spi.c
    avr_spi.h
    avr_timer.c
    avr_timer.h
    avr_twi.c
    avr_twi.h
    avr_uart.c
    avr_uart.h
    avr_usb.c
    avr_usb.h
    avr_watchdog.c
    avr_watchdog.h
    fifo_declare.h
    sim_avr.c
    sim_avr.h
    sim_avr_types.h
    sim_cmds.c
    sim_cmds.h
    sim_core.c
    sim_core.h
    sim_cycle_timers.c
    sim_cycle_timers.h
    sim_elf.c
    sim_elf.h
    sim_gdb.c
    sim_gdb.h
    sim_hex.c
    sim_hex.h
    sim_interrupts.c
    sim_interrupts.h
    sim_io.c
    sim_io.h
    sim_irq.c
    sim_irq.h
    sim_network.h
    sim_regbit.h
    sim_time.h
    sim_utils.c
    sim_utils.h
    sim_vcd_file.c
    sim_vcd_file.h
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../cores)

add_library(simavr SHARED ${simavr_SOURCES})
target_link_libraries(simavr simavr_cores elf)

add_executable(run_avr run_avr.c)
target_link_libraries(run_avr simavr)
