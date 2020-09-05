/*
"Forumscontroller" AVR hardware simulator.
Copyright (C) 2020 Thomas Jarosch.

Based upon "simavr" framework.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>
#include <string.h>
#include <signal.h>

#include "sim_avr.h"
#include "sim_time.h"
#include "sim_elf.h"
#include "sim_core.h"
#include "sim_gdb.h"
#include "sim_hex.h"
#include "sim_vcd_file.h"
#include "avr_ioport.h"
#include "avr_adc.h"

#include "sim_core_decl.h"

enum {
    IRQ_FCSIM_WHEEL = 0,
    IRQ_FCSIM_PAS,
    IRQ_FCSIM_BRAKE,
    _IRQ_FCSIM_COUNT
};

static const char *_fcsim_irq_names[_IRQ_FCSIM_COUNT] = {
    [IRQ_FCSIM_WHEEL] = ">fcsim.wheel",
    [IRQ_FCSIM_PAS] = ">fcsim.pas",
    [IRQ_FCSIM_BRAKE] = ">fcsim.brake",
};

typedef struct fcsim_irq_t {
    avr_irq_t *irq;
    struct avr_t *avr;
    uint8_t PAS_irq_state;
} fcsim_irq_t;

#define USECS_PER_SECOND (1000 * 1000)

static avr_cycle_count_t 
    fcsim_wheel_timer_release(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    // fprintf(stderr, "[FCSIM] called: fcsim_wheel_timer_release: %u\n", when);

    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_WHEEL, 0);

    return 0;       // do not re-schedule
}

static avr_cycle_count_t 
    fcsim_wheel_timer(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    // fprintf(stderr, "[FCSIM] called: fcsim_wheel_timer: %u\n", when);

    // wheel timer just cares abour rising edge
    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_WHEEL, 1);

    // schedule wheel IRQ release
    avr_cycle_timer_register_usec(
            fcsim_irq->avr,
            25 /*ms*/ * 1000,
            fcsim_wheel_timer_release,
            param);

    // return value is absolute cycle counter for the next call
    // TOMJ: wheel_time = 3600 * wheel_circumference / spd
    return when + avr_usec_to_cycles(avr, 1057 /* ms */ * 1000);
}


static avr_cycle_count_t 
    fcsim_pas_timer_release(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    // fprintf(stderr, "[FCSIM] called: fcsim_pas_timer_release: %u\n", when);

    fcsim_irq->PAS_irq_state = !fcsim_irq->PAS_irq_state;
    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_PAS, fcsim_irq->PAS_irq_state);

    return 0;       // do not re-schedule
}

static avr_cycle_count_t 
    fcsim_pas_timer(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    // fprintf(stderr, "[FCSIM] called: fcsim_pas_timer: %u\n", when);

    // PAS IRQ care about rising and falling edge
    fcsim_irq->PAS_irq_state = !fcsim_irq->PAS_irq_state;
    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_PAS, fcsim_irq->PAS_irq_state);

    // schedule wheel IRQ release
    avr_cycle_timer_register_usec(
            fcsim_irq->avr,
            350 * 1000,
            fcsim_pas_timer_release,
            param);

    // return value is absolute cycle counter for the next call
    return when + avr_usec_to_cycles(avr, 500 /* ms */ * 1000);
}

static avr_cycle_count_t 
    fcsim_brake_release(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    fprintf(stderr, "[FCSIM] called: fcsim_brake_release: %u\n", when);
    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_BRAKE, 1);

    return 0;
}

static avr_cycle_count_t 
    fcsim_brake_timer(
        avr_t * avr,
        avr_cycle_count_t when,
        void * param)
{
    fcsim_irq_t *fcsim_irq = (fcsim_irq_t *) param;

    fprintf(stderr, "[FCSIM] called: fcsim_brake_timer: %u\n", when);

    avr_raise_irq(fcsim_irq->irq + IRQ_FCSIM_BRAKE, 0);

    // schedule brake release
    avr_cycle_timer_register_usec(
            fcsim_irq->avr,
            USECS_PER_SECOND * 3,
            fcsim_brake_release,
            param);

    return 0;
}

int main(int argc, char *argv[])
{
    elf_firmware_t f = {{0}};
    uint32_t loadBase = AVR_SEGMENT_OFFSET_FLASH;
    const char *firmware_fname = "pcontroller.elf";

    if (elf_read_firmware(firmware_fname, &f) == -1) {
        fprintf(stderr, "%s: Unable to load firmware from file %s\n",
                        argv[0], firmware_fname);
        exit(1);
    }

    // hardcoded for FC 2.x for now
    strcpy(f.mmcu, "atmega2560");
    f.frequency = 16000000;

    avr_t *avr = avr_make_mcu_by_name(f.mmcu);
    if (!avr) {
            fprintf(stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
            exit(1);
    }

    avr_init(avr);
    avr->log = LOG_TRACE;

    // init (ADC) voltages
    avr->vcc  = 5000;
    avr->aref = 5000;
    avr->avcc = 5000;

    avr_load_firmware(avr, &f);

    // TODO: EEPROM from extra file support?

    // init (fake) IRQs
    fcsim_irq_t fcsim_irq;
    memset(&fcsim_irq, 0, sizeof(fcsim_irq_t));

    fcsim_irq.irq = avr_alloc_irq(&avr->irq_pool,
                                  0,    // base
                                  _IRQ_FCSIM_COUNT,  // total count
                                  _fcsim_irq_names);

    fcsim_irq.avr = avr;

    // connect virtual IRQ to real INT7 (wheel)
    avr_connect_irq(
            fcsim_irq.irq + IRQ_FCSIM_WHEEL,
            avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('E'), 7));

    avr_connect_irq(
            fcsim_irq.irq + IRQ_FCSIM_PAS,
            avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('E'), 5));

    avr_connect_irq(
            fcsim_irq.irq + IRQ_FCSIM_BRAKE,
            avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('E'), 4));

    // register periodic wheel timer
    avr_cycle_timer_register_usec(
            avr,
            USECS_PER_SECOND / 2,
            fcsim_wheel_timer,
            &fcsim_irq);

    // register periodic PAS timer
    avr_cycle_timer_register_usec(
            avr,
            USECS_PER_SECOND / 2,
            fcsim_pas_timer,
            &fcsim_irq);

    // virtual battery input voltages
    struct avr_irq_t *battery_v_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC14);
    avr_raise_irq(battery_v_irq, 36 * (5000/60) /* full 5V equals 60 batt voltage */ );

    // virtual current
    struct avr_irq_t *current_A_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC15);
    avr_raise_irq(current_A_irq, 200 /* 5A -> 1,5V */ );

    // virtual poti
    struct avr_irq_t *poti_in_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC4);
    avr_raise_irq(poti_in_irq, 2 * 1000 /* 5V max */ );

    // virtual throttle
    struct avr_irq_t *throttle_in_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC3);
    avr_raise_irq(throttle_in_irq, 4 * 1000 /* 5V max */ );

    // trigger brake after 5 seconds
    avr_cycle_timer_register_usec(
            avr,
            USECS_PER_SECOND *5,
            fcsim_brake_timer,
            &fcsim_irq);

    for (;;) {
        int state = avr_run(avr);
        if (state == cpu_Done || state == cpu_Crashed)
                break;
    }

    avr_terminate(avr);

    return EXIT_SUCCESS;
}
