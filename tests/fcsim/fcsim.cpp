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
#include "sim_elf.h"
#include "sim_core.h"
#include "sim_gdb.h"
#include "sim_hex.h"
#include "sim_vcd_file.h"

#include "sim_core_decl.h"

static avr_t * avr = NULL;

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

    avr = avr_make_mcu_by_name(f.mmcu);
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

    for (;;) {
        int state = avr_run(avr);
        if (state == cpu_Done || state == cpu_Crashed)
                break;
    }

    avr_terminate(avr);

    return EXIT_SUCCESS;
}
