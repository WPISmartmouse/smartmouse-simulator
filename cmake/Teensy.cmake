# Copyright (c) 2015, Pierre-Andre Saulais <pasaulais@free.fr>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

find_program(TY_EXECUTABLE NAMES "teensy_loader_cli" DOC "Path to the executable that can upload programs to the Teensy")

macro(add_teensy_executable TARGET_NAME)
    # Build the ELF executable.
    add_executable(${TARGET_NAME} ${ARGN})
    target_link_libraries(${TARGET_NAME} teensy_core)
    set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME ${TARGET_NAME} SUFFIX ".elf")
    set(TARGET_ELF "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.elf")

    # Generate the hex firmware files that can be flashed to the MCU.
    set(EEPROM_OPTS -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0)
    set(HEX_OPTS -O ihex -R .eeprom)
    add_custom_command(OUTPUT ${TARGET_ELF}.eep
            COMMAND ${CMAKE_OBJCOPY} ${EEPROM_OPTS} ${TARGET_ELF} ${TARGET_ELF}.eep
            DEPENDS ${TARGET_ELF})
    add_custom_command(OUTPUT ${TARGET_ELF}.hex
            COMMAND ${CMAKE_OBJCOPY} ${HEX_OPTS} ${TARGET_ELF} ${TARGET_ELF}.hex
            DEPENDS ${TARGET_ELF})
    add_custom_target(${TARGET_NAME}_Firmware ALL
            DEPENDS ${TARGET_ELF}.eep ${TARGET_ELF}.hex)
    add_dependencies(${TARGET_NAME}_Firmware ${TARGET_NAME})

    if (NOT TY_EXECUTABLE)
        message("teensy_loader_cli does not exist! Will not generate ${TARGET_NAME}_Upload target")
    else ()
        add_custom_target(${TARGET_NAME}_Upload
                DEPENDS ${TY_EXECUTABLE} ${TARGET_ELF}.hex
                COMMAND "${TY_EXECUTABLE}" --mcu=TEENSY35 -w -s ${TARGET_ELF}.hex)
        add_dependencies(${TARGET_NAME}_Upload ${TARGET_NAME}_Firmware)
    endif ()
endmacro(add_teensy_executable)
