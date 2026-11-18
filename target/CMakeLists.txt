# Bare-metal firmware for the Cortex-M3 (QEMU board mps2-an385).

add_executable(aeb_firmware startup.cpp semihosting.cpp libc_min.cpp main.cpp)
set_target_properties(aeb_firmware PROPERTIES
  SUFFIX ".elf"
  LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld)
target_compile_options(aeb_firmware PRIVATE -Wall -Wextra)
set_source_files_properties(libc_min.cpp PROPERTIES COMPILE_OPTIONS -fno-tree-loop-distribute-patterns)

# libgcc provides the software floating-point routines (the M3 has no FPU).
target_link_libraries(aeb_firmware PRIVATE aeb_sim aeb_core gcc)
target_link_options(aeb_firmware PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/aeb_firmware.map
  -Wl,--print-memory-usage)

add_custom_command(TARGET aeb_firmware POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:aeb_firmware>
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:aeb_firmware> ${CMAKE_CURRENT_BINARY_DIR}/aeb_firmware.bin)

# cmake --build --preset cortex-m3 --target run
add_custom_target(run
  COMMAND qemu-system-arm -M mps2-an385 -cpu cortex-m3 -nographic -monitor none
          -semihosting-config enable=on,target=native -kernel $<TARGET_FILE:aeb_firmware>
  DEPENDS aeb_firmware
  USES_TERMINAL)
