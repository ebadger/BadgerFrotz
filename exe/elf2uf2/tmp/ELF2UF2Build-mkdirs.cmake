# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/msys64/home/ebadger/pico-sdk/tools/elf2uf2"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/elf2uf2"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2/tmp"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2/src/ELF2UF2Build-stamp"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2/src"
  "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/msys64/home/ebadger/EB6502/Badger6502Pico/exe/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
