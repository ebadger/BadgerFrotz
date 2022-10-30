file(GLOB FILES *.cpp *.h)
add_executable(badgerfrotz 
badgerfrotz.cpp
blorblib.cpp
buffer.cpp
dumb_blorb.cpp
dumb_init.cpp
dumb_input.cpp
dumb_output.cpp
dumb_pic.cpp
err.cpp
fastmem.cpp
files.cpp
getopt.cpp
hotkey.cpp
input.cpp
main.cpp
math.cpp
mound.cpp
object.cpp
process.cpp
quetzal.cpp
random.cpp
redirect.cpp
screen.cpp
stream.cpp
table.cpp
text.cpp
variable.cpp
)

pico_generate_pio_header(badgerfrotz ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(badgerfrotz ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(badgerfrotz ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

target_link_libraries(badgerfrotz pico_stdlib pico_multicore hardware_pio hardware_dma hardware_flash hardware_sync pico_mem_ops pico_util)

pico_add_extra_outputs(badgerfrotz)


