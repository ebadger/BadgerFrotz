# BadgerFrotz
Frotz Z-machine ported to a Raspberry PI Pico designed to run on the ebadger LodeRunner platform

There are a few components in this project that combined create something fun and new.

1) VGA on the Pico - mostly taken from https://vanhunteradams.com/Pico/VGA/VGA.html
2) An 80 column terminal including font rendering and PS/2 keyboard input
3) Frotz Z-Machine port.  Started with https://github.com/DavidGriffith/frotz and rewrote file system, input, output, save/restore routines

The project demonstrates how to utilize the Pico's flash memory in a way that works with multiple cores and the DMA/PIO generating the VGA signal.
Writing to the flash is moderately painful since no code can be running from the flash while writing to it. As such, all running code must be in RAM,
DMA can't be running.  The project demonstrates how to write to flash and then reestablish the VGA signal.

![0](https://user-images.githubusercontent.com/7229532/198904962-b18ab4f1-21a5-46bd-87bb-71daa843e2fd.png)

![0](https://user-images.githubusercontent.com/7229532/198905486-305a9765-1ebe-4df7-ae62-6322271d3297.png)

It should be failry easy to adapt the code to custom hardware, or to a Pimoroni VGA board, etc...

If you have an ebadger LodeRunner appliance, it will run without any change on that device.
I have a bunch of PCBs that need a Pico, a VGA port and a PS/2 port soldered.
[PDF documentation](https://github.com/ebadger/BadgerFrotz/blob/main/eb6502pico-manual.pdf)
Send email to eric.badger@gmail.com for more info on the hardware if you're interested in buying one to support this project.

![FVkIgqnUYAAFe1m](https://user-images.githubusercontent.com/7229532/198905597-a85daf7f-2c76-4f9b-9c10-262700c0f92c.jpg)
![FVkI9tuVUAIgpaC](https://user-images.githubusercontent.com/7229532/198905605-ced0b7f5-948c-4844-87c2-9cd39e94daff.jpg)

LodeRunner hardware demonstrated here:  

https://blog.pishop.co.za/raspberry-pi-pico-emulates-6502-computer-and-runs-loderunner/


and here

https://twitter.com/ebad73/status/1548135753720348673?s=20&t=SdgSOiQ7SnGn88zoFUkfwA

If you create a derivative build, please share photos!
Please feel free to contribute changes

There are various instructions on the web on how to set up a build environment in Windows with VSCode to build C++ for the Pi Pico
For example:
https://projects-raspberry.com/setting-up-the-raspberry-pi-pico-for-c-c-development-on-windows/
