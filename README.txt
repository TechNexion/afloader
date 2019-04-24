afloader - Axon Fabric Loader

A simple utility to program the Axon Fabric from the SOC

Usage: afloader <options> -d [SPI device file] -j [jedec file]
   -d <SPI device file>
       Example: -d /dev/spidev2.0
       Default if no -d option given: /dev/spidev2.2
   -j <Path to JEDEC file>
       Example: -j file.jed
   -i : Read the device ID and exit
   -q : Run quietly. Print no informational messages during execution.
   -r : Dry run only. Open and close files, and print information. Perform no erase or configuration.
   -h : Print help information.

To build:

This program uses gnu autotools to build.

$ autoreconf --install
$ ./configure --host <prefix of arm toolchain>

Example (for ARM64 targets):
$ ./configure --host aarch64-linux-gnu

Then build by running Make:
$ make

The result of the build is in the main project directory.

To rebuild:

$ make clean
$ make

To clean the configuration (you'll need to rerun ./configure):

$ make distclean

