/*
 *  This code file compiles to an executable for programming and verifying
 *  the configuration of the AXON Fabric on TechNexion's AXON series of
 *  System on Modules.
 *
 *  (C) 2019 John Weber <john.weber@technexion.com>, TechNexion
 *
 *  Derived from fipsyloader.c, which was originally derived from
 *  software to support the Backhauler PMOD.
 *  https://github.com/MocoMakers/Fipsy-FPGA-edu
 *  https://github.com/AlliedComponentWorks/BackHaulerPMOD
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <ctype.h>
#include <errno.h>
#include "MachXO3.h"

/* Convenient types */
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

/* Windows function conversions */
#define SLEEP(t)	usleep(t*1000)

/* Subroutine declarations */
u32 af_disable_shared_sysconfig(void);
u32 af_open(void);
u32 af_close(void);
u32 af_read_device_id(u8 *DeviceID);
u32 af_read_unique_id(u8 *UniqueID);
u32 af_erase_all(void);
u32 af_load_configuration(void);
u32 af_write_features(u8 *FeatureRow, u8 *Feabits);
u32 af_write_configuration(char *JEDECFileName);
char jedec_seek_next_non_ws(void);
char jedec_seek_next_key_char(void);
u8 jedec_read_fuse_byte(u8 *FuseByte);

/* General purpose subroutine declarations */
u32 af_spi_transaction(u8 Count, void *Data);
int af_error_message(char *ErrorDescription, char *ErrorType);

// Predefined error messages
#define af_err_file_not_found()   af_error_message("Unable to open the specified file", "File Error")
#define af_err_file_format()      af_error_message("File format is not valid for JEDEC", "File Error")
#define af_err_bad_setting()      af_error_message("JEDEC file has SPI slave port disabled - programming aborted", "File Error")
#define af_err_bad_value()        af_error_message("Specified value is out of range", "Parameter Error")
#define af_err_bad_length()       af_error_message("Requested length is not valid", "Parameter Error")
#define af_err_bad_address()      af_error_message("Bad offset or length given", "Parameter Error")
#define af_err_null_ptr()         af_error_message("Data pointer provided is NULL", "Parameter Error")
#define af_err_not_erased()       af_error_message("The FPGA must be erased to program", "Operation Order Error")
#define af_err_not_open()         af_error_message("The SPI connection has not been initialized", "Operation Order Error")
#define af_err_timeout()          af_error_message("Timed out waiting for FPGA busy", "Timeout Error")
#define af_err_bad_usage()        af_error_message("afloader <jedec file name>", "Usage")
#define af_err_hardware()	      af_error_message("Could not open SPI port", "Hardware Error")

/* Local data definitions */

// General purpose message buffer
char gUserMsg[1000];

// SPI port stream identifier or handle
int gSPIPort = 0;

// SPI device file path
char gSpiDevFilePath[100] = "/dev/spidev2.2";

// Axon Fabric path
char gFabricSysfsPath[100] = "/sys/bus/i2c/drivers/axonfabric/3-0041/shared_sysconfig_disable";

// JEDEC file path
char gJEDECFilePath[256];

// General purpose buffer used to transact data on SPI
// This is bigger than most routines need, but reduces repeated declarations
// and is bigger than the actual SPI transaction can be, meaning there is
// always enough room
u8 gSPIBuf[100];

// Count of bytes in the SPI buffer
// This is filled based on the count to send in a transaction
// On return from a transaction, this will specify the number of bytes returned
// Bytes returned is the entire SPI transaction, not just the data, so the value
// should not change unless something went wrong.
u8 gMachXO3Count = -1;

// Macro to set the most frequently used dummy values for the SPI buffer
u8 SPIBUF_DEFAULT[20] = { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define SPIBUFINIT                  { gMachXO3Count = 0; memcpy(gSPIBuf, SPIBUF_DEFAULT, 20); }

// Defines for key elements of SPI transactions
#define MACHXO3_COMMAND             gSPIBuf[0]
#define PTR_MACHXO3_OPERAND         (&gSPIBuf[1])
#define PTR_MACHXO3_DATA            (&gSPIBuf[4])

// Macro to complete an SPI transaction of the specified count with the global buffer
// The global buffer count is set with the parameter, saving coding steps
#define MACHXO3_SPITRANS(c)         { gMachXO3Count = c; af_spi_transaction(gMachXO3Count, gSPIBuf); }

// Flag indicating hardware has been opened and the port is ready
#define HW_IS_OPEN				(gSPIPort > 0)

// Flag indicating that a caller has erased the FPGA
u8 gFPGAIsErased = 0;

// Flag to prevent the display of information messages
int gRunQuiet = 0;

// Flag to indicate a dry run. In this mode, the program does not perform any configuration.
// It will attempt to open and close files, and print informational messages.
int gDryRun = 0;

// File stream pointer for JEDEC file
// See comments with the parsing support functions
FILE *g_pJedecFile = NULL;

// Predefined advanced error message macro including closing the file on a format error
#define EXIT_FILEFORMATERROR		{ fclose(g_pJedecFile); return(af_err_file_format()); }
#define EXIT_FILEBADSETTINGERROR	{ fclose(g_pJedecFile); return(af_err_bad_setting()); }

/*--------------------------------------------------------------------------*/
/* System Level Functions                                                   */
/*--------------------------------------------------------------------------*/

void print_help() {
	printf("afloader: Axon Fabric Loader\n");
	printf("Usage: afloader <options> -d [SPI device file] -j [jedec file]\n");
	printf("   -d <SPI device file>\n");
	printf("       Example: -d /dev/spidev2.0\n");
	printf("       Default if no -d option given: /dev/spidev2.2\n");
	printf("   -j <Path to JEDEC file>\n");
	printf("       Example: -j file.jed\n");
	printf("   -i : Read the device ID and exit\n");
	printf("   -q : Run quietly. Print no informational messages during execution.\n");
	printf("   -r : Dry run only. Open and close files, and print information. Perform no erase or configuration.\n");
	printf("   -h : Print this help.\n");
}

/*
   MAIN accepts a command line argument of the JEDEC file to be written to
   the part and proceeds with part communication.  To verify communication,
   device id codes are read and printed.  Then the device is erased and
   programmed from the specified JEDEC file.
*/

int main (int argc, char *argv[]) {
	int c;
	u8 device_id[10];
	u8 unique_id[16];

	opterr = 0;
	int print_id_only = 0;

	while ((c = getopt(argc, argv, "iqhred:j:")) != -1) {
		switch (c) {
			case 'd': // SPI device file name
				sprintf(gSpiDevFilePath,"%s",optarg);
				break;
			case 'j': // JEDEC filename
				sprintf(gJEDECFilePath,"%s",optarg);
				break;
			case 'i': // Read the device ID and exit
				print_id_only = 1;
				break;
			case 'q': // Run quietly, do not print messages
				gRunQuiet = 1;
				break;
			case 'r': // Dry run - perform no configuration, just print information
				gRunQuiet = 0;
				gDryRun = 1;
				break;
			case 'h': // Print help message and exit
				print_help();
				return (0);
				break;
			case '?':
				if( (optopt == 'j') || (optopt == 'd') )
					printf("Option -%c requires an argument.\n", optopt);
				else if (isprint (optopt))
					printf("Unknown option `-%c'.\n", optopt);
       			else
				   printf("Unknown option character `\\x%x'.\n",optopt);
			default :
				return(-1);
		}
	}

	if(gDryRun) {
		printf("Dry run only. Fabric configuration will NOT be affected. Erase and programming are faked\n");
	}

	// Disable the shared SYSconfig pins
	if(af_disable_shared_sysconfig()) {
		return(af_err_hardware());
	}

	// Configure SPI connection to module
	if(af_open()) {
		return(af_err_hardware());
	}

	// Read and print the ids
	if(af_read_device_id(device_id)) {
		af_close();
		return(-1);
	};

	if(af_read_unique_id(unique_id)) {
		af_close();
		return(-1);
	};

	if(!gRunQuiet) {
		printf("Device ID = %02X %02X %02X %02X\n",
			device_id[0], device_id[1], device_id[2], device_id[3]);
		printf("Unique ID = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			unique_id[0], unique_id[1], unique_id[2], unique_id[3],
			unique_id[4], unique_id[5], unique_id[6], unique_id[7]);
	}

	if(print_id_only) {
		af_close();
		return(0);
	}

	// FPGA programming - these print error messages if they fail
	// Try to erase the device
	if(af_erase_all()) {
		af_close();
		return(-1);
	};

	// Program FPGA
	if(af_write_configuration(gJEDECFilePath)) {
		af_close();
		return(-1);
	};

	// Verify the FPGA - TODO

	// Return success
	return(0);
}

/*--------------------------------------------------------------------------*/
/* Exported Functions                                                       */
/*--------------------------------------------------------------------------*/

/*
	af_disable_shared_sysconfig will first attempt to read the
	'shared_sysconfig_disable' file in the sysfs path. If it can read this
	file, it will attempt to write '1' to it which will disable the fabric
	I/O that are shared with the sysconfig signals. On Axon, the SPI_B bus
	connects to both the regular fabric I/O and also the sysconfig SPI
	slave pins. It is required to disable the fabric I/O connected to these
	pins in order to prevent contention during the fabric programming
	process.
*/

u32 af_disable_shared_sysconfig(void) {

	// First attempt to open the file. If the file does not exist, then it
	// it is likely that the axonfabric driver failed to probe the fabric,
	// and the fabric is not programmed.

	int ssd_fd;
	char buf[10];
	int ret;

	ssd_fd = open(gFabricSysfsPath, O_RDWR);

	if(ssd_fd < 0) {
		printf("Unable to open fabric sysfs file \"%s\": %s\n", gFabricSysfsPath, strerror(errno));
		printf("Fabric is likely not programmed, or axonfabric driver is not loaded\n");
		printf("This will not stop the programming process\n");
		return(0);  // Return normal
	}

	/* Read the file */
	ret = read(ssd_fd, buf, 1);
	if(ret < 1) {
		printf("Unable to read sysfs file: %s\n", strerror(errno));
		return (1); // This is an error and we should stop the programming process.
	}

	printf("Shared SYSconfig Disable bit set to %c\n", buf[0]);

	if(buf[0] == '0') {
		printf("Disabling shared SYSconfig I/O on fabric... \n");
		ret = write(ssd_fd,"1",1);
		if(ret < 0) {
			printf("Unable to write sysfs file: %s\n", strerror(errno));
			return(1);
		}
	}

	printf("Shared SYSconfig disabled\n");

	close(ssd_fd);

	return(0);
}

/*
	af_open will establish a connection to the FPGA through the chosen
	SPI port.  This is typically used to open hardware, which is dependent
	on the target system, but also sets the port for operation as required
	from here.

	More checking of return values is done here than in other routines,
	with the idea that if things are working here they should continue to
	work well in other places if the hardware remains connected and the
	coding is correct.  This is executed first, so a failure here would
	mean that something is wrong with the hardware.

	A valid handle indicates that the hardware has been opened.  This fact
	is tested by a global macro, removing the need for any other indication
	of the expected hardware state.
*/

u32 af_open(void) {
	int ret;
	u8 mode = 0;
	u8 bits = 8;
	u32 speed = 400000;

	// Open the port
	if((gSPIPort = open(gSpiDevFilePath, O_RDWR)) < 0) return(1);

	// Set configurations for this port
	// This appears to be setting up a default or set of limits
	// For us it makes sure we have a valid port
	ret = ioctl(gSPIPort, SPI_IOC_WR_MODE, &mode);
	if (ret != -1) ret = ioctl(gSPIPort, SPI_IOC_RD_MODE, &mode);
	if (ret != -1) ret = ioctl(gSPIPort, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret != -1) ret = ioctl(gSPIPort, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret != -1) ret = ioctl(gSPIPort, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret != -1) ret = ioctl(gSPIPort, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) { af_close(); return(1); };

	// Send a NOP to wakeup the device
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_NOP;
	PTR_MACHXO3_OPERAND[0] = 0xFF;
	PTR_MACHXO3_OPERAND[1] = 0xFF;
	PTR_MACHXO3_OPERAND[2] = 0xFF;
	MACHXO3_SPITRANS(4);

	if(!gRunQuiet)
		printf("Successfully opened SPI device %s\n", gSpiDevFilePath);

	// Return success
	return(0);
}

/*
 * af_close() closes the SPI port connection.
 */

u32 af_close(void) {

	// Close the port
	close(gSPIPort);

	gSPIPort = -1;

	if(!gRunQuiet)
		printf("Closed SPI device %s\n", gSpiDevFilePath);

	// Return success
	return(0);
}

/* af_read_device_id retrieves the device identification number from the
   FPGA connected to the SPI port.  This number can be used to verify that
   the SPI is working and we are talking to the right device.  To improve
   future flexibility, this routine does not decide if this is actually
   the right device, but just returns the four bytes it got.
*/

u32 af_read_device_id(u8 *DeviceID) {
	// All exported library functions get this check of hardware and arguments
	if(!HW_IS_OPEN) {
		return(af_err_not_open());
	}
	if(DeviceID == NULL) {
		return(af_err_null_ptr());
	};

	// Construct the command
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_READ_DEVICEID;
	MACHXO3_SPITRANS(8);

	// Get the data
	memcpy(DeviceID, PTR_MACHXO3_DATA, 4);

	// Return success
	return(0);
}

/* af_read_unique_id retrieves the 8-byte unique identification code
   from the FPGA connected to the SPI port.  This number can be used to
   identify a specific chip, and therefore a specific assembly too.
   There is no specific use for this information in this context, but it
   is is sometimes used alone or connection with the user id (if implemented)
   to form a type of serial number.  It can take the place of other chips
   and resources designed to do just this, so it might be a useful features
   to expose here.  The bytes are returned in a simple array.
*/

u32 af_read_unique_id(u8 *UniqueID) {
	// All exported library functions get this check of hardware and arguments
	if(!HW_IS_OPEN) return(af_err_not_open());
	if(UniqueID == NULL) return(af_err_null_ptr());

	// Construct the command
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_READ_UIDCODE;
	MACHXO3_SPITRANS(12);

	// Copy the data to the argument
	memcpy(UniqueID, PTR_MACHXO3_DATA, 8);

	// Return success
	return(0);
}

/* af_erase_all clears the configuration from all portions of the FPGA.
   By this choice, the FPGA will return to its erased state function here.

   For this library, this is the first step to programming.  This is the
   function that enters the programming mode, so it must be completed
   before the programming operation.  A global flag is set here to be sure
   that happens.  This removes the need to blank check elsewhere.

   It is possible that in the future we will want to erase just part of
   the FPGA configuration or do some other more elaborate things, but this
   is not contemplated here other than in the name of this function.
*/

u32 af_erase_all(void) {
	u8 busy = 0x80;
	u32 timeout = 0;

	// All exported library functions get this check of hardware and arguments
	if(!HW_IS_OPEN) return(af_err_not_open());

	// Since we are now messing with the FPGA, indicate its status is not erased
	gFPGAIsErased = 0;

	if(!gRunQuiet)
		printf("   Erasing Fabric...");

	if(gDryRun) goto exit;

	// Send command to enter programming mode
	// We use only offline mode
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_ENABLE_OFFLINE;
	PTR_MACHXO3_OPERAND[0] = 0x08;
	MACHXO3_SPITRANS(4);

	// Apparently this mode change takes a brief moment, ok to just wait it out here
	SLEEP(1);

	// Send command to erase everything
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_ERASE;
	PTR_MACHXO3_OPERAND[0] = 0x0F;
	MACHXO3_SPITRANS(4);

	// Look at busy status every so often until it is clear or until too much time goes by
	// The busy bit is in the MSB, but still means we can test nonzero
	do {
		// Do a wait between polls of the busy bit
		SLEEP(100);
		timeout += 100;
		if(timeout > 20000) return(af_err_timeout());

		// Go read the busy bit
		SPIBUFINIT;
		MACHXO3_COMMAND = MACHXO3_CMD_CHECK_BUSY;
		MACHXO3_SPITRANS(5);
		busy = PTR_MACHXO3_DATA[0];
	} while(busy);

exit:
	if(!gRunQuiet)
		printf("done.\n");

	// Set flag indicating we has successfully erased the FPGA
	gFPGAIsErased = 1;

	// Return success
	return(0);
}

/* af_load_configuration loads the active configuration from flash as
   it would on power up.  This can be completed at any time, but is only
   useful if there is a good configuration in flash.  It is usually called
   after a configuration has been written so that the new configuration can
   take effect.  This is separate from the configuration programming itself
   so the user can decide when to do that.
*/

u32 af_load_configuration(void) {

	if(!gRunQuiet)
		printf("   Reloading configuration....");

	if(gDryRun) goto exit;

	// All exported library functions get this check of hardware and arguments
	if (!HW_IS_OPEN)
		return (af_err_not_open());

	// Send command to load the configuration
	// Remember there are fewer operands for unknown reasons
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_REFRESH;
	MACHXO3_SPITRANS(3);

exit:
	if(!gRunQuiet)
		printf("done.\n");

	// Wait for what is suppose to be a very fast configuration
	SLEEP(100);

	// Return success
	return (0);
}

 /* af_write_features writes 'feature row' and 'feabits' values to the
   flash as prescribed in the arguments.  This is typically used in a call
   from the programming routine, but it is possible that we would want to
   do that separately.  The caveat is that this does not check for erased
   condition, so overwriting an existing set is a potential user error.

   Also, this routine is for the Axon module, so the SPI must remain
   available, and therefore this routine forces that condition and won't
   let the caller change that setting. However, this appears to be a
   necessary but not sufficient condition, as the diamond software will
   also drop support for the pins in that case.

   Note that most of the action here seems to be in the feabits.  What is
   being called the feature row here, as far as we can tell, contains other
   information that may or may not apply to our chip or configuration,
   including various addresses and programmable codes.  So for it will
   usually be all 0.  The feabits contain the hardware control settings
   that concern us most.
 */

u32 af_write_features(u8 *FeatureRow, u8 *Feabits) {
	// All exported library functions get this check of hardware and arguments
	if(!HW_IS_OPEN) return(af_err_not_open());
	if(FeatureRow == NULL) return(af_err_null_ptr());
	if(Feabits == NULL) return(af_err_null_ptr());

	if(!gRunQuiet)
		printf("   Writing feature bits...");

	if(gDryRun)
		goto exit;

	// Command to write feature row
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_PROG_FEATURE;
	memcpy(PTR_MACHXO3_DATA, FeatureRow, 8);
	MACHXO3_SPITRANS(12);

	// Wait more than the time suggested as required to complete
	SLEEP(2);

	// Command to write feabits
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_PROG_FEABITS;
	memcpy(PTR_MACHXO3_DATA, Feabits, 2);
	// Prevent the SPI from being disabled - not sufficient to prevent a bricked device
	PTR_MACHXO3_DATA[1] &= 0xBF;
	// Send the command
	MACHXO3_SPITRANS(6);

exit:
	if(!gRunQuiet)
		printf("done.\n");

	// Wait more than the time suggested as required to complete
	SLEEP(2);

	// Return success
	return(0);
}

/* af_write_configuration writes a JEDEC file to the configuration
   flash and the feature switches.  This does not erase the chip or load
   the configuration.  The chip must be erased before entry using the above
   routine, and therefore will also be in programming mode.

   This routine accepts a filename as a full path string.  It is assumed that
   the file is a JEDEC file and is structured as required for the target chip.
   There is very little explicit error checking, although there is some
   error checking is undertaking by looking for specific features.  There is
   still no checking of checksums and other such features, mostly to keep this
   code as simple as possible. We also do make some effort to make this as
   ANSI-C portable as possible.

   Importantly, a JEDEC file has the '*' character as the field delimiter,
   so white space should not be used in code as a delimiter (CR, LF, etc),
   even if most files we are working with are structured in a way that seems
   to suggest that might work, and even might work in a specific version.
   We want to parse this strictly according to JEDEC, though we use the
   presentation of this information from the Lattice manual.  We do assume
   that the JEDEC file is not encrypted, we ignore any security fuse
   or OTP fuse settings, and we do not program the user code.  Finally,
   we assume the default fuse state is the erased state and do not program
   unspecified fuses to that nor use it to validate the checksum.

   This version now fully parses the JEDEC file to be sure the SPI slave port
   is not going to be disabled before allowing the programming to begin.
   We could look at header matter to determine the same thing in various ways,
   but there is no guarantee that the header matter won't change in other
   versions of diamond.  So we look at the feature row directly.  That does
   mean we could still be spoofed, but only by user intent.
*/

u32 af_write_configuration(char *JEDECFileName) {
	u8 featurerow[10];
	u8 feabits[4];
	int i, addr_digits, key, keyq;
	u8 status;

	if(!gRunQuiet)
		printf("Writing configuration...\n");

	// All exported library functions get this check of hardware and arguments
	// MessageBox(NULL, "af_write_configuration", "", MB_TASKMODAL);
	if(!HW_IS_OPEN) return(af_err_not_open());
	if(JEDECFileName == NULL) return(af_err_null_ptr());
	if(JEDECFileName[0] == 0) {
		printf("JEDEC file name not provided. Did you forget the -j option?\n");
		return(-1);
	}

	// If the FPGA has not been erased, indicate bad order and quit
	if(!gFPGAIsErased) return(af_err_not_erased());

	// If we are even about to configure the part, let's clear this flag here and
	// so indicate that we tried to program the part and should erase it again before
	// trying to program the part again.
	gFPGAIsErased = 0;

	// Attempt to open the JEDEC file
	// A JEDEC file is a text file - we will only need to read text,
	// and the end of line can be useful to us
	g_pJedecFile = fopen(JEDECFileName, "r");
	if(g_pJedecFile == NULL) return(af_err_file_not_found());

	if(!gRunQuiet)
		printf("JEDEC file: %s opened.\n", JEDECFileName );

	// Read the file characters until we find the starting STX (CTRL-B, 0x02)
	do { key = fgetc(g_pJedecFile); } while ((key != 0x02) && (key != EOF));
	if(key == EOF) EXIT_FILEFORMATERROR;

	// Look for key characters until we find Q and the qualifier 'F' for fuse count
	// We assume this will come before the fuse table as it is needed to read the address
	do {
		// Look for Q
		do { if((key = jedec_seek_next_key_char()) == 0) EXIT_FILEFORMATERROR; } while(key != 'Q');
		// Get qualifier
		keyq = fgetc(g_pJedecFile);
		if(keyq == EOF) return(0);
	} while (keyq != 'F');

	// We really don't care about the fuse count, unless we want to use it the help verify this is the right device
	// But we need to know how may characters it has so we can read addresses in the fuse table
	// Go get that count of characters
	addr_digits = 0;
	do {
		key = fgetc(g_pJedecFile);
		if(key == EOF) EXIT_FILEFORMATERROR;
		if(key != '*') addr_digits += 1;
	} while(key != '*');

	// We just read a delimiter, so we are at the start of the next field
	// Look for the key character - this step is probably overkill here, but for completeness
	if((key = jedec_seek_next_non_ws()) == 0) EXIT_FILEFORMATERROR;

	// Look for the key character 'E' for the feature row
	// Doing this early allows us to make sure the features do not include a disabled SPI port
	// We will do it again later to program these bits
	while(key != 'E') { if((key = jedec_seek_next_key_char()) == 0) EXIT_FILEFORMATERROR; };

	// We are now at the feature row bits, pointed at the fuse data
	// Read the data into the local arrays
	for(i=0;i<8;i++) if(jedec_read_fuse_byte(&featurerow[i]) != 1) EXIT_FILEFORMATERROR;

	for(i=0;i<2;i++) if(jedec_read_fuse_byte(&feabits[i]) != 1) EXIT_FILEFORMATERROR;

	// If the SPI port is disabled, warn the user and exit
	if(feabits[1] & 0x40) EXIT_FILEBADSETTINGERROR;

	// Restart the file to the beginning
	rewind(g_pJedecFile);
	// Read the file characters until we find the starting STX (CTRL-B, 0x02)
	do {
		key = fgetc(g_pJedecFile);
	} while ((key != 0x02) && (key != EOF));

	if(key == EOF) EXIT_FILEFORMATERROR;


	// Next look for the fuse table specifically
	while(key != 'L') {
		if((key = jedec_seek_next_key_char()) == 0)
			EXIT_FILEFORMATERROR;
	};

	// We are now at the fuse table and pointing to the starting address.
	// This address contains the same number of digits as the fuse count as previously found.
	// The documentation says it is followed by white space, but does not really guarantee what kind
	// of white space, so we must read off the address characters one at a time.
	// We do assume from the documentation that the first L key character found will contain the address zero,
	// and fuse data will start from the beginning of flash.  It is more work to translate and use the address,
	// so we don't want to bother if that is not the way it is suppose to be.  However, we can verify this address
	// is zero as expected and flag an error if it is something else.

	// Read the address and check that it is all zero
	// Note that any EOF would also drop out here
	for(i=0;i<addr_digits;i++)
	if(fgetc(g_pJedecFile) != '0') EXIT_FILEFORMATERROR;

	if(gDryRun)  // Skip writing the configuration
		goto exit;

	// The address is good and is zero, so clear the address in the device
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_INIT_ADDRESS;
	MACHXO3_SPITRANS(4);

	// We are now pointed at the fuse data (link table, whatever you want to call it)
	// Proceed to write flash locations per page until the delimiter is reached.
	// The table should have an even set of pages in it, so if we get the delimiter or any other failure
	// in the middle of a page, that is a format error.  Unfortunately, exiting here will have the chip
	// in a bizarre state of partially programmed, but that should be an exceptional case.
	do {
		// Setup the write and increment command
		SPIBUFINIT;
		MACHXO3_COMMAND = MACHXO3_CMD_PROG_INCR_NV;
		PTR_MACHXO3_OPERAND[2] = 0x01;
		// Get the first byte and check that we are not at the delimiter
		status = jedec_read_fuse_byte(PTR_MACHXO3_DATA);
		if(status == 0) EXIT_FILEFORMATERROR;
		// If we did not get the delimiter, this should be a valid row
		if(status != '*') {
			// Attempt to collect the rest of the page
			for(i=1;i<16;i++) {
				status = jedec_read_fuse_byte(&PTR_MACHXO3_DATA[i]);
				if(status != 1) EXIT_FILEFORMATERROR;
			};
			// We can now send the command
			MACHXO3_SPITRANS(20);
		};
		// The page is suppose to program in 200us, which is generally faster than we can send another command,
		// but for completeness and for the benefit of porting this code, we setup a wait here.
		// With a small chip this is not much of a burden, while other systems with a faster connection
		// and a larger chip my want to check busy instead.
		SLEEP(1);
		// We are done when the status here is the delimiter, otherwise go get the next row
	}
	while(status != '*');

	// Note that for our chip the JEDEC file seems to contain two blocks of data without easily located explanation.
	// The first is the configuration for the present design, and the second seems to be the remainder of the
	// configuration memory, as the address for the second block changes for each design.  There is no user flash
	// memory in this chip.  This remainder is always in a benign state, with another end flag of sorts at the
	// end (all 1's), which is never reached.  Considering all this, we ignore this second block of data.

	// Go find the key for our next thing of interest, the feature row fuses
	// We just read a delimiter, so we are at the start of the next field
	if((key = jedec_seek_next_non_ws()) == 0) EXIT_FILEFORMATERROR;
	// Look for the key character
	while(key != 'E') { if((key = jedec_seek_next_key_char()) == 0) EXIT_FILEFORMATERROR; };

	// We are now at the feature row bits, pointed at the fuse data
	// Read the data into the local arrays
	for(i=0;i<8;i++) if(jedec_read_fuse_byte(&featurerow[i]) != 1) EXIT_FILEFORMATERROR;
	for(i=0;i<2;i++) if(jedec_read_fuse_byte(&feabits[i]) != 1) EXIT_FILEFORMATERROR;
	// Call our routine to program these values
	// Note that this routine may alter some bits (see comments with routine)
	af_write_features(featurerow, feabits);

	// Program the DONE bit (internal)
	// This effectively tells the SDM (self download mode) that it is allowed to run
	// and allows the device to enter user mode when loading is complete (ie done)
	SPIBUFINIT;
	MACHXO3_COMMAND = MACHXO3_CMD_PROGRAM_DONE;
	MACHXO3_SPITRANS(4);
	SLEEP(1);

	// Security and OTP bits would be programmed here, but we do not support them
	// They seem to operate similar to DONE, enabling or disabling certain features
	// DONE is all we need - enables user mode
	// Advanced user may also user an external 'DONE' pin to control entry into user mode,
	// but that will require additional study of feature settings and coding

exit:
	if(!gRunQuiet)
		printf("Configuration written.\n");

	// Now that everything is programmed, reload the configuration from flash
	af_load_configuration();

	// Close file
	fclose(g_pJedecFile);

	// If we got here, all went ok, return success
	return(1);
}

/*--------------------------------------------------------------------------*/
/* JEDEC File Parsing Support Subroutines                                   */
/*--------------------------------------------------------------------------*/

/* The following private functions all operate on a JEDEC file using a
   file stream pointer for an open text file established globally by
   the caller.  Coding this way removes the need to pass the file pointer
   to each function and among them.  Because these are private functions
   and used in a manner controlled in this module, we do not do additional
   checking of the pointers and other variables in use.
 */

/* jedec_seek_next_non_ws() parses the file until it finds a character
   that is not white space as defined for a JEDEC file.  That character is
   returned if found.  If there is an error, a 0 is returned.

   Note that the delimiter ('*') is also white space in this context.  This is
   like a line terminator in a sense, so if we have not already read it, we
   don't want to now read it and return it.  This could happen if it is
   the first character in a file, or if a field ended with "**" or more.
   These technically null fields should be ignored in this search.

   So define white space as ' ', CR, LF, NULL, and '*' at least.  But in
   reality we equally ignore any character less than space, which includes
   most control characters, including the JEDEC file start and end STX/EOT.
*/

char jedec_seek_next_non_ws(void) {
	int ic;

	// Read until we find something other than white space
	do {
		ic = fgetc(g_pJedecFile);
		if(ic == EOF) return(0);
	}
	while((ic <= ' ') || (ic == '*'));

	// Return what we found
	return(ic);
}

/* jedec_seek_next_key_char() reads the specified file stream until the next key
   character has been read. The key character (ie key word) is the first
   character of a field (ie after the previous field's delimiter) after any
   white space.  Thus, in order to do this search, this function will also
   search for the start of the next field.  If we know we are at the start
   of a field already, then we should not use this routine but instead just
   look for the character.  This routine will find the next key character.

   The key character found is returned by value.  If the end of the file
   is reached, or some other error occurs, a 0 is returned.
*/

char jedec_seek_next_key_char(void) {
	int ic;
	char key;

	// Look for end of field, point to start of next field
	do {
		ic = fgetc(g_pJedecFile);
		if(ic == EOF) return(0);
	}
	while (ic != '*');

	// Pull white space until an actual character is reached
	// This will be our key character
	key = jedec_seek_next_non_ws();

	// Return the value that got us here
	return(key);
}

/* jedec_read_fuse_byte() reads from the specified file stream until it has
   collected eight binary characters and converts those characters to a byte
   to return by reference.  The value returned is 1 if this happened correctly,
   0 if an error was encountered, and '*' if the field has ended ('*' found).
   Generally, unless something is wrong with the file, the full byte will be
   collected or '*' will be returned.  Other white space is automatically removed.
   A return value of 0 can be interpreted as a format error.  A return value
   of '*' means the caller should assume the file pointer now points to the
   start of the next field.

   Note that anything other than 1,0,* characters can be considered white space.
   If a character is out of place or replaced, and it disrupts the count of 1s
   and 0s, then an error will eventually be found.  If the bad character is there
   without affecting the result, then it has no impact this way.  In a perfect
   file, the characters so removed are truly white space.
*/

u8 jedec_read_fuse_byte(u8 *FuseByte) {
	char bstr[10];
	int ic;
	u8 cnt = 0;

	// Default byte value
	*FuseByte = 0;

	// Read characters
	do {
		// Get a character
		ic = fgetc(g_pJedecFile);
		if(ic == EOF) return(0);
		// Record valid characters
		if(ic == '0') bstr[cnt++] = '0';
		if(ic == '1') bstr[cnt++] = '1';
		// If delimiter found, return it
		if(ic == '*') return('*');
	}
	while(cnt < 8);

	// Convert the characters to binary
	// We could make this a loop, but it would just take more variables and time
	// This also allows us to easily adjust the bit order if it ever needs to be different
	if(bstr[0] == '1') *FuseByte += 128;
	if(bstr[1] == '1') *FuseByte += 64;
	if(bstr[2] == '1') *FuseByte += 32;
	if(bstr[3] == '1') *FuseByte += 16;
	if(bstr[4] == '1') *FuseByte += 8;
	if(bstr[5] == '1') *FuseByte += 4;
	if(bstr[6] == '1') *FuseByte += 2;
	if(bstr[7] == '1') *FuseByte += 1;

	// Return normal success
	return(1);
}

/*--------------------------------------------------------------------------*/
/* General Purpose And Helper Subroutines                                   */
/*--------------------------------------------------------------------------*/

/* af_spi_transaction() completes the data transfer to and/or from the device
   per the methods required by this system.  This uses the global defined
   SPI port handle, which is assumed to be open if this call is reached
   from a routine in this code.  It is also assumed that the arguments are
   valid based on the controlled nature of calls to this routine.
*/

u32 af_spi_transaction(u8 Count, void *Data) {
	int ret;
	// Rx and Tx as the same byte buffer
	u8 *pbRx = Data;
	u8 *pbTx = Data;
	// Structure of transfer
	struct spi_ioc_transfer msg = {
		.tx_buf = (unsigned long)pbTx,
		.rx_buf = (unsigned long)pbRx,
		.len = Count,
		.delay_usecs = 1,
		.speed_hz = 400000,
		.bits_per_word = 8,
	};

	// Complete the transfer
	ret = ioctl(gSPIPort, SPI_IOC_MESSAGE(1), &msg);

	// Return result
	return(ret);
}

/* af_error_message() notifies the user of an error specified by a text string.
   A -1 is returned for convenience.
*/

int af_error_message(char *ErrorDescription, char *ErrorType) {
	printf("%s -> %s\n\r", ErrorType, ErrorDescription);
	return(-1);
}

