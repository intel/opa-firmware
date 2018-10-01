/*
 * Copyright(c) 2015-2018 Intel Corporation.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define _GNU_SOURCE
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "config.h"

#define NO_4KB_BLOCK_ERASE_WA 1


#define MAX_PLATFORM_CONFIG_SIZE            2048
#define PLATFORM_CONFIG_FORMAT_4_FILE_SIZE  528

/*
 * Dividing points between the partitions.
 */
#define P0_SIZE (128 * 1024)
#define P1_SIZE (  4 * 1024)
#define P1_START P0_SIZE
#define P2_START (P0_SIZE + P1_SIZE)

/* erase sizes supported by the controller */
#define SIZE_4KB (4 * 1024)
#define MASK_4KB (SIZE_4KB - 1)

#define SIZE_32KB (32 * 1024)
#define MASK_32KB (SIZE_32KB - 1)

#define SIZE_64KB (64 * 1024)
#define MASK_64KB (SIZE_64KB - 1)

#define SIZE_1MB (1024 * 1024)

#define B_TO_KB(BYTES) ((BYTES) / 1024)

/* "page" size for the EPROM, in bytes */
#define EP_PAGE_SIZE 256

/* EPROM WP_N line in GPIO signals */
#define EPROM_WP_N (1 << 14)

/*
 * ASIC block register offsets.
 */
#define CORE		    0x000000000000
#define ASIC		    (CORE + 0x000000400000)
#define ASIC_GPIO_OE	    (ASIC + 0x000000000208)
#define ASIC_GPIO_OUT	    (ASIC + 0x000000000218)
#define ASIC_EEP_CTL_STAT   (ASIC + 0x000000000300)
#define ASIC_EEP_ADDR_CMD   (ASIC + 0x000000000308)
#define ASIC_EEP_DATA	    (ASIC + 0x000000000310)
#define MAP_SIZE	    (ASIC + 0x000000000318)
void *reg_mem = NULL;

/*
 * Commands
 */
#define CMD_SHIFT 24
#define CMD_NOP			    (0)
#define CMD_PAGE_PROGRAM(addr)	    ((0x02 << CMD_SHIFT) | addr)
#define CMD_READ_DATA(addr)	    ((0x03 << CMD_SHIFT) | addr)
#define CMD_READ_SR1		    ((0x05 << CMD_SHIFT))
#define CMD_WRITE_ENABLE	    ((0x06 << CMD_SHIFT))
#define CMD_SECTOR_ERASE_4KB(addr)  ((0x20 << CMD_SHIFT) | addr)
#define CMD_SECTOR_ERASE_32KB(addr) ((0x52 << CMD_SHIFT) | addr)
#define CMD_CHIP_ERASE		    ((0x60 << CMD_SHIFT))
#define CMD_READ_JEDEC_ID	    ((0x9f << CMD_SHIFT))
#define CMD_RELEASE_POWERDOWN_NOID  ((0xab << CMD_SHIFT))
#define CMD_SECTOR_ERASE_64KB(addr) ((0xd8 << CMD_SHIFT) | addr)


/*
 * Magic bits appended to the end of an image to allow calculation
 * of an exact image size as required by UEFI Secure Boot.
 * Bits used equate in ASCII to 'OPAimage'
 */
#define IMAGE_MAGIC_LEN 8
#define IMAGE_MAGIC_VAL 0x4f5041696d616765

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
const char pci_eprom_device[] = "0x24f0";

struct file_info {
	char *name;	        /* name for the file */
	size_t fsize;		/* file size, in bytes */
	size_t bsize;		/* buffer size, in bytes */
	void *buffer;		/* memory allocated buffer */
	uint32_t start;		/* starting point in the EPROM */
	int fd;			/* file descriptor for the file */
	int part;		/* partition number */
};

/* operations */
#define DO_NOTHING 0
#define DO_READ    1
#define DO_WRITE   2
#define DO_ERASE   3
#define DO_INFO	   4
#define DO_VERSION 5
#define DO_UPDATE  6
#define DO_EFI_VERSION 7

/* Partitions */
#define PART_NONE   -2
#define PART_ALL    -1
#define PART_OPROM   0
#define PART_CONFIG  1
#define PART_BULK    2

/* Status Register 1 bits */
#define SR1_BUSY 0x1	/* the BUSY bit in SR1 */

/* verbose details */
#define WAIT_SLEEP_US 100
#define COUNT_DELAY_SEC(n) ((n) * (1000000/WAIT_SLEEP_US))

/* platform config header structure fields */
#define PLATFORM_CONFIG_HEADER_RECORD_IDX_SHIFT		0
#define PLATFORM_CONFIG_HEADER_RECORD_IDX_LEN_BITS	6
#define PLATFORM_CONFIG_HEADER_TABLE_LENGTH_SHIFT	16
#define PLATFORM_CONFIG_HEADER_TABLE_LENGTH_LEN_BITS	12
#define PLATFORM_CONFIG_HEADER_TABLE_TYPE_SHIFT		28
#define PLATFORM_CONFIG_HEADER_TABLE_TYPE_LEN_BITS	4

/* platform config meta data fields */
#define METADATA_TABLE_FIELD_START_SHIFT		0
#define METADATA_TABLE_FIELD_START_LEN_BITS		15
#define METADATA_TABLE_FIELD_LEN_SHIFT			16
#define METADATA_TABLE_FIELD_LEN_LEN_BITS		16

/* helper for creating a mask from a bit count */
#define mask_of(bits) ((1 << (bits)) - 1)

enum platform_config_table_type_encoding {
	PLATFORM_CONFIG_TABLE_RESERVED,
	PLATFORM_CONFIG_SYSTEM_TABLE,
	PLATFORM_CONFIG_PORT_TABLE,
	PLATFORM_CONFIG_RX_PRESET_TABLE,
	PLATFORM_CONFIG_TX_PRESET_TABLE,
	PLATFORM_CONFIG_QSFP_ATTEN_TABLE,
	PLATFORM_CONFIG_VARIABLE_SETTINGS_TABLE,
	PLATFORM_CONFIG_TABLE_MAX
};

enum platform_config_system_table_fields {
	SYSTEM_TABLE_RESERVED,
	SYSTEM_TABLE_NODE_STRING,
	SYSTEM_TABLE_SYSTEM_IMAGE_GUID,
	SYSTEM_TABLE_NODE_GUID,
	SYSTEM_TABLE_REVISION,
	SYSTEM_TABLE_VENDOR_OUI,
	SYSTEM_TABLE_META_VERSION,
	SYSTEM_TABLE_DEVICE_ID,
	SYSTEM_TABLE_PARTITION_ENFORCEMENT_CAP,
	SYSTEM_TABLE_QSFP_POWER_CLASS_MAX,
	SYSTEM_TABLE_QSFP_ATTENUATION_DEFAULT_12G,
	SYSTEM_TABLE_QSFP_ATTENUATION_DEFAULT_25G,
	SYSTEM_TABLE_VARIABLE_TABLE_ENTRIES_PER_PORT,
	SYSTEM_TABLE_MAX
};

#define ROM_IMAGE_SIGNATURE       0xAA55
#define EFI_IMAGE_SIGNATURE       0x5A4D

/* platform config file magic number (4 bytes) */
#define PLATFORM_CONFIG_MAGIC_NUM 0x3d4f5041

/* string in front of the verison string for oprom and driver files */
const char version_magic[] = "VersionString:";

/* buffer size for platform config version info */
#define VBUF_MAX 128
#define MAX_DEV_ENTRIES 128

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

/*
 * PCI bus address if formatted as:
 * domain:bus:slot.function (dddd:bb:ss.f)
 * 13 chars required, rounder up to power of 2
 */
#define MAX_PCI_BUS_LEN 16
#define MAX_DEV_NAME 256
#define PCI_SHORT_ADDR(PCI_ADDR) (&(PCI_ADDR)[5])

const char pci_device_path[] = "/sys/bus/pci/devices";
int num_dev_entries = 0;
char pci_device_addrs[MAX_DEV_ENTRIES][MAX_PCI_BUS_LEN];
const char resource_fmt[] = "%s/%s/resource0";
char resource_file[sizeof(pci_device_path) + sizeof(resource_fmt) + MAX_PCI_BUS_LEN] = "";
const char enable_fmt[] = "%s/%s/enable";
char enable_file[sizeof(pci_device_path) + sizeof(enable_fmt) + MAX_PCI_BUS_LEN] = "";
const char device_fmt[] = "%s/%s/device";
char device_file[sizeof(pci_device_path) + sizeof(device_fmt) + MAX_DEV_NAME] = "";
const char *command;		/* derived command name */
uint32_t dev_id;		/* EEPROM device identification */
uint32_t dev_mbits;		/* device megabit size */
int verbose;
bool silence_warnings = false;
bool print_meta = false;
bool service_mode = false;

const struct size_info {
	uint32_t dev_id;		/* device */
	uint32_t megabits;		/* size in megabits */
} device_sizes[] = {
	/* JEDEC id, mbits */
	{ 0x001560ef,    16 },          /* Winbond W25Q16D{V,W} */
	{ 0x001660ef,    32 },          /* Winbond W25Q32D{V,W} */
	{ 0x001760ef,    64 },          /* Winbond W25Q64D{V,W} */
	{ 0x001860ef,   128 },          /* Winbond W25Q128FW */
	{ 0x0017BB20,    64 },          /* Micron MT25QU128ABA */
	{ 0x0018BB20,   128 },          /* Micron MT25QU128AB{A}*/
	{ 0x00182001,   128 },          /* Spansion S25FS128 */
};

struct
{
	bool enabled;
	int state;
	int msg_size;
	struct timespec last_time;
} progress_spin;

/*
 * Init progress spin. Setup time variable to determine
 * estimated time and print operation message
 */
void progress_spin_init(const char* format, ...)
{
	va_list args;
	char msg[512];

	if(!verbose) {
		progress_spin.enabled = true;
		progress_spin.state = 0;

		va_start (args, format);
		vsnprintf(msg, ARRAY_SIZE(msg), format, args);
		va_end (args);
		progress_spin.msg_size = strlen(msg);

		clock_gettime(CLOCK_MONOTONIC, &progress_spin.last_time);

		printf("%s ", msg);
	}
}

/*
 * Backspace last character and print next from the sequence
 * -\|/-\|/ imitating spin if done frequently enough. There is
 * time limit between consequtive character changes set to 0.2 sec
 * to avoid spin being to rapid.
 */
void progress_spin_advance(void)
{
	static const char chars[] = "-\\|/-\\|/";

	if(!verbose && progress_spin.enabled) {
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		time_t sec_diff = now.tv_sec - progress_spin.last_time.tv_sec;
		long int nsec_diff = now.tv_nsec - progress_spin.last_time.tv_nsec;
		if(sec_diff > 0 || nsec_diff > 200000000l) {
			printf("\b%c", chars[progress_spin.state]);
			fflush(stdout);
			progress_spin.state = (progress_spin.state + 1) % (ARRAY_SIZE(chars) - 1);
			clock_gettime(CLOCK_MONOTONIC, &progress_spin.last_time);
		}
	}
}

/*
 * End progress spin. Backspace the spin character
 * and print 'done' instead
 */
void progress_spin_fini(void)
{
	if(!verbose) {
		progress_spin.enabled = false;
		printf("\bdone\n");
	}
}

/*
 * End progress spin. Backspace the spin character
 * and accompanying message
 */
void progress_spin_clear(void)
{
	if(!verbose) {
		progress_spin.enabled = false;
		int i = 0;
		for(; i < progress_spin.msg_size + 1; ++i)
			printf("\b");
	}
}

const char *file_name(int part);

/* ========================================================================== */

uint32_t __attribute__ ((noinline)) read_reg(int fd, uint32_t csr)
{
	volatile uint64_t reg;

	if (csr >= MAP_SIZE ) {
		fprintf(stderr, "Unable to read from %x, out of range: max %x\n", csr, MAP_SIZE);
		exit(1);
	}

	reg = *(uint64_t *)(reg_mem + csr);

	return reg;
}

void __attribute__ ((noinline)) write_reg(int fd, uint32_t csr, volatile uint32_t val)
{
	if (csr >= MAP_SIZE) {
		fprintf(stderr, "Unable to write to %x, out of range: max %x\n", csr, MAP_SIZE);
		exit(1);
	}

	*(uint64_t *)((char *)reg_mem + csr) = val;
}

int is_all_1s(const uint8_t *buffer, int size)
{
	while (size-- > 0)
		if (*buffer++ != 0xff)
			return 0;
	return 1;
}

int is_all_0s(const uint8_t *buffer, int size)
{
	while (size-- > 0)
		if (*buffer++ != 0)
			return 0;
	return 1;
}

/*
 * Search through an image represented by buffer for a series
 * of magic bits, marking the end of that image. If those magic
 * bits are not found, return the full size of the buffer.
 */
uint32_t get_image_size(void *buffer, uint32_t bsize)
{
	uint32_t fsize;
	uint32_t tmp_bsize = 0;
	void *prev_p = NULL;
	void *current_p;
	union {
		uint64_t val;
		uint8_t bytes[IMAGE_MAGIC_LEN];
	} u;

	u.val = htole64(IMAGE_MAGIC_VAL);

	current_p = memmem(buffer, bsize, u.bytes, IMAGE_MAGIC_LEN);
	while (current_p) {
		tmp_bsize = (current_p - buffer);

		prev_p = current_p;
		/* Removing Magic bit string from buffer */
		current_p = current_p + IMAGE_MAGIC_LEN;
		tmp_bsize = tmp_bsize - IMAGE_MAGIC_LEN;
		current_p = memmem(current_p, bsize - tmp_bsize,
				   u.bytes, IMAGE_MAGIC_LEN);
	}

	if (prev_p)
		fsize = prev_p - buffer;
	else
		fsize = bsize;

	return fsize;
}

/*
 * Copy in the magic bits so we can determine the length of the driver
 * on read. Only copy the magic bits if there is actually space to
 * do so. Otherwise, leave the image as-is.
 */
void add_magic_bits_to_image(uint32_t fsize, uint32_t bsize, void *buffer)
{
	/* Make sure we don't try to write past the partition boundary */
	if (fsize + IMAGE_MAGIC_LEN <= bsize) {
		uint64_t magic_val = IMAGE_MAGIC_VAL;
		memcpy(&((uint8_t *)buffer)[fsize], &magic_val, IMAGE_MAGIC_LEN);
	}
}

/* Return the chip size */
uint32_t find_chip_bit_size(uint32_t dev_id)
{
	int i;

	/* No EEPROM chip detected, return 0 size */
	if (dev_id == 0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(device_sizes); i++) {
		if (device_sizes[i].dev_id == dev_id)
			return device_sizes[i].megabits;
	}

	/* No match, assume small 16 Mbit chip*/
	return 16;
}

/* Return JEDEC vendor id */
int find_chip_vendor(uint32_t dev_id)
{
	return dev_id & 0xff;
}

/* wait for the device to become not busy */
void wait_for_not_busy(int fd)
{
	unsigned long count = 0;
	uint32_t reg;

	write_reg(fd, ASIC_EEP_ADDR_CMD, CMD_READ_SR1); /* starts page mode */
	while (1) {
		progress_spin_advance();

		usleep(WAIT_SLEEP_US);
		count++;
		reg = read_reg(fd, ASIC_EEP_DATA);
		if ((reg & SR1_BUSY) == 0)
			break;
		/* 200s is the largest time for a 128Mb device */
		if (count > COUNT_DELAY_SEC(200)) {
			fprintf(stderr, "Waited too long for busy to clear - failing\n");
			exit(1);
		}
	}
	if (verbose > 3)
		printf("Wait not busy count: %lu (%d us per count)\n", count,
			WAIT_SLEEP_US);
	/* stop page mode with another NOP */
	write_reg(fd, ASIC_EEP_ADDR_CMD, CMD_NOP);
}

void erase_chip(int dev_fd)
{
	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_WRITE_ENABLE);
	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_CHIP_ERASE);

	wait_for_not_busy(dev_fd);
}

#if NO_4KB_BLOCK_ERASE_WA

void do_write(int dev_fd, struct file_info *fi);
void do_read(int dev_fd, struct file_info *fi);
void prepare_file(int op, int partition, const char *fname,
		  struct file_info *fi);
void clean_file(struct file_info *fi);

/*
 * Alternative erase range routine that does not use block erase commands.
 * Workaround for chips that cannot erase 4KB pages at arbitrary offset.
 * It reads back whole chip into buffer, clears given range in the buffer,
 * erases whole chip and writes back modified buffer to chip.
 * This may be very slow, i.e. HfiPcieGen3Loader.rom update can take
 * 75 seconds on 128Mbit chip.
 */
void erase_range_slow(int dev_fd, uint32_t start, uint32_t len)
{
	struct file_info tmp_fi;
	prepare_file(DO_VERSION, PART_ALL, "chip", &tmp_fi);

	do_read(dev_fd, &tmp_fi);
	if (is_all_1s(tmp_fi.buffer + start, len)) {
		/* partition to erase is empty, we are done */
		return;
	}

	if (verbose)
		printf("Erasing whole chip\n");
	erase_chip(dev_fd);

	memset(tmp_fi.buffer + start, 0xff, len);
	do_write(dev_fd, &tmp_fi);
	clean_file(&tmp_fi);
}
#endif

void erase_range(int dev_fd, uint32_t start, uint32_t len)
{
	uint32_t end = start + len;

#if NO_4KB_BLOCK_ERASE_WA
	if (find_chip_vendor(dev_id) == 0x01) {
		erase_range_slow(dev_fd, start, len);
		return;
	}
#endif
	if (verbose)
		printf("...erasing range 0x%08x-0x%08x\n", start, end);
	if ((start & MASK_4KB) || (end & MASK_4KB)) {
		fprintf(stderr, "Non-algined range (0x%x,0x%x) for a 4KB erase\n",
			start, end);
		exit(1);
	}

	while (start < end) {
		write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_WRITE_ENABLE);
		if (((start & MASK_64KB) == 0 && (start + SIZE_64KB) <= end)) {
			if (verbose > 1)
				printf("Erase 64KB %x\n", start);
			write_reg(dev_fd, ASIC_EEP_ADDR_CMD,
				  CMD_SECTOR_ERASE_64KB(start));
			start += SIZE_64KB;
		} else if (((start & MASK_32KB) == 0
			    && (start + SIZE_32KB) <= end)) {
			if (verbose > 1)
				printf("Erase 32KB %x\n", start);
			write_reg(dev_fd, ASIC_EEP_ADDR_CMD,
				  CMD_SECTOR_ERASE_32KB(start));
			start += SIZE_32KB;
		} else {	/* 4K */
			if (verbose > 1)
				printf("Erase 4KB %x\n", start);
			write_reg(dev_fd, ASIC_EEP_ADDR_CMD,
				  CMD_SECTOR_ERASE_4KB(start));
			start += SIZE_4KB;
		}
		wait_for_not_busy(dev_fd);
	}

	/* the wait_for_not_busy will clear page mode */
}

uint32_t read_device_id(int fd)
{
	/* Read the Manufacture Device ID */
	write_reg(fd, ASIC_EEP_ADDR_CMD, CMD_READ_JEDEC_ID);
	return read_reg(fd, ASIC_EEP_DATA);
}

/* reads a 256 byte (64 dword) page, placing it in result */
void read_page(int dev_fd, uint32_t offset, uint32_t *result)
{
	int i;

	if ((offset % EP_PAGE_SIZE) != 0) {
		fprintf(stderr, "%s: invalid address 0x%x", __func__, offset);
		exit(1);
	}

	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_READ_DATA(offset));
	for (i = 0; i < EP_PAGE_SIZE/sizeof(uint32_t); i++)
		result[i] = read_reg(dev_fd, ASIC_EEP_DATA);
	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_NOP);

	if (verbose > 2)  {
		if (is_all_1s((uint8_t *)result, EP_PAGE_SIZE)) {
			if (verbose > 3)
				printf("%08x: all 1ns\n", offset);
		} else {
			printf("%08x: %08x %08x %08x %08x %08x %08x ...\n",
			       offset, result[0], result[1], result[2],
			       result[3], result[4], result[5]);
		}
	}
}

/* writes a 256 byte (64 dword) page */
void write_page(int dev_fd, uint32_t offset, uint32_t *data)
{
	int i;

	if ((offset % EP_PAGE_SIZE) != 0) {
		fprintf(stderr, "%s: invalid address 0x%x", __func__, offset);
		exit(1);
	}
	/*
	 * No need to write data which is all 1ns - we can only write over
	 * erased (all 1ns) sectors so  they are already all 1ns.
	 * For 128Mbit EEPROMs it can speed up driver partition update by
	 * a factor of 10 (from ~36 to ~3.5 seconds)
	 */
	if (is_all_1s((uint8_t *)data, EP_PAGE_SIZE))
		return;

	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_WRITE_ENABLE);
	write_reg(dev_fd, ASIC_EEP_DATA, data[0]);
	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_PAGE_PROGRAM(offset));
	for (i = 1; i < EP_PAGE_SIZE/sizeof(uint32_t); i++)
		write_reg(dev_fd, ASIC_EEP_DATA, data[i]);
	/* will close the open page */
	wait_for_not_busy(dev_fd);
}

void init_eep_interface(int dev_fd)
{
	/* reset on (a level variable) */
	write_reg(dev_fd, ASIC_EEP_CTL_STAT, 0x4);
	/* reset off and set speed */
	/*    RATE_SPI = 0x2		// 0x2 << 8 */
	write_reg(dev_fd, ASIC_EEP_CTL_STAT, 0x200);
	/* wake the device with command "release powerdown NoID" */
	write_reg(dev_fd, ASIC_EEP_ADDR_CMD, CMD_RELEASE_POWERDOWN_NOID);
}

void write_system_file(struct file_info *fi)
{
	ssize_t nwritten;

	/* write buffer to system */
	nwritten = write(fi->fd, fi->buffer, fi->fsize);
	if (nwritten < 0) {
		fprintf(stderr, "Partition %d file \"%s\" write error: %s\n",
				fi->part, fi->name, strerror(errno));
		exit(1);
	}
	if (nwritten != fi->fsize) {
		fprintf(stderr, "Partition %d file \"%s\" only wrote 0x%lx of 0x%lx bytes\n",
			fi->part, fi->name, nwritten, fi->fsize);
		exit(1);
	}
}

/*
 * Find the given string in the buffer.  The buffer may not be all text.
 * Return a pointer to the next character after the string or NULL if the
 * string is not present.
 */
void *find_string_in_buffer(const uint8_t *buffer, int size, const char *str)
{
	void *p;
	int offset = 0;
	int str_len = strlen(str);

	while (size - offset > 0) {
		/* look for first letter */
		p = memchr(&buffer[offset], str[0], size - offset);
		/* not found */
		if (!p)
			break;
		/* update offset */
		offset = p - (void *)buffer;

		/* if not enough room for string, done */
		if (size - offset < str_len)
			break;

		/* is this the string? */
		if (strncmp(p, str, str_len) == 0)
			return p + str_len; /* found */

		/* move to next character */
		offset++;
	}

	return NULL; /* not found */
}

static char *print_meta_ver(const uint32_t *sys_table_data,
			    uint32_t meta_ver_field_def,
			    char *str, size_t size)
{
	uint32_t mask;
	int offset = 0;
	int len = 0;

	if (meta_ver_field_def == 0) {
		snprintf(str, size, "[meta : <unknown>]");
		goto done;
	}

	mask = mask_of(METADATA_TABLE_FIELD_START_LEN_BITS);
	offset = meta_ver_field_def & mask;

	meta_ver_field_def >>= METADATA_TABLE_FIELD_LEN_SHIFT;
	mask = mask_of(METADATA_TABLE_FIELD_LEN_LEN_BITS);
	len = meta_ver_field_def & mask;

	offset /= 8;	/* change to bytes */

	meta_ver_field_def = *((uint8_t *)sys_table_data + offset) & mask_of(len);
	snprintf(str, size, "[meta : 0x%0*X]", len/4, meta_ver_field_def);

done:
	return str;
}

/*
 * Retrieve version string from rom of efi driver image
 */
int get_version_string(uint8_t *buf, int size, char *out_buf,
		       int out_size)
{
	uint8_t *vers;
	int offset;
	int i;
	uint8_t c;
	int found = 0;


	/* look for version magic in the data */
	vers = find_string_in_buffer((void*)buf, size, version_magic);
	if (vers) {
		found = 1;
		/*
		 * Expect the version string to immediately
		 * follow the version magic in a C string.
		 * Copy it out, but keep checking for buffer
		 * end to be on the safe side.
		 */
		offset = (int)(vers - buf);
		for (i = 0; i < out_size && i+offset < size; i++) {
			c = buf[i+offset];
			if (c == 0)
				break;
			out_buf[i] = c;
		}
	}
	return found;
}

/*
 * Look through the platform config file in buffer and extract the
 * NODE_STRING, placing it in out_buf.
 * Return 1 on success, 0 on failure.
 */
int parse_platform_config(const void *buffer, int size, char *out_buf,
				int out_size)
{
	const uint32_t *ptr = buffer;
	const uint32_t *end;
	const uint32_t *sys_table_data = NULL;
	const char *vers;
	uint32_t ns_metadata = 0;	/* node string meta data */
	uint32_t mask;
	uint32_t temp;
	uint32_t file_length;
	uint32_t header1, header2;
	uint32_t record_idx, table_length_dwords, table_type;
	int found_metadata = 0;
	int ns_start = 0;
	int ns_len = 0;
	uint32_t meta_ver_field_def = 0;

	/* read magic */
	temp = *ptr;
	if (temp != PLATFORM_CONFIG_MAGIC_NUM) {
		printf("%s: invalid magic\n", __func__);
		return 0; /* fail */
	}
	ptr++;

	/* read file length */
	file_length = (*ptr) * 4;	/* *4 to convert to bytes */
	if (verbose > 1) {
		printf("%s: file length %d (buffer size %d)\n",
			__func__, file_length, size);
	}

	if (file_length > MAX_PLATFORM_CONFIG_SIZE)
		file_length = PLATFORM_CONFIG_FORMAT_4_FILE_SIZE;
	else
		ptr++;

	/* this is the valid bounds within the file */
	end = (uint32_t *)(buffer + file_length);

	/* fully walk the file as a sanity check */
	while (ptr < end) {
		if (ptr + 2 > end) {
			printf("%s: not enough room for header\n", __func__);
			return 0; /* fail */
		}
		header1 = *ptr;
		header2 = *(ptr + 1);
		if (header1 != ~header2) {
			printf("%s: header validation failed, "
				"h1 0x%x, h2 0x%x\n",
				__func__, header1, header2);
			return 0; /* fail */
		}
		ptr += 2;

		record_idx = header1 &
			mask_of(PLATFORM_CONFIG_HEADER_RECORD_IDX_LEN_BITS);

		table_length_dwords = (header1 >>
				PLATFORM_CONFIG_HEADER_TABLE_LENGTH_SHIFT) &
		      mask_of(PLATFORM_CONFIG_HEADER_TABLE_LENGTH_LEN_BITS);

		table_type = (header1 >>
				PLATFORM_CONFIG_HEADER_TABLE_TYPE_SHIFT) &
			mask_of(PLATFORM_CONFIG_HEADER_TABLE_TYPE_LEN_BITS);
		if (verbose > 1)
			printf("%s: header: record %d, length %2d, type %d\n",
				__func__, record_idx, table_length_dwords,
				table_type);

		/* NODE_STRING is in the system table, gather data */
		if (table_type == PLATFORM_CONFIG_SYSTEM_TABLE) {
			if (record_idx) { /* the data */
				sys_table_data = ptr;
			} else { /* the metadata */
				ns_metadata = *(ptr + SYSTEM_TABLE_NODE_STRING);
				found_metadata = 1;
				meta_ver_field_def = *(ptr + SYSTEM_TABLE_META_VERSION);
			}
		}

		ptr += table_length_dwords;
		ptr++; /* jump the CRC dword */

		/* something is wrong if we are past the end */
		if (ptr > end) {
			printf("%s: position: oops %ld bytes over\n",
				__func__, ptr - end);
			return 0; /* fail */
		}
	}

	/* we have walked through the file and grabbed the parts we want */

	/* make sure the parts were found */
	if (!sys_table_data) {
		printf("%s: system table data not found\n", __func__);
		return 0; /* fail */
	}
	if (!found_metadata) {
		printf("%s: system table metadata not found\n", __func__);
		return 0; /* fail */
	}

	/*
	 * Extract the offset and size from the meta data.  The values
	 * are in bits.
	 */
	mask = mask_of(METADATA_TABLE_FIELD_START_LEN_BITS);
	ns_start = ns_metadata & mask;

	ns_metadata >>= METADATA_TABLE_FIELD_LEN_SHIFT;
	mask = mask_of(METADATA_TABLE_FIELD_LEN_LEN_BITS);
	ns_len = ns_metadata & mask;

	ns_start /= 8;	/* change to bytes */
	ns_len /= 8;	/* change to bytes */

	if (verbose > 1)
		printf("%s: node string start %d, len %d\n", __func__,
			ns_start, ns_len);

	/* make sure we have a large enough buffer */
	if (ns_len > out_size) {
		printf("%s: node string len %d too long, truncating\n",
			__func__, ns_len);
		ns_len = out_size;
	}

	/* copy into alreay-zeroed buffer, with an extra nul */
	vers = ((const char *)sys_table_data) + ns_start;
	memcpy(out_buf, vers, ns_len);

	if (print_meta) {
		ns_len = strlen(out_buf);
		out_buf[ns_len] = ' ';
		ns_len++;
		out_buf += ns_len;
		out_size -= ns_len;
		print_meta_ver(sys_table_data, meta_ver_field_def,
			       out_buf, out_size);
	}

	return 1; /* success */
}

/* called when finding the version of the data */
void print_data_version(struct file_info *fi)
{
	uint8_t *buf;
	char vers_buf[VBUF_MAX+1]; /* +1 for terminator */
	int found = 0;
	int size;

	buf = fi->buffer;
	size = fi->bsize;
	memset(vers_buf, 0, sizeof(vers_buf));

	if (is_all_1s(buf, size)) {
		printf("%s is erased (all 1s)\n", file_name(fi->part));
		return;
	}
	if (is_all_0s(buf, size)) {
		printf("%s is zeroed\n", file_name(fi->part));
		return;
	}

	if (fi->part == PART_OPROM || fi->part == PART_BULK) {
		/* UEFI drivers */
		found = get_version_string(buf, size, vers_buf, VBUF_MAX);
	} else if (fi->part == PART_CONFIG) {
		/* platform config file */
		found = parse_platform_config(buf, size, vers_buf, VBUF_MAX);
	} else {
		return;
	}

	if (found)
		printf("%s version: %s\n", file_name(fi->part), vers_buf);
	else
		printf("%s version: not found\n", file_name(fi->part));
}

void print_of(const char *what, uint32_t offset_k, uint32_t total_k)
{
	const uint32_t print_size_k = B_TO_KB(64 * 1024);

	/* show something every print_size bytes */
	if (verbose > 1 && ((offset_k % print_size_k) == 0)) {
		printf("...%s %dK of %dK\n", what, offset_k, total_k);
		fflush(stdout);
	}
}

/* read from the EPROM into the buffer, then write the buffer to the file */
void do_read(int dev_fd, struct file_info *fi)
{
	uint32_t offset;
	uint32_t total_k;

	if (verbose)
		printf("Reading %s\n", file_name(fi->part));
	total_k = B_TO_KB(fi->bsize);

	/* read into our buffer */
	for (offset = 0; offset < fi->bsize; offset += EP_PAGE_SIZE) {
		read_page(dev_fd, fi->start + offset,
					(uint32_t *)(fi->buffer + offset));
		print_of("read", B_TO_KB(offset + EP_PAGE_SIZE), total_k);

		progress_spin_advance();
	}

	fi->fsize = get_image_size(fi->buffer, fi->bsize);
}

void do_read_version(int dev_fd, struct file_info *fi)
{
	progress_spin_init("%s version: ", file_name(fi->part));
	do_read(dev_fd, fi);
	progress_spin_clear();
	print_data_version(fi);
}

void do_read_file(int dev_fd, struct file_info *fi)
{
	progress_spin_init("Reading %s... ", file_name(fi->part));
	do_read(dev_fd, fi);
	write_system_file(fi);
	progress_spin_fini();
}

void write_enable(int dev_fd)
{
	/* raise signal */
	write_reg(dev_fd, ASIC_GPIO_OUT, EPROM_WP_N);
	/* raise enable */
	write_reg(dev_fd, ASIC_GPIO_OE, EPROM_WP_N);
}

void write_disable(int dev_fd)
{
	/* assumes output already enabled */
	write_reg(dev_fd, ASIC_GPIO_OUT, 0);	/* lowers signal */
	write_reg(dev_fd, ASIC_GPIO_OE, 0);	/* remove enable */
}

void do_erase(int dev_fd, struct file_info *fi)
{
	write_enable(dev_fd);

	progress_spin_init("Erasing %s... ", file_name(fi->part));

	if (fi->part < 0) {
		if (verbose)
			printf("Erasing whole chip\n");
		erase_chip(dev_fd);
	} else {
		if (verbose)
			printf("Erasing %s\n", file_name(fi->part));
		erase_range(dev_fd, fi->start, fi->bsize);
	}

	progress_spin_fini();

	write_disable(dev_fd);
}

/* write to the EPROM, from the file, using the buffer as a stage */
void do_write(int dev_fd, struct file_info *fi)
{
	uint32_t offset;
	uint32_t total_k;

	if (verbose)
		printf("Writing %s at 0x%x len %lx\n",
		       fi->name, fi->start, fi->bsize);
	total_k = B_TO_KB(fi->bsize);

	progress_spin_init("Writing %s... ", file_name(fi->part));

	/* data is already read from the file into the buffer */

	write_enable(dev_fd);

	/* write from buffer into EPROM */
	for (offset = 0; offset < fi->bsize; offset += EP_PAGE_SIZE) {
		write_page(dev_fd, fi->start + offset,
					(uint32_t *)(fi->buffer + offset));
		print_of("wrote", B_TO_KB(offset + EP_PAGE_SIZE), total_k);

		progress_spin_advance();
	}

	progress_spin_fini();

	write_disable(dev_fd);
}

uint32_t do_info(int dev_fd)
{
	return read_device_id(dev_fd);
}

bool enable_device()
{
	FILE *fp;
	char boolean_value = '1';

	fp = fopen(enable_file, "w");
	if (fp == NULL) {
		fprintf(stderr, "Unable to open device enable file as writable: %s\n",
			enable_file);

		return false;
	}

	fwrite(&boolean_value, sizeof(boolean_value), sizeof(boolean_value), fp);
	fclose(fp);

	return true;
}

bool is_valid_dev(int dev_entry)
{
        return dev_entry >= 0 && dev_entry < num_dev_entries;
}

bool set_pci_files(int entry)
{
	if(!is_valid_dev(entry))
		return false;

	snprintf(resource_file, sizeof(resource_file),
		resource_fmt, pci_device_path, pci_device_addrs[entry]);
	snprintf(enable_file, sizeof(enable_file),
		enable_fmt, pci_device_path, pci_device_addrs[entry]);

	return true;
}

void list_all_devices(FILE* file)
{
	int i;
	for (i = 0; i < num_dev_entries; i++) {
		fprintf(file, "%d: %s/%s/resource0\n",
			i, pci_device_path, pci_device_addrs[i]);
	}
}

void invalid_device()
{
	fprintf(stderr, "Incorrect device specified, "
		"HFI discrete device is required\n"
		"Specify one from the list below:\n");
	list_all_devices(stderr);
	exit(1);
}

bool choose_device(const char* dev_name, int* last_dev)
{
	int i;
	char buffer[sizeof(int)*8+1];

	if (!dev_name) {
		if(is_valid_dev(*last_dev))
			return false;

		set_pci_files(0);

		*last_dev = 0;

	} else if (!strcmp(dev_name, "all")) {
		*last_dev += 1;
		if(!set_pci_files(*last_dev))
			return false;

	} else if (i = atoi(dev_name), sprintf(buffer, "%d", i),
			0 == strcmp(dev_name, buffer)) {

		if(is_valid_dev(*last_dev))
			return false;

		if (!set_pci_files(i))
			invalid_device();

		*last_dev = i;
	} else {
		if(is_valid_dev(*last_dev))
			return false;

		for (i = 0; i < num_dev_entries; i++) {
			set_pci_files(i);
			*last_dev = i;
			if (!strcmp(dev_name, resource_file)
				|| !strcmp(dev_name, pci_device_addrs[i])
				|| (!strcmp(dev_name, PCI_SHORT_ADDR(pci_device_addrs[i]))
					&& !strncmp(pci_device_addrs[i], "0000:", 5))) {

				break;
			}
		}

		if (i == num_dev_entries) {
			invalid_device();
		}
	}

	printf("Using device: %s\n", resource_file);
	return true;
}

int do_init()
{
	/*
	 * If the driver has never been loaded, the device is not
	 * enabled. Enable the device every time.
	 */
	if(!enable_device()) {
		return -1;
	}

	int dev_fd = open(resource_file, O_RDWR);
	if (dev_fd < 0) {
		fprintf(stderr, "Unable to open file [%s]\n", resource_file);
		return -1;
	}

	reg_mem = mmap(NULL, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED,
				dev_fd, 0);

	if (reg_mem == MAP_FAILED) {
		fprintf(stderr, "Unable to mmap %s, %s\n", resource_file,
			strerror(errno));
		close(dev_fd);
		return -1;
	}

	init_eep_interface(dev_fd);

	return dev_fd;
}

void do_cleanup(int dev_fd)
{
	if (reg_mem)
		munmap(reg_mem, MAP_SIZE);

	close(dev_fd);
}

/* ========================================================================== */

/* return the file name */
const char *file_name(int part)
{
	switch (part) {
	case PART_ALL: return "whole chip";
	case PART_OPROM: return "loader file";
	case PART_CONFIG: return "config file";
	case PART_BULK: return "driver file";
	}
	return "unknown";
}

/*
 * Initialize the file info fields.
 */
void prepare_file(int op, int partition, const char *fname,
		  struct file_info *fi)
{
	ssize_t nread;
	int flags;
	mode_t mode;
	struct stat st;
	int read_op_cnt = 0;

	if (fname) {
		if(op == DO_READ)
		{
			char tmp_fname[PATH_MAX] = { 0 };

			strncpy(tmp_fname, fname, ARRAY_SIZE(tmp_fname)-1);
			while (!stat(tmp_fname, &st))
				snprintf(tmp_fname, ARRAY_SIZE(tmp_fname) - 1,
					 "%s_%d", fname, read_op_cnt++);
			fi->name = strdup(tmp_fname);
		} else {
			fi->name = strdup(fname);
		}
		if (!fi->name) {
			fprintf(stderr,
				"Unable to allocate memory for filename\n");
			exit(1);
		}
	} else {
		fi->name = NULL;
	}

	fi->part = partition;
	fi->fd = -1;
	fi->buffer = NULL;

	/* start and buffer (partition) size */
	if (fi->part < 0) {
		fi->start = 0;
		fi->bsize = (dev_mbits * ((1024*1024) / 8));
	} else if (fi->part == PART_OPROM) {
		fi->start = 0;
		fi->bsize = P0_SIZE;
	} else if (fi->part == PART_CONFIG) {
		fi->start = P1_START;
		fi->bsize = P1_SIZE;
	} else if (fi->part == PART_BULK) {
		fi->start = P2_START;
		if (dev_mbits == 0)
			fi->bsize = 0;
		else
			fi->bsize = (dev_mbits * ((1024*1024) / 8)) - P2_START;
	}

	if (op == DO_ERASE || op == DO_INFO) {
		/* no buffer or file for erase and info */
		return;
	}

	if (op == DO_READ || op == DO_WRITE || op == DO_UPDATE) {
		/* open our file system file */
		if (op == DO_READ) {
			/* read from EPROM, write to file */
			flags = O_WRONLY | O_CREAT | O_TRUNC;
			mode = S_IRWXU | S_IRGRP | S_IROTH;
		} else {
			/* write to EPROM, read from file */
			flags = O_RDONLY;
			mode = 0;
		}

		fi->fd = open(fi->name, flags, mode);
		if (fi->fd < 0) {
			fprintf(stderr, "Cannot open file \"%s\": %s\n",
				fi->name, strerror(errno));
			exit(1);
		}
	} /* else op == DO_VERSION - fall through to create a buffer */

	/*
	 * Allocate a read/write buffer.  Always make this the partition size
	 * to avoid complicating the read and write routines.  When writing
	 * the parition, always write the whole thing even though the actual
	 * file will be smaller.  When reading the partition, always read the
	 * whole thing because the actual working size is unkown.
	 */
	fi->buffer = malloc(fi->bsize);
	if (!fi->buffer) {
		fprintf(stderr, "Unable to allocate 0x%lx sized buffer\n",
			fi->bsize);
		exit(1);
	}

	/*
	 * Always init buffer to 0xff (not 0x00)!
	 * 1ns represent erased data in EPPROM memory.
	 * The write_page() function take advantage of the fact that
	 * most of the driver buffer is all 1ns.
	 */
	memset(fi->buffer, 0xff, fi->bsize);

	/* if writing, read from file into buffer */
	if (op == DO_WRITE || op == DO_UPDATE) {
		struct stat sbuf;

		if (fstat(fi->fd, &sbuf) == -1) {
			fprintf(stderr, "Unable to stat \"%s\": %s\n",
				fi->name, strerror(errno));
			exit(1);
		}

		/* the input file cannot be larger than the parition */
		if (sbuf.st_size > fi->bsize) {
			fprintf(stderr, "File size 0x%lx is larger than partition size 0x%lx\n",
				sbuf.st_size, fi->bsize);
			exit(1);
		}

		/* read from the file into the buffer */
		fi->fsize = sbuf.st_size;
		nread = read(fi->fd, fi->buffer, sbuf.st_size);
		if (nread < 0) {
			fprintf(stderr, "Read fail for \"%s\": %s\n", fi->name,
				strerror(errno));
			exit(1);
		}
		if (nread != sbuf.st_size) {
			fprintf(stderr, "Read only 0x%lx of expectd 0x%lx bytes for \"%s\"\n",
				nread, sbuf.st_size, fi->name);
			exit(1);
		}

		add_magic_bits_to_image(sbuf.st_size, fi->bsize, fi->buffer);
	}
}

void clean_file(struct file_info *fi)
{
	if (fi->name)
		free(fi->name);
	if (fi->fd >= 0)
		close(fi->fd);
	if (fi->buffer)
		free(fi->buffer);
}

char *operation_str(int operation)
{
	switch (operation) {
		case DO_NOTHING: return "DO NOTHING";
		case DO_READ:    return "READ from";
		case DO_WRITE:   return "WRITE to";
		case DO_ERASE:   return "ERASE";
		case DO_INFO:    return "read INFO from";
		case DO_VERSION: return "read VERSION from";
		case DO_UPDATE:  return "update ";
	}
	return "<unknown>";
}

void print_config_change_warning(void) {

		printf(
"\n"
"     ***** WARNING *****\n"
"\n"
"     Modification of HFI platform configuration may prevent the\n"
"     HFI from training the link.\n"
"\n"
"     Care should be taken to ensure that platform configuration\n"
"     information being written is correct for the HFI model being\n"
"     targeted.  If unsure contact support.\n"
"\n"
"     ***** WARNING *****\n"
"\n"
		);
}

void verify_with_user(int operation) {
	char response[128];
	char *rc;

	printf (
"\n"
"To silence this warning see option \"-y\"\n"
"\n"
"You are about to %s %s\n"
"     do you wish too proceed (\"Yes\" to continue)? ",
		operation_str(operation), resource_file);

	rc = fgets(response, sizeof(response), stdin);
	if (rc == NULL)
		exit(1);

	response[strcspn(response, " \t\n\0")] = '\0';

	if (strncmp(response, "Yes", sizeof(response)) != 0)
		exit(1);

	printf ( "\nProceeding with %s device %s\n",
		operation_str(operation), resource_file);
}

void warn_user(int operation, int partition)
{
	if (silence_warnings)
		return;

	if (operation == DO_ERASE || operation == DO_WRITE) {
		if (partition == PART_CONFIG)
			print_config_change_warning();

		verify_with_user(operation);
	}

	if (operation == DO_UPDATE && partition == PART_CONFIG) {
		print_config_change_warning();
		verify_with_user(operation);
	}
}

void do_operation(int operation, int dev_fd, int partition, const char *fname)
{
	struct file_info fi;

	warn_user(operation, partition);

	/* open file if needed, get sizes */
	prepare_file(operation, partition, fname, &fi);
	if (verbose) {
		printf("Operation \"%s\"\n", operation_str(operation));
		printf("File information:\n");
		printf("  part   %d\n", fi.part);
		printf("  fname   %s\n", fi.name);
		printf("  fsize  0x%lx\n", fi.fsize);
		printf("  start  0x%x\n", fi.start);
		printf("  bsize  0x%lx\n", fi.bsize);
		printf("  buffer %p\n", fi.buffer);
		printf("  fd     %d\n", fi.fd);
	}

	if (operation == DO_ERASE) {
		do_erase(dev_fd, &fi);
	} else if (operation == DO_WRITE) {
		do_erase(dev_fd, &fi);
		do_write(dev_fd, &fi);
	} else if (operation == DO_UPDATE) {
		do_erase(dev_fd, &fi);
		do_write(dev_fd, &fi);
	} else if (operation == DO_READ) {
		do_read_file(dev_fd, &fi);
	} else if (operation == DO_VERSION) {
		do_read_version(dev_fd, &fi);
	}

	clean_file(&fi);
}

/*
 *  enumerate_devices() finds HFI discrete devices and stores pci bus
 *  addresses in gobal array: pci_device_addrs
 */
void enumerate_devices(void)
{
	DIR *dir = opendir(pci_device_path);
	struct dirent *dentry;

	if (!dir) {
		fprintf(stderr, "Unable to open %s\n", pci_device_path);
		exit(1);
	}

	/* Search through the directory looking for device files */
	num_dev_entries = 0;
	while ((dentry = readdir(dir))) {
		FILE *file;
		char buf[7];
		char *buf_ptr;

		snprintf(device_file, sizeof(device_file), device_fmt, pci_device_path, dentry->d_name);

		/* try to open the file, it may error, ignore */
		file = fopen(device_file, "r");
		if (!file)
			continue;

		memset(buf, '\0', sizeof(buf));
		buf_ptr = fgets(buf, sizeof(buf), file);
		fclose(file);
		if (!buf_ptr)
			continue;

		if (!strncmp(buf, pci_eprom_device, 6)) {
			if (strlen(dentry->d_name) > MAX_PCI_BUS_LEN - 1) {
				fprintf(stderr, "Device pci address is too long: %s\n", dentry->d_name);
				exit(1);
			}
			if (num_dev_entries == MAX_DEV_ENTRIES) {
				fprintf(stderr, "Too many HFI discrete devices found in the system\n");
				exit(1);
			}
			strcpy(pci_device_addrs[num_dev_entries], dentry->d_name);
			num_dev_entries++;
		}
	}

	if (num_dev_entries > 0)
		set_pci_files(0);
	closedir(dir);
}

/* Function detects if file is valid OPA driver image
 * or platform configuration file.
 * Currently checks file signature and version
 */
int get_image_type(const char *fname) {
	int fd;
	struct stat sbuf;
	int part = PART_NONE;
	ssize_t nread;
	void* buf = NULL;
	char vers_buf[VBUF_MAX+1] = {}; /* +1 for terminator */


	fd = open(fname, O_RDONLY, 0);
	if (fd < 0) {
		fprintf(stderr, "Cannot open file \"%s\": %s\n",
			fname, strerror(errno));
		goto done;
	}
	if (fstat(fd, &sbuf) == -1) {
		fprintf(stderr, "Unable to stat \"%s\": %s\n",
			fname, strerror(errno));
		goto done;
	}
	if (!S_ISREG(sbuf.st_mode)) {
		if (verbose) {
			printf("File  \"%s\" is not a regilar file\n",fname);
		}
		goto done;
	}
	if (sbuf.st_size < 8 || sbuf.st_size > 1000000) {
		if (verbose) {
			printf("Unexpectd file size \"%s\": %ld\n",
			       fname, sbuf.st_size);
		}
		goto done;

	}

	buf = malloc(sbuf.st_size + 4);
	if (!buf) {
		goto done;
	}
	nread = read(fd, buf, sbuf.st_size);
	if (nread != sbuf.st_size) {
		fprintf(stderr, "Read only %ld of expectd %ld bytes for \"%s\"\n",
			nread, sbuf.st_size, fname);
		goto done;
	}
	if (*(uint16_t*)buf == ROM_IMAGE_SIGNATURE) {
		if (get_version_string(buf, sbuf.st_size, vers_buf, VBUF_MAX)) {
			part = PART_OPROM;
		}
	} else if (*(uint16_t*)buf == EFI_IMAGE_SIGNATURE) {
		if (get_version_string(buf, sbuf.st_size, vers_buf, VBUF_MAX)) {
			part = PART_BULK;
		}
	} else if (*(uint32_t*)buf == PLATFORM_CONFIG_MAGIC_NUM) {
		if (parse_platform_config(buf, sbuf.st_size, vers_buf, VBUF_MAX)) {
			part = PART_CONFIG;
		}
	}

done:
	if (fd) {
		close(fd);
	}
	if (buf) {
		free(buf);
	}
	return part;
}

void get_attached_driver() {
	FILE *pp;
	FILE *check_hfi;
	FILE *Version;
	int i,j;

	char *line;
	char buf[256];
	char command[256];
	char command2[256];
	char command3[256];
	char res[256];

	printf("HFI UEFI driver was attached to following PCI devices\n");
	pp = popen("ls /sys/firmware/efi/efivars/*-driver-version-* 2> /dev/null", "r");
	if (pp) {
		for(line = fgets(buf, sizeof buf, pp); line != NULL; line = fgets(buf, sizeof buf, pp)) {
			j= 0; i = 0;
			while(line[j] != '-') {
				if( line[j] == '\n') {
					break;
				}
				if( line[j] == '/') {
					i = j;
				}
				j++;
			}
			line[j] = '\0';
			strcpy(command, "lspci -s ");
			strcat(command,(line + i + 1));
			strcat(command," | grep HFI");
			check_hfi = popen(command, "r");
			if (check_hfi) {
				if(fgets(command2, sizeof command2, check_hfi)){
					strcpy(command3, "cat /sys/firmware/efi/efivars/");
					strcat (command3,(line + i + 1));
					strcat (command3,"-driver-version-* | sed -e 's/[^A-Z0-9a-z.]//g'");
					Version = popen(command3, "r");
					if (Version) {
						if (fgets(res, sizeof res, Version))
							printf("Device: %s \tDriver Version: %s\n", (line + i + 1), res);
						pclose(Version);
					}
				}
				pclose(check_hfi);
			}
		}
		pclose(pp);
	}

}

void usage(void)
{
	char dev_select_help[512] = "";

	snprintf(dev_select_help, sizeof(dev_select_help),
		 "                Examples:\n"
		 "                  -d %s\n"
		 "                  -d %s\n"
		 "                  -d %s\n"
		 "                  -d %d\n"
		 "                  -d all - to select all available devices\n"
		 "                  -d     - to list all available devices\n",
		 resource_file, pci_device_addrs[0], PCI_SHORT_ADDR(pci_device_addrs[0]), 0);

	printf("\n"
	       "Usage: hfi1_eprom [-d device] -u [loaderfile] [driverfile]\n"
	       "       hfi1_eprom [-d device] -V\n"
	       "       hfi1_eprom -h\n"
	       "       hfi1_eprom -S\n"
	       "\n"
	       "Update or list image versions on Intel Omni-Path HFI Adapter EPROM\n"
	       "\n"
	       "Options:\n"
	       "  -u            update the given file(s) on the EPROM\n"
	       "  -V            print the versions of all files written in the EPROM\n"
	       "  -d device     specify the device file to use\n"
	       "                or list devices if none is specified\n"
	       "%s"
	       "  -v            verbose output, also print application version\n"
	       "  -h            print help\n"
	       "  -S            service mode with additional options\n"
	       "\nExamples:\n"
	       "  hfi1_eprom -d all -u /usr/share/opa/bios_images/*\n"
	       "  hfi1_eprom -d all -V\n"
	       "\n", dev_select_help);

	if (!service_mode)
		return;

	printf("\nService mode:\n"
	       "     ***** WARNING *****\n"
	       "\n"
	       "     Service options may corrupt HFI adapter EPROM contents.\n"
	       "     Use with caution!\n"
	       "\n"
	       "     ***** WARNING *****\n"
	       "Usage: hfi1_eprom -S -w [-o loaderfile][-c configfile][-b driverfile][allfile]\n"
	       "       hfi1_eprom -S -r [-o loaderfile][-c configfile][-b driverfile][allfile]\n"
	       "       hfi1_eprom -S -e [-o][-c][-b]\n"
	       "       hfi1_eprom -S -u [configfile]\n"
	       "       hfi1_eprom -S -i\n"
	       "\n"
	       "Write, read or erase images on Intel Omni-Path HFI adapter EPROM.\n"
	       "Above operation may be performed on a specific file or the whole device.\n"
	       "\n"
	       "Options:\n"
	       "  -w              write the given file(s) to the EPROM\n"
	       "  -r              read the given file(s) from the EPROM\n"
	       "  -e              erase the given file type or whole EPROM\n"
	       "    allfile       name of file to use for reading or writing the whole device\n"
	       "    -o loaderfile use the driver loader (option rom) file (.rom)\n"
	       "    -b driverfile use the EFI driver file (.efi)\n"
	       "    -c configfile use the platform configuration file\n");


	print_config_change_warning();

	printf("  -m              show format version of platform configuration file\n"
	       "  -i              print the EPROM device ID\n"
	       "  -s size         override EPROM size, must be power of 2, in Mbits\n"
	       "  -y              Answer (y)es : Silence warnings and confirmations\n"
	       "  -z              show actual version of UEFI driver attached to HFIs\n"
	       "\n");
}

void only_one_operation(void)
{
	fprintf(stderr, "Only one operation may be performed at a time\n");
	usage();
	exit(1);
}

void do_error(const char *message)
{
	fprintf(stderr, "%s", message);
	usage();
	exit(1);
}

int main(int argc, char **argv)
{
	const char *dev_name = NULL;
	uint32_t override_mbits = 0;
	int operation = DO_NOTHING;
	char *end;
	int opt;
	bool do_oprom_part = false;
	bool do_config_part = false;
	bool do_bulk_part = false;
	const char *oprom_name = NULL;
	const char *config_name = NULL;
	const char *bulk_name = NULL;
	const char *all_name = NULL;
	#define MAX_UPDATE_NAMES  8
	const char *update_name[MAX_UPDATE_NAMES + 1] = {};
	int update_name_count = 0;
	const char** optarg_dst = NULL;
	bool do_list_devices = false;

	/* isolate the command name */
	command = strrchr(argv[0], '/');
	if (command)
		command++;
	else
		command = argv[0];

	enumerate_devices();

	/*
	 * 1. Use '-' at the beginning of getopt's string permits arguments
	 *    that are not options to be returned as if they were associated
	 *    with option character ‘\1’
	 * 2. Use ':' at the start of getopt's string return a ':' for options
	 *    that require an argument but don't have one.  Erase and
	 *    version expect individual options to not have an argument.
	 */
	while ((opt = getopt(argc, argv, "-:bcdeShiors:vVwuymz")) != -1) {
		switch (opt) {
		case '\1':
			if(!optarg_dst) {
				fprintf(stderr, "Unrecognized argument %s\n", optarg);
				usage();
				exit(1);
			}
			if(optarg_dst == &dev_name)
				do_list_devices = false;

			*optarg_dst = optarg;
			if (optarg_dst == &update_name[update_name_count]) {
				if (update_name_count >= MAX_UPDATE_NAMES) {
					fprintf(stderr, "Too many file names\n");
					usage();
					exit(1);
				}

				optarg_dst = &update_name[++update_name_count];
			} else {
				optarg_dst = NULL;
			}
			break;
		case ':':
			fprintf(stderr, "An argument is required"
				" for option '%c'\n", optopt);
			usage();
			exit(1);
			break;
		case 'b':
			do_bulk_part = true;
			optarg_dst = &bulk_name;
			break;
		case 'c':
			do_config_part = true;
			optarg_dst = &config_name;
			break;
		case 'd':
			do_list_devices = true;
			optarg_dst = &dev_name;
			break;
		case 'e':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_ERASE;
			optarg_dst = NULL;
			break;
		case 'h':
			usage();
			exit(0);
		case 'i':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_INFO;
			optarg_dst = NULL;
			break;
		case 'o':
			do_oprom_part = true;
			optarg_dst = &oprom_name;
			break;
		case 'r':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_READ;
			optarg_dst = &all_name;
			break;
		case 's':
			override_mbits = strtoul(optarg, &end, 0);
			/*
			 * Reject if:
			 *   - not all bytes converted
			 *   - zero value
			 *   - must be power of 2
			 */
			if (*end != 0 || override_mbits == 0 ||
				(override_mbits & (override_mbits - 1)) != 0) {

				fprintf(stderr, "Invalid size \"%s\"\n",
					optarg);
				usage();
				exit(1);
			}
			optarg_dst = NULL;
			break;
		case 'w':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_WRITE;
			optarg_dst = &all_name;
			break;
		case 'u':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_UPDATE;
			optarg_dst = &update_name[0];
			break;
		case 'v':
			verbose++;
			optarg_dst = NULL;
			break;
		case 'V':
			if (operation != DO_NOTHING)
				only_one_operation();
			operation = DO_VERSION;
			optarg_dst = NULL;
			break;
		case 'm':
			print_meta = true;
			break;
		case 'y':
			silence_warnings = true;
			break;
		case 'z':
                        operation = DO_EFI_VERSION;
                        break;
		case 'S':
			service_mode = true;
			break;
		case '?':
			fprintf(stderr, "Unrecognized option -%c\n", optopt);
			usage();
			exit(1);
			break;
		}
	}

	if (do_list_devices && operation != DO_NOTHING)
		only_one_operation();

	if (verbose) {
		printf("Version: %s\n", PACKAGE_VERSION);
	}

	if (do_list_devices) {
		list_all_devices(stdout);
		exit(0);
	}


	if (operation == DO_EFI_VERSION) {
		if (service_mode == true) {
			get_attached_driver();
			exit(0);
		} else {
			usage();
			exit(0);
		}
	}

	if (operation == DO_UPDATE) {
		int part;
		int update_count = 0;
		int i;

		if (update_name_count == 0) {
			do_error("Update command requires at least 1 file name\n)");
		}

		for (i = 0; i < update_name_count; i++) {
			part = get_image_type(update_name[i]);
			switch (part) {
			case PART_OPROM:
				if (do_oprom_part) {
					do_error("Too many oprom files\n");
				}
				do_oprom_part = true;
				oprom_name = update_name[i];
				break;
			case PART_BULK:
				if (do_bulk_part) {
					do_error("Too many driver files\n");
				}
				do_bulk_part = true;
				bulk_name = update_name[i];
				break;
			case PART_CONFIG:
				if (service_mode) {
					if (do_config_part) {
						do_error("Too many configuration files\n");
					}
					do_config_part = true;
					config_name = update_name[i];
					break;
				}
				/* Fall through - ignore config file if not
				 * in service mode
				 */
			default:
				if (verbose) {
					printf("File %s not recognized\n",
					       update_name[i]);
				}
				continue;
			}
			printf("Updating %s with \"%s\"\n",
			       file_name(part), update_name[i]);
			update_count++;
		}
		if (!update_count) {
			do_error("No valid loader (.rom) or driver (.efi) file found\n");
		}
	}

	/* if no operation is specified, print usage */
	if (operation == DO_NOTHING) {
		usage();
		exit(0);
	}

	if (operation == DO_VERSION && !do_oprom_part && !do_config_part
				&& !do_bulk_part) {
		do_oprom_part = true;
		do_config_part = true;
		do_bulk_part = true;
	}

	if (!service_mode &&
	    !(operation == DO_UPDATE || operation == DO_VERSION)) {
		usage();
		exit(1);
	}

	if ((operation == DO_ERASE || operation == DO_VERSION) && (all_name ||
					((do_oprom_part && oprom_name)
					|| (do_config_part && config_name)
					|| (do_bulk_part && bulk_name)))) {
		fprintf(stderr, "Erase and version takes no file names\n");
		usage();
		exit(1);
	}

	if ((operation == DO_READ || operation == DO_WRITE) && !all_name
			&& !(do_oprom_part || do_config_part || do_bulk_part)) {
		fprintf(stderr, "Read or write requires a file name"
			" or individual file names\n");
		usage();
		exit(1);
	}

	if ((operation == DO_READ || operation == DO_WRITE) && all_name
			&& (do_oprom_part || do_config_part || do_bulk_part)) {
		fprintf(stderr, "Read or write requires a file name"
			" or individual file names, but not both\n");
		usage();
		exit(1);
	}

	if ((operation == DO_READ || operation == DO_WRITE) && (
					(do_oprom_part && !oprom_name)
					|| (do_config_part && !config_name)
					|| (do_bulk_part && !bulk_name))) {
		fprintf(stderr, "Read or write requires a named file\n");
		usage();
		exit(1);
	}

	if (operation == DO_INFO && (all_name || do_oprom_part
					|| do_config_part || do_bulk_part)) {
		fprintf(stderr, "Info takes no file arguments\n");
		usage();
		exit(1);
	}

	if (num_dev_entries == 0) {
		fprintf(stderr, "Unable to find an HFI discrete device\n");
		exit(1);
	}

	int ret_code = 0;
	int last_dev = -1;

	while(choose_device(dev_name, &last_dev)) {

		/* do steps for all operations */

		int dev_fd = do_init();
		if(dev_fd < 0) {
			ret_code = 1;
			continue;
		}

		/* find the size */
		dev_id = do_info(dev_fd);
		if (dev_id == 0xFFFFFFFF) {
			fprintf(stderr, "Unable to read device id.\n");
			ret_code = 1;
			goto device_cleanup;
		}

		dev_mbits = find_chip_bit_size(dev_id);
		if (dev_mbits == 0)
			printf("Device ID: 0x%08x, unrecognized - no size\n", dev_id);
		else
			if (operation == DO_INFO || verbose)
				printf("Device ID: 0x%08x, %d Mbits\n", dev_id,
					dev_mbits);
		if (override_mbits) {
			if (verbose)
				printf("Using override size of %d Mbits\n",
					override_mbits);
			dev_mbits = override_mbits;
		}

		/* if only gathering information, we're done */
		if (operation == DO_INFO)
			goto device_cleanup;

		/*
		* Fail if we don't have the device size and the operation
		* requires knowing the full chip size.
		*/
		if (dev_mbits == 0 && (operation == DO_READ || operation == DO_WRITE)
							&& (all_name || bulk_name)) {
			fprintf(stderr, "Cannot operate on device without the device size\n");
			ret_code = 1;
			goto device_cleanup;
		}

		/* do the operation(s) */
		if (all_name || (operation == DO_ERASE
				&& !(do_oprom_part || do_config_part || do_bulk_part))) {
			/* doing a whole-chip operation */
			do_operation(operation, dev_fd, PART_ALL, all_name);
		} else {
			/* doing an individual partition operation */
			if (do_oprom_part)
				do_operation(operation, dev_fd, PART_OPROM, oprom_name);
			if (do_bulk_part)
				do_operation(operation, dev_fd, PART_BULK, bulk_name);
			if (do_config_part)
				do_operation(operation, dev_fd, PART_CONFIG, config_name);
		}

	device_cleanup:
		do_cleanup(dev_fd);

	}

	return ret_code;
}
