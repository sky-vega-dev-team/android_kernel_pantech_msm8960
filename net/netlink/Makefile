#
# Makefile for the netlink driver.
#

obj-y  				:= af_netlink.o genetlink.o

ifdef CONFIG_MACH_MSM8960_EF52_STRICT_ALIASING
CFLAGS_KERNEL := -fno-strict-aliasing
endif
