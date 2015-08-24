#
# Makefile for the packet AF.
#

obj-$(CONFIG_PACKET) += af_packet.o

ifdef CONFIG_MACH_MSM8960_EF52_STRICT_ALIASING
CFLAGS_KERNEL := -fno-strict-aliasing
endif
