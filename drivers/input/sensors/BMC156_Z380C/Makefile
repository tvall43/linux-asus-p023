#
# Makefile for Bosch sensor driver.
#
obj-y += bstclass.o

obj-$(CONFIG_SENSORS_BMA2X2_Z380C)	+= bma2x2.o
# obj-y	+= bma2x2.o

ifeq ($(CONFIG_BOSCH_BMA2X2_ENABLE_INT2),y)
	EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2
endif

EXTRA_CFLAGS += -DBMA2X2_ENABLE_INT2

obj-$(CONFIG_SENSORS_BMM050_Z380C)    += bmm050_driver.o bmm050.o
# obj-y    += bmm050_driver.o bmm050.o
EXTRA_CFLAGS += -DBMM_USE_BASIC_I2C_FUNC -DCONFIG_BMM_USE_PLATFORM_DATA -DCONFIG_FLICK -DCONFIG_FLICK_HV
# EXTRA_CFLAGS += -DBMM_USE_BASIC_I2C_FUNC -DCONFIG_BMM_USE_PLATFORM_DATA





