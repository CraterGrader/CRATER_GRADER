####################################################
# @file    Makefile
#
# @attention
#
# Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
#
# All rights reserved.
#
 
####################################################
#  Configurations

####################################################
#  TARGET
#  0: Raspberry-Pi
#  1: else
TARGET = 0

####################################################
#  INTERFACE_NUMBER
#  0: USE_UART  
#  1: USE_SPI    
#  2: USE_SPI_DRDY
INTERFACE_NUMBER = 0

####################################################
#  HAL_LOG_ENABLED
#  for   HAL_Log         
#  0:    not enabled      
#  1:    enabled
HAL_LOG_ENABLED = 1

PROGRAM = test_les
SOURCES = test_les.c


INCLUDES += ../test_util/test_util.h
SOURCES += ../test_util/test_util.c
LOGFILES += test_report.txt

PROJ_DIR += ../..

CFLAGS += -Wall

include $(PROJ_DIR)/include/dwm1001.mak
