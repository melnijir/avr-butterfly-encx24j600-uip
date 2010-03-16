#Chip choice
MCU	= atmega169
CHIP	= m169

#Project name
NAME    = simple-httpd

#Compiled apps
APPS	= simple-httpd

#Compiled drivers
DEVS	= encx24j600 #The rest are determined by makefiles for apps

TOPLEVEL = ..

#Programer details
PROGRAMMER = dapa
FULLCHIP = atmega16
PORT = /dev/parport0

#Compiled libs
#LIBS	= #Determined by makefiles for apps or drivers

#Project specific files
$(NAME)_SOURCES = main.c clock-arch.c LCD_driver.c

#Include global configurations
-include ../MakeConf.include

#Include the genetic makefile
-include ../Makefile.include
