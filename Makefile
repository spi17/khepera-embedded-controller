#
# (c) 2018 Brunete, Madrid, España
# Santiago Pariente
#

# The list of modules that this program needs
MODULES	:= khepera3 commandline i2cal measurement nmea

# The file name of the executable
TARGET	:= ks-measure-test

# If the K3_ROOT environment variable is set, you can comment out the following line
K3_ROOT	:= ../..

# Include the Makefile
include $(K3_ROOT)/Programs/Makefile.include
