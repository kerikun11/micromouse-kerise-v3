#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_ADD_LDFLAGS += -lmazelib
COMPONENT_ADD_INCLUDEDIRS += mazelib/MazeLib/inc

COMPONENT_ADD_INCLUDEDIRS += config drivers utils peripheral
COMPONENT_SRCDIRS += config drivers utils peripheral
