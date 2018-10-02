#
# Component makefile
#
COMPONENT_ADD_INCLUDEDIRS := multipart-parser-c
COMPONENT_SRCDIRS := multipart-parser-c
## project uses char not uint8_t or unisgned char :(
CFLAGS += -Wno-char-subscripts
