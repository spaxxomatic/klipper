.DEFAULT_GOAL := all
ECHO := echo
MD := mkdir
RM := rm -f
CP := cp

TARGETLIB := c_helper.so
EXTRA_CFLAGS :=  -Wall -g -O2 -shared -fPIC -flto -fwhole-program 

SRC :=   $(wildcard *.c)

TARGET_DIR := out
VPATH = $(dir $(SRC)) 

FNAMES = $(notdir $(SRC)) 

o1 = $(patsubst %.c,%.o,$(FNAMES)) 
objnames = $(patsubst %.c,%.o,$(o1)) 

OBJ = $(addprefix $(TARGET_DIR)/,$(objnames))

list_objects:
	@echo $(FNAMES)
	@echo $(OBJ)

DEP = $(OBJ:.o=.d)
-include $(OBJ:.o=.d)

CC := gcc

.PHONY: prepare list_objects prepare_mock gccversion clean install

clean:
	$(RM)r $(TARGET_DIR)
	$(RM) $(FINALLIB)
	 
prepare:
	$(MD) -p $(TARGET_DIR)

#VPATH = $(dir $(MOCK_FILES)) $(dir $(SRC))
build: $(SRC) $(OBJ) 

link:
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS)  -o $(TARGETLIB) $(OBJ) $(LIB_LIST)

LIB_LIST = -lpthread -lwiringPi

build_mock: 
	@echo Source files: $(SRC)
	@echo Source obj: $(OBJ)
	@echo Mock files: $(MOCK_FILES)
	@echo Mock obj: $(MOCK_OBJ)
build_mock: $(MOCK_FILES) $(MOCK_OBJ)  $(SRC) $(OBJ) 
#build_mock:
#	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS) -s -o spaxxserver $(OBJ) $(MOCK_OBJ) -lm -lmosquitto -lrt -lpthread

mock: EXTRA_CFLAGS := $(EXTRA_CFLAGS) -D_MOCKED
#mock: $(SRC) += $(MOCK_FILES) 	
#mock: $(OBJ) += $(MOCK_OBJ) 

gccversion: 
	$(CC) --version
	
$(TARGET_DIR)/%.o : %.c
	@echo "Compiling $< " 
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@

all: gccversion prepare build link 
