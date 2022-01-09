SRC_DIR := src
OBJ_DIR := obj
IDIR := ./include

ifndef LOCAL
CC = arm-linux-gnueabihf-gcc
BIN_DIR := ../../bb_bin
LDLIBS := -pthread -lm -lrt -l:librobotcontrol.so.1
CPPFLAGS := -I$(IDIR) -I../../librobotcontrol/library/include
else
CC = gcc
BIN_DIR := ../../bin
LDLIBS := -pthread -lm -lrt
CPPFLAGS := -I$(IDIR)
endif

LDFLAGS := -L../../lib -MMD -MP
CFLAGS := -g -Wall -Wno-address-of-packed-member

SRC := $(wildcard $(SRC_DIR)/*.c)
DEPS := $(wildcard $(IDIR)/*.h)
OBJ := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC))
EXE := $(BIN_DIR)/$(TARGET)

.PHONY: all clean

all: $(EXE)
	@$(RM) *.d 2> /dev/null

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(DEPS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean:
	@$(RM) $(OBJ_DIR)/*

print:
	@echo $(SRC_DIR)
	@echo $(OBJ_DIR)
	@echo $(BIN_DIR)
	@echo $(SRC)
	@echo $(OBJ)
	@echo $(DEPS)

