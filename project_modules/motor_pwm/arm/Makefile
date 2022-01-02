CC = arm-linux-gnueabihf-gcc
SRC_DIR := src
OBJ_DIR := obj
#BIN_DIR := ~/ccp/bb_bin
BIN_DIR := bin
EXE := $(BIN_DIR)/motor_control

IDIR := ./include

LDFLAGS := -L./lib -MMD -MP
LDLIBS := -pthread -lm -lrt -l:librobotcontrol.so.1
CPPFLAGS := -I$(IDIR) -I/home/pala/ccp/librobotcontrol/library/include
CFLAGS := -g -Wall

SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC))

.PHONY: all clean

all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean:
	@$(RM) $(OBJ_DIR)/*


print:
	@echo $(SRC_DIR)
	@echo $(OBJ_DIR)
	@echo $(BIN_DIR)
	@echo $(SRC)
	@echo $(OBJ)

