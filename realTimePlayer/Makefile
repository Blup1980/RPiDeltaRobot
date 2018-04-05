# main compiler
CC := gcc
#CC := arm-linux-gnueabihf-gcc
TARGET1 := main
LIB := pthread

all: $(TARGET1)

$(TARGET1): $(TARGET1).c
	$(CC) $(CFLAGS) $(TARGET1).c -o $(TARGET1) $(LDFLAGS) -l$(LIB)

clean:
	@rm -rf $(TARGET1) $(TARGET2)
