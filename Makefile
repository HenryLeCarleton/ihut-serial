TARGET=ihut_serial
PACKAGE_NAME=$(TARGET)

SOURCES = serial.c
OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: ${TARGET}

%.o: %.c
	$(CC) -g -c -Wall $< -o $@

$(TARGET): $(OBJECTS)
	$(CC) $^ -o $@

clean:
	@rm -f $(TARGET) $(OBJECTS)
