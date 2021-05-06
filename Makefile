CC = gcc

TARGET = racetrack-controllers

SRCDIR = src
OBJDIR = obj
INCDIR = include
TARGETDIR = bin

# -g adds debugging information
# -Wall turns on all warnings
CFLAGS= -g -Wall

# linker options (which libraries to use)
LDFLAGS = -ltensorflow

SRC = $(wildcard $(SRCDIR)/*.c)
OBJ = $(SRC:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# includes
INCFLAGS = $(addprefix -I, $(INCDIR))

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p $(TARGETDIR)
	$(CC) $(LDFLAGS) -o $(TARGETDIR)/$(TARGET) $^

-include $(OBJ:.o=.d)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $@
	$(CC) -MM $(CFLAGS) $(INCFLAGS) $(SRCDIR)/$*.c > $(OBJDIR)/$*.d

.PHONY: clean

clean:
	rm -rf $(OBJDIR) $(TARGETDIR)
