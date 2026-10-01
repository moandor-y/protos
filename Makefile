AS := as
CXX := g++
LD := ld

ASFLAGS := --64
CXXFLAGS := -m64 -masm=intel -std=c++20 -O2 -Wall -Wextra -Werror \
            -ffreestanding -nostdlib -fno-builtin \
            -fno-exceptions -fno-rtti -mno-red-zone \
            -fno-stack-protector -fno-pie
LDFLAGS := -m elf_x86_64 -z max-page-size=0x1000 --no-warn-rwx-segments -T linker.ld

BUILD_DIR := build

CXX_SRCS := \
    uart.cpp \
    vga.cpp \
    multiboot.cpp \
    paging.cpp \
    pmm.cpp \
    heap.cpp \
    memory_tests.cpp \
    kernel.cpp

CXX_HDRS := \
    uart.h \
    vga.h \
    multiboot.h \
    paging.h \
    pmm.h \
    heap.h \
    memory_tests.h

CXX_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CXX_SRCS))
OBJS := $(BUILD_DIR)/boot.o $(CXX_OBJS)

DOCKER ?= docker
DOCKER_IMAGE ?= x86-64-kernel-build
DOCKER_PLATFORM ?= linux/amd64
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
DOCKER_STAMP := $(BUILD_DIR)/.docker.stamp

DOCKER_RUN := $(DOCKER) run --rm --platform=$(DOCKER_PLATFORM) \
              --user $(HOST_UID):$(HOST_GID) \
              -e HOME=/tmp \
              -v "$(CURDIR):/workspace" \
              -w /workspace \
              $(DOCKER_IMAGE)

.PHONY: all clean test

all: $(BUILD_DIR)/kernel.bin $(BUILD_DIR)/kernel.iso

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(DOCKER_STAMP): Dockerfile | $(BUILD_DIR)
	$(DOCKER) build --platform=$(DOCKER_PLATFORM) -t $(DOCKER_IMAGE) .
	touch $(DOCKER_STAMP)

$(BUILD_DIR)/boot.o: boot.S $(DOCKER_STAMP) | $(BUILD_DIR)
	$(DOCKER_RUN) $(AS) $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp $(CXX_HDRS) $(DOCKER_STAMP) | $(BUILD_DIR)
	$(DOCKER_RUN) $(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/kernel.bin: $(OBJS) linker.ld $(DOCKER_STAMP) | $(BUILD_DIR)
	$(DOCKER_RUN) $(LD) $(LDFLAGS) $(OBJS) -o $@

$(BUILD_DIR)/kernel.iso: $(BUILD_DIR)/kernel.bin $(DOCKER_STAMP) | $(BUILD_DIR)
	rm -rf $(BUILD_DIR)/isodir
	mkdir -p $(BUILD_DIR)/isodir/boot/grub
	cp $(BUILD_DIR)/kernel.bin $(BUILD_DIR)/isodir/boot/kernel.bin
	printf 'set timeout=0\nset default=0\n\nmenuentry "x86-64 Kernel" {\n    multiboot2 /boot/kernel.bin\n    boot\n}\n' > $(BUILD_DIR)/isodir/boot/grub/grub.cfg
	$(DOCKER_RUN) grub-mkrescue -o $@ $(BUILD_DIR)/isodir
	rm -rf $(BUILD_DIR)/isodir

test: $(BUILD_DIR)/kernel.iso $(DOCKER_STAMP)
	$(DOCKER_RUN) ./test_boot.sh $(BUILD_DIR)/kernel.iso

clean:
	rm -rf $(BUILD_DIR)
