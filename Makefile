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

DOCKER ?= docker
DOCKER_IMAGE ?= x86-64-kernel-build
DOCKER_PLATFORM ?= linux/amd64
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
DOCKER_STAMP := $(BUILD_DIR)/.docker.stamp

NEEDS_SG_DOCKER := $(shell [ -S /var/run/docker.sock ] && [ ! -w /var/run/docker.sock ] && command -v sg >/dev/null 2>&1 && echo 1 || echo 0)
ifeq ($(NEEDS_SG_DOCKER),1)
  DOCKER_EXEC = sg docker -c '$(1)'
else
  DOCKER_EXEC = $(1)
endif

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
	$(call DOCKER_EXEC,$(DOCKER) build --platform=$(DOCKER_PLATFORM) -t $(DOCKER_IMAGE) .)
	touch $(DOCKER_STAMP)

$(BUILD_DIR)/boot.o: boot.S $(DOCKER_STAMP) | $(BUILD_DIR)
	$(call DOCKER_EXEC,$(DOCKER_RUN) $(AS) $(ASFLAGS) $< -o $@)

$(BUILD_DIR)/kernel.o: kernel.cpp $(DOCKER_STAMP) | $(BUILD_DIR)
	$(call DOCKER_EXEC,$(DOCKER_RUN) $(CXX) $(CXXFLAGS) -c $< -o $@)

$(BUILD_DIR)/kernel.bin: $(BUILD_DIR)/boot.o $(BUILD_DIR)/kernel.o linker.ld $(DOCKER_STAMP) | $(BUILD_DIR)
	$(call DOCKER_EXEC,$(DOCKER_RUN) $(LD) $(LDFLAGS) $(BUILD_DIR)/boot.o $(BUILD_DIR)/kernel.o -o $@)

$(BUILD_DIR)/kernel.iso: $(BUILD_DIR)/kernel.bin $(DOCKER_STAMP) | $(BUILD_DIR)
	rm -rf $(BUILD_DIR)/isodir
	mkdir -p $(BUILD_DIR)/isodir/boot/grub
	cp $(BUILD_DIR)/kernel.bin $(BUILD_DIR)/isodir/boot/kernel.bin
	printf 'set timeout=0\nset default=0\n\nmenuentry "x86-64 Kernel" {\n    multiboot2 /boot/kernel.bin\n    boot\n}\n' > $(BUILD_DIR)/isodir/boot/grub/grub.cfg
	$(call DOCKER_EXEC,$(DOCKER_RUN) grub-mkrescue -o $@ $(BUILD_DIR)/isodir)
	rm -rf $(BUILD_DIR)/isodir

test: $(BUILD_DIR)/kernel.iso $(DOCKER_STAMP)
	$(call DOCKER_EXEC,$(DOCKER_RUN) ./test_boot.sh $(BUILD_DIR)/kernel.iso)

clean:
	rm -rf $(BUILD_DIR)
