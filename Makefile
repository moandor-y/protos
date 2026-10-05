AS := as
CXX := g++
LD := ld

ASFLAGS := --64
CXXFLAGS := -m64 -masm=intel -std=c++20 -O2 -Wall -Wextra -Werror \
            -ffreestanding -nostdlib -fno-builtin \
            -fno-exceptions -fcheck-new -fno-rtti -mno-red-zone \
            -fno-stack-protector -fno-pie
LDFLAGS := -m elf_x86_64 -z max-page-size=0x1000 --no-warn-rwx-segments -T linker.ld

TEST_COMMON_CXXFLAGS := -std=c++20 -Wall -Wextra -Werror -MMD -MP
TEST_COMMON_LDFLAGS := -lgmock_main -lgmock -lgtest -pthread

TEST_BUILD_MODES := debug opt
TEST_BUILD_FLAGS_debug := -O0 -g3 -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS
TEST_BUILD_FLAGS_opt := -O2 -g -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS

TEST_SAN_MODES := nosan asan ubsan lsan tsan
TEST_SAN_FLAGS_nosan :=
TEST_SAN_FLAGS_asan := -fsanitize=address,pointer-compare,pointer-subtract \
                       -fno-sanitize-recover=all
TEST_SAN_FLAGS_ubsan := -fsanitize=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow \
                        -fno-sanitize-recover=all
TEST_SAN_FLAGS_lsan := -fsanitize=leak -fno-sanitize-recover=all
TEST_SAN_FLAGS_tsan := -fsanitize=thread -fno-sanitize-recover=all

TEST_SAN_ENV_nosan :=
TEST_SAN_ENV_asan := ASAN_OPTIONS=strict_string_checks=1:detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:detect_invalid_pointer_pairs=2:detect_leaks=1
TEST_SAN_ENV_ubsan := UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1
TEST_SAN_ENV_lsan := LSAN_OPTIONS=print_suppressions=0
TEST_SAN_ENV_tsan := TSAN_OPTIONS=halt_on_error=1

HOST_TEST_SRCS := $(sort $(wildcard *_test.cpp))
HOST_TESTS := $(patsubst %.cpp,%,$(HOST_TEST_SRCS))
TEST_EXTRA_SRCS_heap_test := heap.cpp

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
    check.h \
    uart.h \
    vga.h \
    multiboot.h \
    paging.h \
    pmm.h \
    heap.h \
    rbtree.h \
    memory_tests.h

CXX_OBJS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CXX_SRCS))
OBJS := $(BUILD_DIR)/boot.o $(CXX_OBJS)

DOCKER ?= docker
DOCKER_IMAGE ?= x86-64-kernel-build
DOCKER_PLATFORM ?= linux/amd64
HOST_UID ?= $(shell id -u)
HOST_GID ?= $(shell id -g)
DOCKER_STAMP := $(BUILD_DIR)/.docker.stamp

IN_DOCKER := $(wildcard /.dockerenv /run/.containerenv)
ifeq ($(IN_DOCKER),)
DOCKER_RUN := $(DOCKER) run --rm --platform=$(DOCKER_PLATFORM) \
              --user $(HOST_UID):$(HOST_GID) \
              -e HOME=/tmp \
              -v "$(CURDIR):/workspace" \
              -w /workspace \
              $(DOCKER_IMAGE)
DOCKER_DEPS := $(DOCKER_STAMP)
else
DOCKER_RUN :=
DOCKER_DEPS :=
endif

.PHONY: all clean test test-host test-rbtree test-heap

all: $(BUILD_DIR)/kernel.bin $(BUILD_DIR)/kernel.iso

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(DOCKER_STAMP): Dockerfile | $(BUILD_DIR)
	$(DOCKER) build --platform=$(DOCKER_PLATFORM) -t $(DOCKER_IMAGE) .
	mkdir -p $(BUILD_DIR)/include
	$(DOCKER_RUN) cp -r /usr/include/gmock /usr/include/gtest $(BUILD_DIR)/include/
	touch $(DOCKER_STAMP)

$(BUILD_DIR)/boot.o: boot.S $(DOCKER_DEPS) | $(BUILD_DIR)
	$(DOCKER_RUN) $(AS) $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp $(CXX_HDRS) $(DOCKER_DEPS) | $(BUILD_DIR)
	$(DOCKER_RUN) $(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/kernel.bin: $(OBJS) linker.ld $(DOCKER_DEPS) | $(BUILD_DIR)
	$(DOCKER_RUN) $(LD) $(LDFLAGS) $(OBJS) -o $@

$(BUILD_DIR)/kernel.iso: $(BUILD_DIR)/kernel.bin Makefile $(DOCKER_DEPS) | $(BUILD_DIR)
	rm -rf $(BUILD_DIR)/isodir
	mkdir -p $(BUILD_DIR)/isodir/boot/grub
	cp $(BUILD_DIR)/kernel.bin $(BUILD_DIR)/isodir/boot/kernel.bin
	printf 'set timeout=0\nset default=0\n\nmenuentry "x86-64 Kernel" {\n    if [ "$${grub_platform}" = "efi" ]; then\n        insmod all_video\n    fi\n    multiboot2 /boot/kernel.bin\n    if [ "$${grub_platform}" = "pc" ]; then\n        set gfxpayload=text\n    fi\n    boot\n}\n' > $(BUILD_DIR)/isodir/boot/grub/grub.cfg
	$(DOCKER_RUN) grub-mkrescue -o $@ $(BUILD_DIR)/isodir
	rm -rf $(BUILD_DIR)/isodir

# $(call DEFINE_HOST_TEST_VARIANT,test_name,build_mode,san_mode)
define DEFINE_HOST_TEST_VARIANT
$(BUILD_DIR)/$(1)_$(2)_$(3): $(1).cpp $$(TEST_EXTRA_SRCS_$(1)) $$(DOCKER_DEPS) | $(BUILD_DIR)
	$$(DOCKER_RUN) $$(CXX) $$(TEST_COMMON_CXXFLAGS) $$(TEST_BUILD_FLAGS_$(2)) $$(TEST_SAN_FLAGS_$(3)) \
		$(1).cpp $$(TEST_EXTRA_SRCS_$(1)) $$(TEST_SAN_FLAGS_$(3)) $$(TEST_COMMON_LDFLAGS) -o $$@

.PHONY: run-$(1)-$(2)-$(3)
run-$(1)-$(2)-$(3): $(BUILD_DIR)/$(1)_$(2)_$(3)
	$$(DOCKER_RUN) env $$(TEST_SAN_ENV_$(3)) ./$(BUILD_DIR)/$(1)_$(2)_$(3)

HOST_TEST_BINS_$(1) += $(BUILD_DIR)/$(1)_$(2)_$(3)
HOST_TEST_RUNS_$(1) += run-$(1)-$(2)-$(3)
HOST_TEST_ALL_BINS += $(BUILD_DIR)/$(1)_$(2)_$(3)
HOST_TEST_ALL_RUNS += run-$(1)-$(2)-$(3)
endef

define DEFINE_HOST_TEST_SUITE
$(foreach b,$(TEST_BUILD_MODES),$(foreach s,$(TEST_SAN_MODES),$(eval $(call DEFINE_HOST_TEST_VARIANT,$(1),$(b),$(s)))))

.PHONY: test-$(1)
ifeq ($(IN_DOCKER),)
test-$(1): $(DOCKER_STAMP)
	$$(DOCKER_RUN) sh -c 'make --no-print-directory -j$$$$(nproc) $$(HOST_TEST_BINS_$(1)) && make --no-print-directory $$(HOST_TEST_RUNS_$(1))'
else
test-$(1): $$(HOST_TEST_RUNS_$(1))
endif
endef

$(foreach t,$(HOST_TESTS),$(eval $(call DEFINE_HOST_TEST_SUITE,$(t))))

test-rbtree: test-rbtree_test
test-heap: test-heap_test

ifeq ($(IN_DOCKER),)
test-host: $(DOCKER_STAMP)
	$(DOCKER_RUN) sh -c 'make --no-print-directory -j$$(nproc) $(HOST_TEST_ALL_BINS) && make --no-print-directory $(HOST_TEST_ALL_RUNS)'
else
test-host: $(HOST_TEST_ALL_RUNS)
endif

test: test-host $(BUILD_DIR)/kernel.iso $(DOCKER_DEPS)
	$(DOCKER_RUN) ./test_boot.sh $(BUILD_DIR)/kernel.iso

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
