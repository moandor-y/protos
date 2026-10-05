FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/tmp

RUN apt-get update && apt-get install -y --no-install-recommends \
    binutils \
    g++ \
    grub-common \
    grub-efi-amd64-bin \
    grub-pc-bin \
    grub2-common \
    libgmock-dev \
    libgtest-dev \
    make \
    mtools \
    ovmf \
    python3 \
    qemu-system-x86 \
    xorriso \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
