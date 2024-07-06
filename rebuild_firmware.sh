#!/bin/bash

BUILD_DIR=$PWD/build
ELF_FILENAME="neimark_1.elf"
BIN_FILENAME="neimark_1.bin"

if [ -d $BUILD_DIR ]; then
  rm -rf $BUILD_DIR
fi
mkdir $BUILD_DIR
cd $BUILD_DIR
cmake ..
make -j$(nproc)
arm-none-eabi-objcopy -S -O binary $ELF_FILENAME $BIN_FILENAME
cp $BIN_FILENAME /run/media/edgecrush3r/NODE_F410RB/
