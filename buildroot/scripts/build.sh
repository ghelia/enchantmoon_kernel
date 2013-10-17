#!/bin/bash
set -e

CUR_DIR=`pwd`

# clean
rm -rf ${CUR_DIR}/out/*

if [ ! -d ${CUR_DIR}/out ]; then
mkdir ${CUR_DIR}/out
fi

# buildroot
cd ${CUR_DIR}/buildroot && ./build.sh

# build kernel
export PATH=${CUR_DIR}/buildroot/output/external-toolchain/bin:$PATH
cd ${CUR_DIR}/linux-2.6.36 && ./build.sh kernel
cd ${CUR_DIR}/linux-2.6.36 && ./build.sh modules


# output
rm -rf ${CUR_DIR}/out/rootfs

# copy to out dir
cp -f ${CUR_DIR}/linux-2.6.36/output/?Image ${CUR_DIR}/buildroot/output/target/
mkdir -p ${CUR_DIR}/buildroot/output/target/lib/modules
cp -rf ${CUR_DIR}/linux-2.6.36/output/lib/modules/* ${CUR_DIR}/buildroot/output/target/lib/modules/
cp -f ${CUR_DIR}/linux-2.6.36/output/?Image ${CUR_DIR}/out/
cp -rf ${CUR_DIR}/buildroot/output/target ${CUR_DIR}/out/rootfs

cd ${CUR_DIR}/buildroot
make rootfs-ext2
make rootfs-tar


# copy to tools
cp -f ${CUR_DIR}/linux-2.6.36/output/bImage ${CUR_DIR}/buildroot/tools/pack/sun4i_pack_ddr2_win/wboot/bootfs/linux/
cp -f ${CUR_DIR}/linux-2.6.36/output/bImage ${CUR_DIR}/buildroot/tools/pack/sun4i_pack_ddr3_win/wboot/bootfs/linux/
cp -f ${CUR_DIR}/buildroot/output/images/rootfs.ext2 ${CUR_DIR}/buildroot/tools/pack/sun4i_pack_ddr3_win/wboot/rootfs.fex
cp -f ${CUR_DIR}/buildroot/output/images/rootfs.ext2 ${CUR_DIR}/buildroot/tools/pack/sun4i_pack_ddr2_win/wboot/rootfs.fex

# u-boot if needed
if [ -f ${CUR_DIR}/buildroot/output/images/u-boot.bin ]; then
	cp ${CUR_DIR}/buildroot/output/images/u-boot.bin ${CUR_DIR}/out/
	cp -rf ${CUR_DIR}/linux-2.6.36/output/uImage ${CUR_DIR}/out/rootfs/
	rm -rf ~/xyz_nfs
	ln -sv ${CUR_DIR}/out/rootfs ~/xyz_nfs
fi

