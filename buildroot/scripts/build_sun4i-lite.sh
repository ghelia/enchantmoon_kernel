#!/bin/bash
set -e

############################################################################################
#
# Show how to build buildroot automatically -- Benn
#
###########################################################################################

CUR_DIR=`pwd`

#Uncomment here if you want
#LICHEE_KDIR=${CUR_DIR}/../linux-2.6.36/include

show_help()
{
    printf "\nValid Options:\n"
    printf "\tbuildroot - build buildroot\n"
    printf "\thelp      - print help\n"
    printf "\texternal  - build external\n\n"
}

# Build all for sun4i platform
build_buildroot()
{
    if [ ! -e .config ]; then
        printf "\nUsing default config... ...!\n"
        make sun4i-lite_defconfig
    fi

    make LICHEE_GEN_ROOTFS=n
}

EXTERNAL_DIR=${CUR_DIR}/external-packages

#############################################################################################
#
# Build external packages with some 'famous' variables
#     - DESTDIR       : image files
#     - CROSSS_COMPILE
#     - STAGING_DIR   : Libraries and header files
#     - TARGET_DIR    : Rootfs
#
############################################################################################
# if unset LICHEE_KDIR, give it a default
LICHEE_KDIR=${LICHEE_KDIR:-${CUR_DIR}/output/staging/usr/include}

DESTDIR=${CUR_DIR}/output/images
CROSS_COMPILE=arm-none-linux-gnueabi-
STAGING_DIR=${CUR_DIR}/output/staging
TARGET_DIR=${CUR_DIR}/output/target
TARGET_SYSROOT_OPT="--sysroot=${STAGING_DIR}"

TARGET_AR=${CROSS_COMPILE}ar
TARGET_AS=${CROSS_COMPILE}as
TARGET_CC="${CROSS_COMPILE}gcc ${TARGET_SYSROOT_OPT}"
TARGET_CPP="${CROSS_COMPILE}cpp ${TARGET_SYSROOT_OPT}"
TARGET_CXX="${CROSS_COMPILE}g++ ${TARGET_SYSROOT_OPT}"
TARGET_FC="${CROSS_COMPILE}gfortran ${TARGET_SYSROOT_OPT}"
TARGET_LD="${CROSS_COMPILE}ld ${TARGET_SYSROOT_OPT}"
TARGET_NM="${CROSS_COMPILE}nm"
TARGET_RANLIB="${CROSS_COMPILE}ranlib"
TARGET_OBJCOPY="${CROSS_COMPILE}objcopy"
TARGET_OBJDUMP="${CROSS_COMPILE}objdump"
TARGET_CFLAGS="-pipe -Os  -mtune=cortex-a8 -march=armv7-a -mabi=aapcs-linux -msoft-float -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -I${LICHEE_KDIR} "


BUILD_OPTIONS="DESTDIR=${DESTDIR} CROSS_COMPILE=${CROSS_COMPILE}  \
    STAGING_DIR=${STAGING_DIR} TARGET_DIR=${TARGET_DIR} CC=\"${TARGET_CC}\"  \
    AR=${TARGET_AR} AS=${TARGET_AS} CPP=\"${TARGET_CPP}\" CXX=\"${TARGET_CXX}\"  \
    FC=\"${TARGET_FC}\" LD=\"${TARGET_LD}\" NM=${TARGET_NM} RANLIB=${TARGET_RANLIB}  \
    OBJCOPY=${TARGET_OBJCOPY} OBJDUMP=${TARGET_OBJDUMP} CFLAGS=\"${TARGET_CFLAGS}\" "

build_external()
{
    for dir in $(ls ${EXTERNAL_DIR}); do
	echo ${EXTERNAL_DIR}/$dir
	if [ -f ${EXTERNAL_DIR}/$dir/Makefile ]; then
	    BUILD_COMMAND="make -C ${EXTERNAL_DIR}/$dir ${BUILD_OPTIONS} all"
	    eval $BUILD_COMMAND
	    BUILD_COMMAND="make -C ${EXTERNAL_DIR}/$dir ${BUILD_OPTIONS} install"
	    eval $BUILD_COMMAND
	    BUILD_COMMAND="pwd"
	fi
    done
}

case "$1" in
buildroot)
	build_buildroot
	;;
external)
	export PATH=${CUR_DIR}/output/external-toolchain/bin:$PATH
	#which arm-none-linux-gnueabi-gcc
	build_external
	;;
help)
	show_help
	;;
all)
	build_buildroot
	export PATH=${CUR_DIR}/output/external-toolchain/bin:$PATH
	build_external
	;;
*)
	build_buildroot
	;;
esac
