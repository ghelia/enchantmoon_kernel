#!/bin/bash

LICHEE_ROOT=$PWD
PACK_ROOT=tools/pack
TARGET_CHIP="sun4i"
TARGET_PLATFORM="linux"
TARGET_BOARD="evb"
count=0

select_chips()
{
    count=0

    printf "All valid chips:\n"

    for chip in $(cd $PACK_ROOT/chips/; find -mindepth 1 -maxdepth 1 -type d |sort); do
        chips[$count]=`basename $PACK_ROOT/chips/$chip`
        printf "$count. ${chips[$count]}\n"
        let count=$count+1
    done

    while true; do
        read -p "Please select a chip:"
        RES=`expr match $REPLY "[0-9][0-9]*$"`
        if [ "$RES" -le 0 ]; then
            echo "please use index number"
            continue
        fi
        if [ "$REPLY" -ge $count ]; then
            echo "too big"
            continue
        fi
        if [ "$REPLY" -lt "0" ]; then
            echo "too small"
            continue
        fi
        break
    done

    TARGET_CHIP=${chips[$REPLY]}
}

select_platform()
{
    count=0
    chip=$1

    printf "All valid platforms:\n"

    for platform in $(cd $PACK_ROOT/chips/$chip/configs/; find -mindepth 1 -maxdepth 1 -type d |sort); do
        platforms[$count]=`basename $PACK_ROOT/chips/$chip/configs/$platform`
        printf "$count. ${platforms[$count]}\n"
        let count=$count+1
    done

    while true; do
        read -p "Please select a platform:"
        RES=`expr match $REPLY "[0-9][0-9]*$"`
        if [ "$RES" -le 0 ]; then
            echo "please use index number"
            continue
        fi
        if [ "$REPLY" -ge $count ]; then
            echo "too big"
            continue
        fi
        if [ "$REPLY" -lt "0" ]; then
            echo "too small"
            continue
        fi
        break
    done

    TARGET_PLATFORM=${platforms[$REPLY]}
}

select_boards()
{
    count=0
    chip=$1
    platform=$2

    printf "All valid boards:\n"

    for board in $(cd $PACK_ROOT/chips/$chip/configs/$platform/; find -mindepth 1 -maxdepth 1 -type d |grep -v default|sort); do
        boards[$count]=`basename $PACK_ROOT/chips/$chip/configs/$platform/$board`
        printf "$count. ${boards[$count]}\n"
        let count=$count+1
    done

    while true; do
        read -p "Please select a board:"
        RES=`expr match $REPLY "[0-9][0-9]*$"`
        if [ "$RES" -le 0 ]; then
            echo "please use index number"
            continue
        fi
        if [ "$REPLY" -ge $count ]; then
            echo "too big"
            continue
        fi
        if [ "$REPLY" -lt "0" ]; then
            echo "too small"
            continue
        fi
        break
    done

    TARGET_BOARD=${boards[$REPLY]}
}

printf "Start packing for Lichee system\n\n"

select_chips

select_platform $TARGET_CHIP

select_boards $TARGET_CHIP $TARGET_PLATFORM

echo "$TARGET_CHIP $TARGET_PLATFORM $TARGET_BOARD"

if [ "$TARGET_PLATFORM" = "crane" ]; then
    if [ -z "$CRANE_IMAGE_OUT" ]; then
        echo "You need to export CRANE_IMAGE_OUT var to env"
        exit 1
    fi

    if [ ! -f "$CRANE_IMAGE_OUT/system.img" ]; then
        echo "You have wrong CRANE_IMAGE_OUT env"
        exit 1
    fi
fi

cd $PACK_ROOT
./pack -c $TARGET_CHIP -p $TARGET_PLATFORM -b $TARGET_BOARD
cd -


