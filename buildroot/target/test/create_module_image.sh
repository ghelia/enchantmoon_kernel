#!/bin/bash

BR_ROOT=`cd ../../; pwd`
LICHEE_ROOT=`cd $BR_ROOT/../; pwd`
LINUX_ROOT=`cd $LICHEE_ROOT/linux-3.0; pwd`

export PATH=$PATH:$BR_ROOT/target/tools/host/usr/bin


rm -rf modules/
mkdir modules


(cd $LINUX_ROOT/output/lib/modules; tar -c *)|gzip > modules/modules.tgz

NR_SIZE=`du -s modules|awk '{print $1}'`
NEW_NR_SIZE=0
TARGET_IMAGE=modules.ext2

#((NEW_NR_SIZE=$NR_SIZE+512))
((NEW_NR_SIZE=$NR_SIZE*12/10))


echo "blocks: $NR_SIZE -> $NEW_NR_SIZE"
genext2fs -d modules -b $NEW_NR_SIZE $TARGET_IMAGE

tune2fs -j -O extents,uninit_bg,dir_index $TARGET_IMAGE

fsck.ext4 -y $TARGET_IMAGE
echo $? > /dev/null
