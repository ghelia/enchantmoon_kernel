#!/bin/bash

##################################################################################################
# 
# Demonstrates auto build & pack for android under linux
#
# Resume the directory arch:
#   -android2.3.4
#   -lichee+-linux-2.6.36
#   |      |-buildroot
#   |      `-build.sh 
#   `-build_android.sh
#
#                                                  - Benn Huang(benn@allwinnertech.com)
###################################################################################################


printf "Start building android firmware\n"

TOP_ROOT=$PWD
LICHEE_ROOT=$PWD/lichee
CRANE_ROOT=$PWD/android2.3.4

function do_config()
{
    cd $CRANE_ROOT
    source build/envsetup.sh
    lunch
    cd $TOP_ROOT
}

function do_build_lichee()
{
    cd $LICHEE_ROOT
    ./build.sh -p sun4i_crane
    cd $TOP_ROOT
}

function do_build_crane()
{
    cd $CRANE_ROOT
    make update-api
    make -j8
    mkimg
    cd $TOP_ROOT
}

function do_pack()
{
    rm -rf $LICHEE_ROOT/buildroot/tools/pack/wboot/android/*.fex
    ln -sv ${OUT}/images/root.img  $LICHEE_ROOT/buildroot/tools/pack/wboot/android/root.fex
    ln -sv ${OUT}/images/system.img  $LICHEE_ROOT/buildroot/tools/pack/wboot/android/system.fex
    ln -sv ${OUT}/images/recovery.img  $LICHEE_ROOT/buildroot/tools/pack/wboot/android/recovery.fex
    cd $LICHEE_ROOT/buildroot/tools/pack

    case $TARGET_PRODUCT in
    crane_evb_v13)
        ./pack-new -c sun4i -p crane -b evb-v13
        ;;
    crane_evb)
        ./pack-new -c sun4i -p crane -b evb
        ;;
    crane_aino)
        ./pack-new -c sun4i -p crane -b aino
        ;;
    crane_h5)
        ./pack-new -c sun4i -p crane -b onda_n507h5
        ;;
    crane_h6)
        ./pack-new -c sun4i -p crane -b onda_m702h6
        ;;
    crane_t780)
        ./pack-new -c sun4i -p crane -b t780
        ;;
    *)
        echo "Skip pack, please pack manually"
        ;;
    esac
    
    cd $TOP_ROOT
}


do_config
do_build_lichee
do_build_crane
do_pack



