#!/bin/bash
###############################################################################
#
#                           Kernel Build Script 
#
###############################################################################
# 2011-10-24 effectivesky : modified
# 2010-12-29 allydrop     : created
# 2015-03-08 Le Hoang   : modified for custom source
###############################################################################
##############################################################################
# set toolchain
##############################################################################
# Don't edit ARCH
export ARCH=arm
#Patch to toolchain
#export PATH=$(pwd)/../../../../arm-eabi-4.6/bin:$PATH
export CROSS_COMPILE=~/android/arm-cortex_a15-linux-gnueabihf-linaro_4.9/bin/arm-cortex_a15-linux-gnueabihf-

#export CROSS_COMPILE=~/android/mokee/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-
#Export Host name and user build
#export KBUILD_BUILD_USER=hl
#export KBUILD_BUILD_HOST=HL-G480
##############################################################################
# make zImage
##############################################################################
while :
do
echo -n " Clean kernel before build? (y/n) "
read set
case $set in 
	y)	echo "Cleaning..."; rm -R ./obj; make clean && make mrproper; break;;

	n) 	echo "No clean, continue build"; break;;
	
	*)	echo "Invalid option";;
    esac
done
clear
mkdir -p ./obj/KERNEL_OBJ/
rm ./zImage
#make O=./obj/KERNEL_OBJ/
rm ./arch/arm/configs/temp_defconfig
cp ./arch/arm/configs/cm_ef52_defconfig ./arch/arm/configs/temp_defconfig
while :
do
echo "  1 - IM-A870K"
echo "  2 - IM-A870S"
echo "  3 - IM-A870L"
echo -n " Change your device you want to build kernel: "
read num
case $num in 
	1)	echo "CONFIG_MACH_APQ8064_EF52K=y" >> ./arch/arm/configs/temp_defconfig; break;;

	2) 	echo "CONFIG_MACH_APQ8064_EF52S=y" >> ./arch/arm/configs/temp_defconfig; break;;

	3) 	echo "CONFIG_MACH_APQ8064_EF52L=y" >> ./arch/arm/configs/temp_defconfig; break;;
	
	*)	echo "Invalid option";;
    esac
done
clear
make ARCH=arm O=./obj/KERNEL_OBJ/ temp_defconfig

make -j9 ARCH=arm O=./obj/KERNEL_OBJ/ 2>&1 | tee kernel_log.txt
make -j9 ARCH=arm INSTALL_MOD_PATH=$(pwd)/obj/KERNEL_OBJ O=./obj/KERNEL_OBJ/ modules_install
# Use make -j#
#Strip modules
    mkdir $(pwd)/obj/KERNEL_OBJ/modules
    mdpath=`find $(pwd)/obj/KERNEL_OBJ/lib/modules -type f -name modules.order`;
    if [ "$mdpath" != "" ];then
        mpath=`dirname $mdpath`;
        ko=`find $mpath/kernel -type f -name *.ko`;
        for i in $ko; do "$CROSS_COMPILE"strip --strip-unneeded $i;
        mv $i $(pwd)/obj/KERNEL_OBJ/modules/; done;
    fi

##############################################################################
# Copy Kernel Image
##############################################################################

cp -f ./obj/KERNEL_OBJ/arch/arm/boot/zImage .
if [  -e zImage ]
then
  echo "Build succed /$(pwd)/zImage"
  exit 0
fi
