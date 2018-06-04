#!/bin/bash
### DO NOT EDIT THIS FILE ###
app_build_root=`pwd`
echo ${app_build_root}

#INIT_TARGET_PRODUCT=$1
#FINAL_TARGET_PRODUCT=$1
INIT_TARGET_PRODUCT=NX563J
FINAL_TARGET_PRODUCT=NX563J

#KERNEL_PERF_DEFCONFIG=$2
KERNEL_PERF_DEFCONFIG=perf

export TARGET_NUBIA_BUILD_TYPE=release
#export

function usage()
{
cat<<EOF

[01;32m===========================================================[0m
[0m[01;31mFast build kernel[0m
[0m[01;31mUsage:[0m
	1. ./zte_build/fastbuild_kernel.sh NX563J
	1. ./zte_build/fastbuild_kernel.sh NX563J perf
	2. ./zte_build/fastbuild_kernel.sh NX595J
	2. ./zte_build/fastbuild_kernel.sh NX595J perf
	2. ./zte_build/fastbuild_kernel.sh NX610J
	2. ./zte_build/fastbuild_kernel.sh NX610J perf
[01;32m===========================================================[0m

EOF
}

#if [ ! -d "zte_build" ]
#then
#	echo
#	echo -e "\033[01;31mPlease call me in android root dir!!!\033[0m"
#	echo
#	exit 1
#fi

#if [ $# -lt 1 ]
#then
#	usage
#	exit 1
#fi

################################################################################################
echo -e "\033[01;32mINIT_TARGET_PRODUCT=${INIT_TARGET_PRODUCT}\033[0m"


if [ "${INIT_TARGET_PRODUCT}" == "NX563J" ]
then
	FINAL_TARGET_PRODUCT=NX563J
fi

if [ "${INIT_TARGET_PRODUCT}" == "NX595J" ]
then
	FINAL_TARGET_PRODUCT=NX595J
fi

if [ "${INIT_TARGET_PRODUCT}" == "NX610J" ]
then
	FINAL_TARGET_PRODUCT=NX610J
fi
echo -e "\033[01;32mFINAL_TARGET_PRODUCT=${FINAL_TARGET_PRODUCT}\033[0m"
################################################################################################


if [ ! -d "out/target/product/${FINAL_TARGET_PRODUCT}/root" ]
then
	echo
	echo -e "\033[01;31mPlease First Full build kernel!!!\033[0m"
	echo -e "\033[01;31m./build.sh ${INIT_TARGET_PRODUCT} kernel\033[0m"
	echo
	exit 1
fi


################################################################################################
KERNEL_MODULES_OUT=${app_build_root}/out/target/product/${FINAL_TARGET_PRODUCT}/system/lib/modules
function mv-modules()
{
	mdpath=`find ${KERNEL_MODULES_OUT} -type f -name modules.dep`
	if [ "$mdpath" != "" ];then
		mpath=`dirname $mdpath`
		echo mpath=$mpath
		ko=`find $mpath/kernel -type f -name *.ko`
		for i in $ko
		do
			mv $i ${KERNEL_MODULES_OUT}/
		done
	fi
}

function clean-module-folder()
{
	mdpath=`find ${KERNEL_MODULES_OUT} -type f -name modules.dep`
	if [ "$mdpath" != "" ];then
		mpath=`dirname $mdpath`
		rm -rf $mpath
	fi
}
################################################################################################


################################################################################################
export ZTEMT_DTS_NAME="msm8998-v2.1-mtp-${FINAL_TARGET_PRODUCT}.dtb msm8998-v2-mtp-${FINAL_TARGET_PRODUCT}.dtb msm8998-mtp-${FINAL_TARGET_PRODUCT}.dtb"
echo -e "\033[01;32mZTEMT_DTS_NAME = ${ZTEMT_DTS_NAME}\033[0m"

if [ "${KERNEL_PERF_DEFCONFIG}" == "perf" ]
then
	KERNEL_DEFCONFIG=msmcortex-perf-${FINAL_TARGET_PRODUCT}_defconfig
else
	KERNEL_DEFCONFIG=msmcortex-${FINAL_TARGET_PRODUCT}_defconfig
fi
echo -e "\033[01;32mKERNEL_DEFCONFIG = ${KERNEL_DEFCONFIG}\033[0m"
################################################################################################


echo -e "\033[01;32m======================config build kernel environment value============================\033[0m"
export  PATH=${app_build_root}/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin:${app_build_root}/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin:$PATH
export
echo -e "\033[01;32m=======================================================================================\033[0m"


echo -e "\033[01;32mdelete Image.gz-dtb...\033[0m"
rm -rf out/target/product/${FINAL_TARGET_PRODUCT}/obj/KERNEL_OBJ/arch/arm64/boot


#echo -e "\033[01;32mkernelclean...\033[0m"
#make -C kernel O=../out/target/product/${FINAL_TARGET_PRODUCT}/obj/KERNEL_OBJ ARCH=arm64 CROSS_COMPILE=arm-eabi-  clean
#make -C kernel O=../out/target/product/${FINAL_TARGET_PRODUCT}/obj/KERNEL_OBJ ARCH=arm64 CROSS_COMPILE=arm-eabi-  mrproper
#make -C kernel O=../out/target/product/${FINAL_TARGET_PRODUCT}/obj/KERNEL_OBJ ARCH=arm64 CROSS_COMPILE=arm-eabi-  distclean


echo -e "\033[01;32mBuilding .config...\033[0m"
make -C kernel/msm-4.4 O=../../out/target/product/${FINAL_TARGET_PRODUCT}/obj/kernel/msm-4.4 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- ${KERNEL_DEFCONFIG}


################################################################################################
echo -e "\033[01;32mBuilding kernel...\033[0m"
make -C kernel/msm-4.4 O=../../out/target/product/${FINAL_TARGET_PRODUCT}/obj/kernel/msm-4.4 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- KCFLAGS=-mno-android  -j`grep processor /proc/cpuinfo |wc -l`
RET_VAL=$?
make -C kernel/msm-4.4 O=../../out/target/product/${FINAL_TARGET_PRODUCT}/obj/kernel/msm-4.4 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- KCFLAGS=-mno-android modules
make -C kernel/msm-4.4 O=../../out/target/product/${FINAL_TARGET_PRODUCT}/obj/kernel/msm-4.4 INSTALL_MOD_PATH=../../../system INSTALL_MOD_STRIP=1 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- modules_install

echo -e "\033[01;32mmv-modules clean-module-folder...\033[0m"
mv-modules
clean-module-folder

if [ $RET_VAL -gt 0 ]
then
	echo -e "\033[01;32m***********************************************************\033[0m"
	echo -e "\033[01;31m         Build error!!! Please see build log above         \033[0m"
	echo -e "\033[01;32m***********************************************************\033[0m"
	exit $RET_VAL
fi
################################################################################################


echo -e "\033[01;32mcp -rf Image.gz-dtb kernel...\033[0m"
cp -rf out/target/product/${FINAL_TARGET_PRODUCT}/obj/kernel/msm-4.4/arch/arm64/boot/Image.gz-dtb out/target/product/${FINAL_TARGET_PRODUCT}/kernel


echo -e "\033[01;32mmkbootfs ramdisk.img...\033[0m"
out/host/linux-x86/bin/mkbootfs -d out/target/product/${FINAL_TARGET_PRODUCT}/system out/target/product/${FINAL_TARGET_PRODUCT}/root | out/host/linux-x86/bin/minigzip > out/target/product/${FINAL_TARGET_PRODUCT}/ramdisk.img
#gunzip -c ramdisk.img | cpio -i


################################################################################################
echo -e "\033[01;32mmkbootimg boot.img...\033[0m"
#BOARD_KERNEL_CMDLINE=`grep "BOARD_KERNEL_CMDLINE :="  device/zte/common/BoardConfig.mk`
#if [ "$BOARD_KERNEL_CMDLINE" == "" ];then
#   	echo Error:BOARD_KERNEL_CMDLINE not found
#   	exit 1
#fi
#KERNEL_CMDLINE=`echo ${BOARD_KERNEL_CMDLINE#*:=}`
#echo KERNEL_CMDLINE=$KERNEL_CMDLINE
out/host/linux-x86/bin/mkbootimg --kernel out/target/product/${FINAL_TARGET_PRODUCT}/kernel --ramdisk out/target/product/${FINAL_TARGET_PRODUCT}/ramdisk.img --base 0x00000000 --pagesize 4096 --cmdline "console=ttyMSM0,115200,n8 androidboot.console=ttyMSM0 earlycon=msm_serial_dm,0xc1b0000 androidboot.hardware=qcom user_debug=31 msm_rtb.filter=0x237 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 sched_enable_hmp=1 sched_enable_power_aware=1 service_locator.enable=1 swiotlb=2048 androidboot.configfs=true androidboot.usbcontroller=a800000.dwc3 buildvariant=eng" --os_version 7.1.1 --os_patch_level 2017-02-05 --output out/target/product/${FINAL_TARGET_PRODUCT}/boot.img

#out/host/linux-x86/bin/mkbootimg  --kernel out/target/product/${FINAL_TARGET_PRODUCT}/kernel --ramdisk out/target/product/${FINAL_TARGET_PRODUCT}/ramdisk.img --cmdline "$KERNEL_CMDLINE" --base 0x00000000 --pagesize 4096  --output out/target/product/${FINAL_TARGET_PRODUCT}/boot.img
################################################################################################


echo -e "\033[01;32mboot_signer boot.img...\033[0m"
out/host/linux-x86/bin/boot_signer /boot out/target/product/${FINAL_TARGET_PRODUCT}/boot.img build/target/product/security/verity.pk8 build/target/product/security/verity.x509.pem out/target/product/${FINAL_TARGET_PRODUCT}/boot.img


echo "[01;32m==================================================================[0m"
echo "Now, the boot.img images are in dir:"
echo -e "\033[01;32m${app_build_root}/out/target/product/${FINAL_TARGET_PRODUCT}/boot.img\033[0m"
echo "[01;32m==================================================================[0m"


echo "=============================================="
echo          Build finished at `date`.
echo "=============================================="


### DO NOT EDIT THIS FILE ###
