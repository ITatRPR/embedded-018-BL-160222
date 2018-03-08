#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED="../../Source/MDD File System/FSIO.c" ../../Source/NVMem.c ../../Source/SD/SD-SPI.c ../../Source/SD/sd_bootloader.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1382283020/FSIO.o ${OBJECTDIR}/_ext/1787047461/NVMem.o ${OBJECTDIR}/_ext/1811228923/SD-SPI.o ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1382283020/FSIO.o.d ${OBJECTDIR}/_ext/1787047461/NVMem.o.d ${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1382283020/FSIO.o ${OBJECTDIR}/_ext/1787047461/NVMem.o ${OBJECTDIR}/_ext/1811228923/SD-SPI.o ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o

# Source Files
SOURCEFILES=../../Source/MDD File System/FSIO.c ../../Source/NVMem.c ../../Source/SD/SD-SPI.c ../../Source/SD/sd_bootloader.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX340F512H
MP_LINKER_FILE_OPTION=,--script="..\..\linker_scripts\others\btl_32MX340F512H_generic.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1382283020/FSIO.o: ../../Source/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1382283020 
	@${RM} ${OBJECTDIR}/_ext/1382283020/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1382283020/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1382283020/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1382283020/FSIO.o.d" -o ${OBJECTDIR}/_ext/1382283020/FSIO.o "../../Source/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/1787047461/NVMem.o: ../../Source/NVMem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1787047461 
	@${RM} ${OBJECTDIR}/_ext/1787047461/NVMem.o.d 
	@${RM} ${OBJECTDIR}/_ext/1787047461/NVMem.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1787047461/NVMem.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1787047461/NVMem.o.d" -o ${OBJECTDIR}/_ext/1787047461/NVMem.o ../../Source/NVMem.c   
	
${OBJECTDIR}/_ext/1811228923/SD-SPI.o: ../../Source/SD/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1811228923 
	@${RM} ${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1811228923/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/1811228923/SD-SPI.o ../../Source/SD/SD-SPI.c   
	
${OBJECTDIR}/_ext/1811228923/sd_bootloader.o: ../../Source/SD/sd_bootloader.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1811228923 
	@${RM} ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d" -o ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o ../../Source/SD/sd_bootloader.c   
	
else
${OBJECTDIR}/_ext/1382283020/FSIO.o: ../../Source/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1382283020 
	@${RM} ${OBJECTDIR}/_ext/1382283020/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1382283020/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1382283020/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1382283020/FSIO.o.d" -o ${OBJECTDIR}/_ext/1382283020/FSIO.o "../../Source/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/1787047461/NVMem.o: ../../Source/NVMem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1787047461 
	@${RM} ${OBJECTDIR}/_ext/1787047461/NVMem.o.d 
	@${RM} ${OBJECTDIR}/_ext/1787047461/NVMem.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1787047461/NVMem.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1787047461/NVMem.o.d" -o ${OBJECTDIR}/_ext/1787047461/NVMem.o ../../Source/NVMem.c   
	
${OBJECTDIR}/_ext/1811228923/SD-SPI.o: ../../Source/SD/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1811228923 
	@${RM} ${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1811228923/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1811228923/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/1811228923/SD-SPI.o ../../Source/SD/SD-SPI.c   
	
${OBJECTDIR}/_ext/1811228923/sd_bootloader.o: ../../Source/SD/sd_bootloader.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1811228923 
	@${RM} ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d 
	@${RM} ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -DTRANSPORT_LAYER_SD_CARD -DDEMO_BOARD_EXPLORER_16 -DCMP_REF_SRC_CVREF -I"../../MPLAB_Workspace" -I"../.." -I"../../Include" -I"../../Include/MDD File System" -I"../../Include/HardwareProfile" -I"../../Include/Usb/Stack" -I"../../Include/FrameWork" -MMD -MF "${OBJECTDIR}/_ext/1811228923/sd_bootloader.o.d" -o ${OBJECTDIR}/_ext/1811228923/sd_bootloader.o ../../Source/SD/sd_bootloader.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../../linker_scripts/others/btl_32MX340F512H_generic.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)    -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}              -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../../linker_scripts/others/btl_32MX340F512H_generic.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/SD_Card_Btl_SpaceLogger.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
