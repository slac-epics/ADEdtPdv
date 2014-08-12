TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The files in this directory are installed first because they are
# used by the EDTpdv driver
INC += bayer_filter_asm.h
INC += camconfighelp.h
INC += check_gap_lib.h
INC += chkbuf.h
INC += cl_logic_lib.h
INC += clsim_lib.h
INC += ConfigDataSet.h
INC += cpuid.h
INC += edt_bitload.h
INC += edtdef.h
INC += edt_devid.h
INC += edt_error.h
INC += edtinc.h
INC += edt_jpeg.h
INC += edt_net_msg.h
INC += edt_oc192.h
INC += edt_ocm.h
INC += edt_os_lnx.h
INC += edtregbits.h
INC += edtreg.h
INC += edt_si570.h
INC += edt_ss_vco.h
INC += edt_threads.h
INC += edt_trace.h
INC += edt_types.h
INC += edt_utils.h
INC += edt_vco.h
INC += initcam.h
INC += libdvu.h
INC += libedtdma.h
INC += libedt.h
INC += libedt_timing.h
INC += libpdv.h
INC += libsg.h
INC += lib_two_wire.h
INC += lib_xfp_sfp.h
INC += pciload.h
INC += pdv_dependent.h
INC += pdv_interlace.h
INC += pdv_interlace_methods.h
INC += pdv_irig.h
INC += pdv_recplay.h
INC += sdvlib.h

# Note, the following files were manually copied from the /opt/EDTpdv  directory
# to this directory after building
ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpdv.a

else ifeq (rhel6-x86_64, $(findstring rhel6-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/rhel6-x86_64/libpdv.a

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libpdv.a

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

