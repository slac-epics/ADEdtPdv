T_A=$(EPICS_HOST_ARCH)
TOP=../..
include $(TOP)/configure/CONFIG

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#----------------------------------------------------

# Hamamatsu Orca Flash 4.0 Scientific CMOS Camera
MODELS += hamaOrcaFlash4_0

# Uniq UP900
MODELS += up900

# Expand to install all needed files
# Symbolic links can be used if desired for shared files, but
# each model installed get's its own copy of each file installed
DB_INSTALLS += $(addsuffix  .db,    $(MODELS))
DB_INSTALLS += $(addsuffix f.cfg,   $(MODELS))
DB_INSTALLS += $(addsuffix p.cfg,   $(MODELS))
DB_INSTALLS += $(addsuffix t.cfg,   $(MODELS))
DB_INSTALLS += $(addsuffix  .env,   $(MODELS))
DB_INSTALLS += $(addsuffix  .proto, $(MODELS))

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
