## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,m4fg linker.cmd package/cfg/hello_pm4fg.om4fg

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/hello_pm4fg.xdl
	$(SED) 's"^\"\(package/cfg/hello_pm4fgcfg.cmd\)\"$""\"D:/doubi/desktop/dbMouse_2017/git_recode/dbHu_m_2017_9_12/.config/xconfig_hello/\1\""' package/cfg/hello_pm4fg.xdl > $@
	-$(SETDATE) -r:max package/cfg/hello_pm4fg.h compiler.opt compiler.opt.defs
