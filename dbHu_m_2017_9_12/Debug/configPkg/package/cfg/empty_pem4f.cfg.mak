# invoke SourceDir generated makefile for empty.pem4f
empty.pem4f: .libraries,empty.pem4f
.libraries,empty.pem4f: package/cfg/empty_pem4f.xdl
	$(MAKE) -f D:\doubi\desktop\dbMouse_2017\git_recode\dbHu_m_2017_9_12/src/makefile.libs

clean::
	$(MAKE) -f D:\doubi\desktop\dbMouse_2017\git_recode\dbHu_m_2017_9_12/src/makefile.libs clean

