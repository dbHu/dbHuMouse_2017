# invoke SourceDir generated makefile for hello.pem4f
hello.pem4f: .libraries,hello.pem4f
.libraries,hello.pem4f: package/cfg/hello_pem4f.xdl
	$(MAKE) -f D:\doubi\desktop\dbMouse_2017\git_recode\dbHu_m_2017_9_12/src/makefile.libs

clean::
	$(MAKE) -f D:\doubi\desktop\dbMouse_2017\git_recode\dbHu_m_2017_9_12/src/makefile.libs clean

