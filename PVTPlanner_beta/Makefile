include Makefile.config

ifeq ($(UNAME), Darwin)
	JAVA_INC_DIR= $(MAC_JAVA_INC_DIR)
endif
ifeq ($(UNAME), Linux)
	JAVA_INC_DIR = $(LINUX_JAVA_INC_DIR)
endif

JAVAINCDIR=
INCDIR=

ifeq ($(BUILD_TYPE), 1)

	ifeq ($(BUILD_JNI), 1)
		JAVAINCDIR= /System/Library/Frameworks/JavaVM.framework/Headers
	endif
	
	INCDIR= $(LOCALINC) $(JAVAINCDIR)

	ifeq ($(BUILD_GSL), 1)
		INCDIR:= $(INCDIR) $(GSLINC)
	endif
	
	ifeq ($(BUILD_BOOST), 1)
		INCDIR:= $(INCDIR) $(BOOSTINC)
	endif

	include Makefile.shared

else

	INCDIR= $(LOCALINC)

	ifeq ($(BUILD_GSL), 1)
		INCDIR:= $(INCDIR) $(GSLINC)
	endif
	
	ifeq ($(BUILD_BOOST), 1)
		INCDIR:= $(INCDIR) $(BOOSTINC)
	endif

	include Makefile.static

endif