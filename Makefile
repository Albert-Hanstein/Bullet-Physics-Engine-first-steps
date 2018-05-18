# GNU Make solution makefile autogenerated by Premake
# Type "make help" for usage help

ifndef config
  config=release
endif
export config

PROJECTS := demo

.PHONY: all clean help $(PROJECTS)

all: $(PROJECTS)

demo: 
	@echo "==== Building demo ($(config)) ===="
	@${MAKE} --no-print-directory -C build/windows.gmake/demo -f Makefile

clean:
	@${MAKE} --no-print-directory -C build/windows.gmake/demo -f Makefile clean

help:
	@echo "Usage: make [config=name] [target]"
	@echo ""
	@echo "CONFIGURATIONS:"
	@echo "   release"
	@echo ""
	@echo "TARGETS:"
	@echo "   all (default)"
	@echo "   clean"
	@echo "   demo"
	@echo ""
	@echo "For more information, see http://industriousone.com/premake/quick-start"
