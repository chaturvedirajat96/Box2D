.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++ -pg
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs251_base

# Project Paths
PROJECT_ROOT=.
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
PROFDIR = $(PROJECT_ROOT)/profile
# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O2 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup doc clean distclean release report codeDoc profile

all: setup $(BINDIR)/$(TARGET)

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin

$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -O2 -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include -include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

codeDoc:
	@$(ECHO) -n "Doxygen Documentation of Group 19... "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null 2>&1
	@$(ECHO) "Done"

report:reportm
	@cd ./report;\
	pdflatex ProjectReport.tex > /dev/null 2>&1
	@rm -rf ./report/ProjectReport.aux
	@rm -rf ./reprt/ProjectReport.log
	@rm -rf ./report/ProjectReport.out

reportm:
	@cd ./report;\
	pdflatex Beamer.tex > /dev/null 2>&1
	@rm -rf ./report/Beamer.log
	@rm -rf ./report/Beamer.aux
	@rm -rf ./report/Beamer.out
	@rm -rf ./report/Beamer.nav
	@rm -rf ./report/Beamer.snm
	@rm -rf ./report/Beamer.toc

profile:
	@if [ ! -f "$(PROJECT_ROOT)/gmon.out" ] ; then \
		$(ECHO) "gmon.out not present" ;\
	fi
	@if [ -f "$(PROJECT_ROOT)/gmon.out" ] ; then
		mkdir -p profile
		gprof $(PROJECT_ROOT)/bin/cs251_base > $(PROFDIR)/Analysis.txt ;\
		python gprof2dot.py $(PROFDIR)/Analysis.txt > $(PROFDIR)/Analysis.dot ;\
		dot -Tpng $(PROFDIR)/Analysis.dot  -o $(PROFDIR)/Callgraph.png ;\
		$(ECHO) "Done" ;\
	fi

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) $(OBJDIR)/* $(SRCDIR)/*~  $(PROFDIR)/*
	@$(ECHO) "Done"
	@rm -rf ./report/ProjectReport.pdf
	@rm -rf ./report/Beamer.pdf

distclean: clean
	@$(RM) $(BINDIR) $(OBJDIR) $(DOCDIR)/html $(PROFDIR)

release:
