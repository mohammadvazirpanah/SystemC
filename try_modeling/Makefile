EXTRACFLAGS = 
EXTRA_LIBS = 
MODULE = run
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)

SYSTEMC = /home/mohammad/systemc-2.3.3
TARGET_ARCH = linux64

CC     = g++  
#CC     = clang++  
OPT    = -O0 -std=c++11
DEBUG  = -g

SYSDIR = -I $(SYSTEMC)/include
LIBDIR = -L. -L$(SYSTEMC)/lib-$(TARGET_ARCH)
INCDIR = -I. $(SYSDIR) 
RPATH = -rpath=/home/mohammad/systemc-2.3.3/lib-linux64

## Build with maximum gcc warning level
#CFLAGS = -Wall -Wno-unknown-pragmas -pthread $(DEBUG) $(OPT) $(EXTRACFLAGS)
CFLAGS = -Wall -pthread $(DEBUG) $(OPT) $(EXTRACFLAGS)
#CFLAGS = -arch i386 -Wall -Wno-deprecated -Wno-return-type -Wno-char-subscripts $(DEBUG) $(OPT) $(EXTRACFLAGS)

LIBS   =  -lstdc++ -lm $(EXTRA_LIBS) -lsystemc
EXE    = $(MODULE).x

.PHONY: clean 

#$(EXE): $(OBJS) $(SYSTEMC)/lib-$(TARGET_ARCH)/libsystemc.a 
$(EXE): $(OBJS)
	$(CC) $(CFLAGS) $(INCDIR) $(LIBDIR) -o $@ $(OBJS) $(LIBS) 2>&1 | c++filt


%.o : %.cpp
	$(CC) $(CFLAGS) $(INCDIR) -c -MMD -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d


clean:
	-rm -f $(OBJS) *~ $(EXE) *.vcd *.wif *.isdb *.dmp *.P *.log

-include $(SRCS:.cpp=.P)
