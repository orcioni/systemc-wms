include ../Makefile-local
CFLAGS += -O2
LDLIBS += -lsystemc 
TARGET := test
ifeq ($(HAVE_LAPACK),yes)
        LDLIBS += -llapack
endif

SRCS := transistor.cpp

%.o : %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

$(TARGET) : $(SRCS:%.cpp=%.o)
	$(CXX) -o $@ $+ $(LDLIBS)

Depends : $(SRCS)
	$(CXX) $(CFLAGS) -MM $+ > Depends

clean :
	rm -f Depends $(SRCS:%.cpp=%.o) $(TARGET)

Makefile : Depends

include Depends
