include ../Makefile-local
CFLAGS += -O3
LDLIBS += -lsystemc
TARGET := hbtest
ifeq ($(HAVE_LAPACK),yes)
        LDLIBS += -llapack
endif


SRCS := HB.cpp

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
