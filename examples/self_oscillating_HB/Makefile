include ../Makefile-local
CFLAGS += -O2 
LDLIBS += -lsystemc -llapack
TARGET := self
ifeq ($(HAVE_LAPACK),yes)
        LDLIBS += -llapack
endif
SRCS := self_oscillating_HB.cpp

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
