CXX = g++
CFLAGS := -I ../include
LDLIBS := -L ../lib -l awms

include ../Makefile-local
CFLAGS += -O2 -fpermissive
LDLIBS += -lsystemc -llapack
TARGET := simula


SRCS := top_layer.cpp noise.cpp mtrand.cpp

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
