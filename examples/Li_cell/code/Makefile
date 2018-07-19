CXX = g++

CFLAGS := -I ../../../include
LDLIBS := -L ../../../lib -l awms -framework Accelerate
LDLIBS += -L/usr/local/systemc/lib-macosx64
CFLAGS += -I/usr/local/systemc/include
HAVE_LAPACK=yes

CFLAGS += -O2 -std=c++11 -I/opt/local/include
LDLIBS += -lsystemc -llapack -larmadillo -L /opt/local/lib
TARGET := batt

SRCS := battery_S_of_P.cpp interpol.cpp cell.cpp

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
