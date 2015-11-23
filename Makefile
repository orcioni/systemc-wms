CXX = g++
AR  = ar
CFLAGS := -I include
include Makefile-local
CFLAGS += -O3
TARGET := lib/libawms.a
ifeq ($(HAVE_LAPACK),yes)
	CFLAGS += -DHAVE_LAPACK
endif

SRCS := src/analog_system.cpp src/wave_system.cpp
SRCS += src/tab_trace.cpp
SRCS += src/devices/sources.cpp
SRCS += src/devices/electromechanical.cpp
SRCS += src/devices/threephase.cpp
SRCS += src/devices/electrical_silicon.cpp

%.o : %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

%.d : %.cpp
	$(CXX) $(CFLAGS) -MM -MF $@ -MQ $(<:%.cpp=%.o) -MQ $@ $<

$(TARGET): $(SRCS:%.cpp=%.o)
	mkdir -p $(dir $@)
	$(AR) rc $@ $+

clean :
	rm -f $(SRCS:%.cpp=%.d) $(SRCS:%.cpp=%.o) $(TARGET)

-include $(SRCS:%.cpp=%.d)

