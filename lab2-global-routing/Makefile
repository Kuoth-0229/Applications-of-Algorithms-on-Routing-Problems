CXX := g++
CXXFLAGS := -std=c++14 -O3 -I. -MMD -MP -g
LDFLAGS := 
SRCS := $(shell find . -name '*.cpp')
OBJS := $(SRCS:.cpp=.o)
DEPS := $(OBJS:.o=.d)
TARGET := router

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.cpp.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(RM) $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)
