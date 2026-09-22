CXX := g++
CXXFLAGS := -O2 -g -std=c++14 -MMD -MP
LDFLAGS :=
SRCS := main.cpp router.cpp
OBJS := $(SRCS:.cpp=.o)
DEPS := $(OBJS:.o=.d)
TARGET := Lab3

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(RM) $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)
