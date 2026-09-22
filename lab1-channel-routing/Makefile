CXX := g++
TARGET := Lab1
SRCDIR := .
OBJDIR := ./obj
CXXFLAGS := -std=c++17 -g -static -Wall -O3
SRCS := $(notdir $(wildcard ${SRCDIR}/*.cpp))
OBJS := $(addprefix $(OBJDIR)/, $(patsubst %.cpp, %.o, $(SRCS)))

$(shell [ -d $(OBJDIR) ] || mkdir -p $(OBJDIR))

all: $(TARGET) $(OBJS)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -rf $(OBJDIR) $(TARGET)
