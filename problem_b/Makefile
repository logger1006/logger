
CC	:= gcc
CXX	:= g++
LD	:= g++
CP	:= cp

PROG	:= cell_move_router

MODULES	:= src/parser src/main src/rcm src/router src/flute

default: $(PROG)

OPTFLAGS	:= -g -O3 -std=c++11 

CFLAGS   += $(OPTFLAGS) $(patsubst %, -I%, $(MODULES))
CXXFLAGS += $(CFLAGS)

LIBS := -ldl
SRC  :=
GARBAGE := core core.* *.stackdump ./tags $(PROG)

.PHONY: tags clean docs

include $(patsubst %, %/module.make, $(MODULES))

OBJ := \
        $(patsubst %.cpp, %.o, $(filter %.cpp, $(SRC))) \
        $(patsubst %.c, %.o,  $(filter %.c, $(SRC)))  \
        $(patsubst %.y, %.o,  $(filter %.y, $(SRC)))

DEP := $(OBJ:.o=.d)

# implicit rules

%.d: %.c
	./depends.sh $(CC) `dirname $*.c` $(CFLAGS) $*.c > $@

%.d: %.cpp
	./depends.sh $(CXX) `dirname $*.cpp` $(CXXFLAGS) $(CFLAGS) $*.cpp > $@

-include $(DEP)
	# Actual targets

depend: $(DEP)

clean:
	rm -rf $(PROG) $(OBJ) $(GARBAGE) $(OBJ:.o=.d)

tags:
	ctags -R .

$(PROG): $(OBJ) 
	$(LD) -o $@ $^ $(LIBS) 

lib$(PROG).a: $(OBJ)
	ar rv $@ $?
	ranlib $@

docs:
	doxygen doxygen.conf

