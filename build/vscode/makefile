#source file search paths
VPATH = ../../src 

# compiler include search paths
INCS = \
	-I../../../boost/boost1_72

# folder for .o files
ODIR=./obj

# sources
_OBJ = unirobot.o 
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.cpp
	g++   -std=c++17 -c -o $@ $< $(INCS)

unirobot: $(OBJ) 
	g++ -o ../../bin/unirobot.exe $^ 


.PHONY: clean

clean:
	del /q obj