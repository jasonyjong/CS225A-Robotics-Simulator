linux:
	make -f Makefile.linux
qnx:
	make -f Makefile.qnx
clean:
	/bin/rm -fr *.a *.o core core.*
clean-all: clean

INCLUDEDIR= -I. -I../utility
SRCS= PrMatrix.cpp PrMatrix3.cpp PrMatrix6.cpp PrVector.cpp PrVector3.cpp \
	PrVector6.cpp PrQuaternion.cpp PrTransform.cpp
OBJS= $(SRCS:.cpp=.o)

%.o : %.cpp *.h
	$(CXX) $(CXXFLAGS) $(INCLUDEDIR) -c -o $@ $< $(DXX)
