CXXFLAGS =	-O3 -fopenmp -Wall -lopencv_core -lopencv_imgproc -lopencv_highgui

OBJS =	renderspace.o SceneDescription.o Raytracer.o Material.o KdTree.o MemManager.o

LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lm -fopenmp

TARGET =	renderspace

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)