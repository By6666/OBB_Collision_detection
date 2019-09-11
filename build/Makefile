exe: main.o obb.o opencvdraw.o
	g++ -o exe.out main.o obb.o opencvdraw.o $(shell pkg-config --cflags --libs opencv)

main.o: ../main.cpp
	g++ -c ../main.cpp -std=c++11

obb.o: ../include/obb.cpp
	g++ -c ../include/obb.cpp -std=c++11

opencvdraw.o: ../include/opencvdraw.cpp
	g++ -c ../include/opencvdraw.cpp -std=c++11

clean:
	rm -f *.o