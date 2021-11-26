all:
	g++ *.cpp -o raytracer -std=c++11
run:
	rm -f test.ppm && ./raytracer ../sample_scenes_v4/inputs/simple.xml && eog test.ppm &
clean:
	rm -f *.o raytracer
