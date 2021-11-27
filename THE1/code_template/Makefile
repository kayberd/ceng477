all:
	g++ *.cpp -o raytracer -std=c++11 -O3
run:
	rm -f test.ppm && ./raytracer ../sample_scenes_v4/inputs/$(input).xml && sleep 1 && eog test.ppm &
clean:
	rm -f *.o raytracer
