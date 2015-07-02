/*
*  Simple monte carlo path tracer in C++
*
*  Compilation:
*      $ mkdir build
*      $ cd build
*      $ cmake ..
*      $ make
*
*  Usage:
*      $ ./pathtracer <number of samples>
*/

#include <stdio.h>
#include <stdlib.h>
#include "time.h"

#include "vector.h"
#include "material.h"
#include "objects.h"
#include "camera.h"
#include "scene.h"
#include "renderer.h"
#include "rgb_traj.h"

#include <fstream>
int main(int argc, char *argv[]) {

	time_t start, stop;
	time(&start);               // Start execution timer
	int samples = 1;            // Default samples per pixel

	if (argc == 2) samples = atoi(argv[1]);

	Scene scene = Scene();                                              // Create scene
	RGBDTrajectory traj;
	traj.LoadFromFile("../choi-hotel_umd.txt");
	std::ifstream intrin("../intrinsics.txt");
	float3x3 cm;
	float tmp;
	intrin >> cm.x.x >> cm.y.x >> cm.z.x;
	intrin >> cm.x.y >> cm.y.y >> cm.z.y;
	intrin >> cm.x.z >> cm.y.z >> cm.z.z;

	intrin.close();

	//scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../hotel_umd.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));

	for (int i = 0; i < 1 && i < traj.data_.size(); i += 100) {
		auto pose = traj.data_[i];
		Camera camera = Camera(pose.transformation_, 640, 480, cm.x.x, cm.y.y, cm.z.x, cm.z.y);     // Create camera
		Renderer renderer = Renderer(&scene, &camera);  // Create renderer with our scene and camera
		renderer.render(samples);                       // Render image to pixel buffer
		char sname[256];
		sprintf(sname, "render_%06d",i);
		auto fn = std::string(sname);
		renderer.save_image((fn + ".png").c_str());              // Save image
		renderer.save_depth((fn + ".raw").c_str());              // Save image
	}

	// Print duration information
	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);
	printf("\rRendering (%i samples): Complete!\nTime Taken: %i hrs, %i mins, %i secs\n\n", samples, hrs, mins, secs);
	return 0;
}