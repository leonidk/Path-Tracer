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


int main(int argc, char *argv[]) {

    time_t start, stop;
    time(&start);               // Start execution timer
    int samples = 1;            // Default samples per pixel

    if (argc == 2) samples = atoi(argv[1]);

	//Vec pose[] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    Scene scene = Scene();                                              // Create scene

    // Add objects to scene



	Pose vp;
	float3x3 rot{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
	vp.position = { 0.0f, -5.0f, 2.5f };//{ 2.0f, 2.0f, -0.3f };
	vp.orientation.w = sqrt(1.0 + rot.x.x + rot.y.y + rot.z.z) / 2.0;
	auto w4 = (4.0 * vp.orientation.w);
	vp.orientation.x = (rot.z.y - rot.y.z) / w4;
	vp.orientation.y = (rot.x.z - rot.z.x) / w4;
	vp.orientation.z = (rot.y.x - rot.x.y) / w4;

				Camera camera = Camera(vp, 1280, 720);     // Create camera
				//scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../obj/hotel_umd.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));
				scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,0,-1000), 1000, Material())) );
				scene.add( dynamic_cast<Object*>(new Sphere(Vec(-1004,0,0), 1000, Material(DIFF, Vec(0.85,0.4,0.4)))) );
				scene.add( dynamic_cast<Object*>(new Sphere(Vec(1004,0,0), 1000, Material(DIFF, Vec(0.4,0.4,0.85)))) );
				scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,1006,0), 1000, Material())) );
				scene.add( dynamic_cast<Object*>(new Sphere(Vec(0,0,110), 100, Material(EMIT, Vec(1,1,1), Vec(2.2,2.2,2.2)))) );
				scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../obj/dragon2.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));
				Renderer renderer = Renderer(&scene, &camera);  // Create renderer with our scene and camera
				renderer.render(samples);                       // Render image to pixel buffer
				renderer.save_image("render.png");              // Save image

    // Print duration information
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff/3600;
    int mins = ((int)diff/60)-(hrs*60);
    int secs = (int)diff-(hrs*3600)-(mins*60);
    printf("\rRendering (%i samples): Complete!\nTime Taken: %i hrs, %i mins, %i secs\n\n", samples, hrs, mins, secs);
    return 0;
}