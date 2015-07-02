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

typedef std::pair< int, int > IntPair;

struct FramedTransformation {
	int id1_;
	int id2_;
	int frame_;
	Pose transformation_;
	FramedTransformation(int id1, int id2, int f, Pose t)
		: id1_(id1), id2_(id2), frame_(f), transformation_(t)
	{}
};

struct RGBDTrajectory {
	std::vector< FramedTransformation > data_;
	int index_;

	void LoadFromFile(std::string filename) {
		data_.clear();
		index_ = 0;
		int id1, id2, frame;
		FILE * f = fopen(filename.c_str(), "r");

		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					Pose vp;
					double tmp;
					double3 trans;
					double3x3 rot{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

					sscanf(buffer, "%d %d %d", &id1, &id2, &frame);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &rot.x.x, &rot.y.x, &rot.z.x, &trans.x);
					fgets(buffer, 1024, f);									
					sscanf(buffer, "%lf %lf %lf %lf", &rot.x.y, &rot.y.y, &rot.z.y, &trans.y);
					fgets(buffer, 1024, f);									
					sscanf(buffer, "%lf %lf %lf %lf", &rot.x.z, &rot.y.z, &rot.z.z, &trans.z);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &tmp, &tmp, &tmp, &tmp);
					vp.orientation.w = sqrt(1.0 + rot.x.x + rot.y.y + rot.z.z) / 2.0;
					auto w4 = (4.0 * vp.orientation.w);
					//vp.orientation.x = (rot.z.y - rot.y.z) / w4;
					//vp.orientation.y = (rot.x.z - rot.z.x) / w4;
					//vp.orientation.z = (rot.y.x - rot.x.y) / w4;
					vp.orientation.x = (rot.y.z - rot.z.y) / w4;
					vp.orientation.y = (rot.z.x - rot.x.z) / w4;
					vp.orientation.z = (rot.x.y - rot.y.x) / w4;
					vp.position = float3(trans.x,trans.y,trans.z);
					data_.push_back(FramedTransformation(id1, id2, frame, vp));
				}
			}
			fclose(f);
		}
	}
	//void SaveToFile(std::string filename) {
	//	FILE * f = fopen(filename.c_str(), "w");
	//	for (int i = 0; i < (int)data_.size(); i++) {
	//		Eigen::Matrix4d & trans = data_[i].transformation_;
	//		fprintf(f, "%d\t%d\t%d\n", data_[i].id1_, data_[i].id2_, data_[i].frame_);
	//		fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0, 0), trans(0, 1), trans(0, 2), trans(0, 3));
	//		fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
	//		fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
	//		fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
	//	}
	//	fclose(f);
	//}
};

float4x4 RigidTransformationMatrix(const float4 & rot, const float3 & vec)
{
	return{ { qxdir(rot), 0 }, { qydir(rot), 0 }, { qzdir(rot), 0 }, { vec, 1 } };
}
#include <fstream>
int main(int argc, char *argv[]) {

    time_t start, stop;
    time(&start);               // Start execution timer
    int samples = 1;            // Default samples per pixel

    if (argc == 2) samples = atoi(argv[1]);

    Scene scene = Scene();                                              // Create scene
	RGBDTrajectory traj;
	traj.LoadFromFile("choi-hotel_umd.txt");
	std::ifstream intrin("intrinsics.txt");
	float3x3 cm;
	float tmp;
	intrin >> cm.x.x >> cm.y.x >> cm.z.x;
	intrin >> cm.x.y >> cm.y.y >> cm.z.y;
	intrin >> cm.x.z >> cm.y.z >> cm.z.z;

	intrin.close();

	scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../obj/hotel_umd.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));

	for (int i = 100; i < traj.data_.size();i+=100) {
		auto pose = traj.data_[i];
		auto f44 = RigidTransformationMatrix(pose.transformation_.orientation, pose.transformation_.position);
		Camera camera = Camera(pose.transformation_, 640, 480, cm.x.x, cm.y.y, cm.z.x, cm.z.y);     // Create camera
		Renderer renderer = Renderer(&scene, &camera);  // Create renderer with our scene and camera
		renderer.render(samples);                       // Render image to pixel buffer
		renderer.save_image((std::string("render_") + std::to_string(i) + ".png").c_str());              // Save image
	}
	//Pose vp;
	//float3x3 rot{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
	//vp.position = { 0.0f, -5.0f, 2.5f };//{ 2.0f, 2.0f, -0.3f };
	////vp.orientation.w = sqrt(1.0 + rot.x.x + rot.y.y + rot.z.z) / 2.0;
	////auto w4 = (4.0 * vp.orientation.w);
	////vp.orientation.x = (rot.z.y - rot.y.z) / w4;
	////vp.orientation.y = (rot.x.z - rot.z.x) / w4;
	////vp.orientation.z = (rot.y.x - rot.x.y) / w4;
	//vp.orientation = RotationQuaternionFromToVec({ 0, 0, 1 }, float3( 0.0f, 0.0f, 1.0f ) - vp.position);
	//			Camera camera = Camera(vp, 1280, 720);     // Create camera
	//			//scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../obj/hotel_umd.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));
	//			scene.add(dynamic_cast<Object*>(new Sphere(Vec(0, 0, -1000), 1000, Material())));
	//			scene.add(dynamic_cast<Object*>(new Sphere(Vec(-1004, 0, 0), 1000, Material(DIFF, Vec(0.85, 0.4, 0.4)))));
	//			scene.add(dynamic_cast<Object*>(new Sphere(Vec(1004, 0, 0), 1000, Material(DIFF, Vec(0.4, 0.4, 0.85)))));
	//			scene.add(dynamic_cast<Object*>(new Sphere(Vec(0, 1006, 0), 1000, Material())));
	//			scene.add(dynamic_cast<Object*>(new Sphere(Vec(0, 0, 110), 100, Material(EMIT, Vec(1, 1, 1), Vec(2.2, 2.2, 2.2)))));
	//			scene.add(dynamic_cast<Object*>(new Mesh(Vec(), "../obj/dragon2.obj", Material(DIFF, Vec(0.9, 0.9, 0.9)))));
	//			Renderer renderer = Renderer(&scene, &camera);  // Create renderer with our scene and camera
	//			renderer.render(samples);                       // Render image to pixel buffer
	//			renderer.save_image("render.png");              // Save image

    // Print duration information
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff/3600;
    int mins = ((int)diff/60)-(hrs*60);
    int secs = (int)diff-(hrs*3600)-(mins*60);
    printf("\rRendering (%i samples): Complete!\nTime Taken: %i hrs, %i mins, %i secs\n\n", samples, hrs, mins, secs);
    return 0;
}