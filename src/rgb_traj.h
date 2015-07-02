#pragma once
#include <vector>
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
					vp.position = float3(trans.x, trans.y, trans.z);
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