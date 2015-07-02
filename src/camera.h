#ifndef CAMERA_H
#define CAMERA_H

#include "../lib/rand48/erand48.h"
#include "vector.h"
#include "ray.h"
#include "linalg.h"

struct Pose
{
	float3 position;
	float4 orientation; // Represented as a unit-length quaternion

	Pose() : orientation(0, 0, 0, 1) {}
	Pose(const float3 & position, const float4 & orientation) : position(position), orientation(orientation) {}

	float3 GetXDir() const { return qxdir(orientation); }
	float3 GetYDir() const { return qydir(orientation); }
	float3 GetZDir() const { return qzdir(orientation); }

	float3 TransformPoint(const float3 & point) const { return position + TransformDirection(point); }
	float3 TransformDirection(const float3 & direction) const { return qrot(orientation, direction); }
};


struct SRay
{
	float3 origin;
	float3 direction; // Must be normalized
};
inline Pose operator * (const Pose & a, const Pose & b) { return{ a.TransformPoint(b.position), qmul(a.orientation, b.orientation) }; }
inline SRay operator * (const Pose & pose, const SRay & ray) { return{ pose.TransformPoint(ray.origin), pose.TransformDirection(ray.direction) }; }

class Camera {

private:
    int m_width;
    int m_height;
	Pose viewPose;
	float fx, fy, px, py;
public:
	Camera(Pose& pose, int width, int height);

    int get_width();
    int get_height();
    Ray get_ray(int x, int y, bool jitter, unsigned short *Xi);

};

#endif //CAMERA_H