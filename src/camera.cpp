#include "ray.h"
#include "camera.h"
#include "linalg.h"
#include <random>
static std::default_random_engine generator;
static std::uniform_real_distribution<double> distr(0.0, 1.0);
static double erand48m(int X=0){
	return distr(generator);
}
Camera::Camera(Pose& pose, int width, int height,float fx,float fy,float px,float py)
: px(px),py(py),fx(fx),fy(fy) {
	m_width = width;
	m_height = height;
	viewPose = pose;

}
Camera::Camera(Pose& pose, int width, int height) {
	m_width = width;
	m_height = height;

	fx = width / 2;
	fy = width / 2;
	px = width / 2;
	py = height / 2;
	viewPose = pose;

}
int Camera::get_width() { return m_width; }
int Camera::get_height() { return m_height; }

// Returns ray from camera origin through pixel at x,y
Ray Camera::get_ray(int x, int y, bool jitter, unsigned short *Xi) {

    double x_jitter;
    double y_jitter;

    // If jitter == true, jitter point for anti-aliasing
    if (jitter) {
        x_jitter = erand48m()-0.5f;
		y_jitter = erand48m()-0.5f;

    }
    else {
        x_jitter = 0;
        y_jitter = 0;
    }
	auto viewDirection = linalg::normalize(float3((x - px + x_jitter) / fx, (y - py + y_jitter) / fy, 1));
	auto nr = viewPose * SRay{ { 0, 0, 0 }, viewDirection };

    //Vec pixel = m_position + m_direction*2;
    //pixel = pixel - m_x_direction*m_ratio + m_x_direction*((x * 2 * m_ratio)*m_width_recp) + x_jitter;
    //pixel = pixel + m_y_direction - m_y_direction*((y * 2.0)*m_height_recp + y_jitter);

	return Ray(Vec(nr.origin.x, nr.origin.y, nr.origin.z), Vec(nr.direction.x, nr.direction.y, nr.direction.z).norm());
}