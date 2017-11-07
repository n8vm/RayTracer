#include "scene.h"

// Using Smits method
bool Box::IntersectRay(const Ray &r, float t_max) const {
	cyPoint3f dirfrac;
	dirfrac.x = 1.0f / r.dir.x;
	dirfrac.y = 1.0f / r.dir.y;
	dirfrac.z = 1.0f / r.dir.z;

	float t1 = (pmin.x - r.p.x) * dirfrac.x;
	float t2 = (pmax.x - r.p.x) * dirfrac.x;
	float t3 = (pmin.y - r.p.y) * dirfrac.y;
	float t4 = (pmax.y - r.p.y) * dirfrac.y;
	float t5 = (pmin.z - r.p.z) * dirfrac.z;
	float t6 = (pmax.z - r.p.z) * dirfrac.z;

	float tmin = MAX(MAX(MIN(t1, t2), MIN(t3, t4)), MIN(t5, t6));
	float tmax = MIN(MIN(MAX(t1, t2), MAX(t3, t4)), MAX(t5, t6));

	if (tmax < 0) // AABB is behind us
		return false;

	if (tmin > tmax) // The ray misses the box
		return false;

	return true;
}