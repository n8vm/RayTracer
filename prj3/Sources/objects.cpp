#include "objects.h"

static float bias = .001;
bool Sphere::IntersectRay(const Ray &r, HitInfo &hInfo, int hitSide) const
{
	// Ray in model space
	float a = (r.dir).Dot(r.dir);   // dir dot dir
	float b = 2.f*(r.p.Dot(r.dir)); // 2 * ray point dot dir
	float c = r.p.Dot(r.p) - 1.f;   // ray point dot ray point - r^2

																		// b^2 - 4ac
	float discriminant = (b * b) - (4 * a * c);
	if (discriminant >= 0.0) {
		float sqrtDis = sqrt(discriminant);
		float quotient = 1.f / (2.0f * a);
		float t1 = (-b + sqrtDis) * quotient;
		float t2 = (-b - sqrtDis) * quotient;
		float tmax = max(t1, t2);
		float tmin = min(t1, t2);

		// tf is how many model space rays needed to reach q. 
		// Note that the model space ray is NOT normalized, so 
		// that tf will represent depth buffer info
		if (hitSide != 0) {
			if (hInfo.z > tmax && tmax > bias) {
				hInfo.z = tmax;
				hInfo.p = r.p + tmax * r.dir;
				hInfo.N = hInfo.p;
			}
		}
		if (hitSide != 1) {
			if (hInfo.z > tmin && tmin > bias) {
				hInfo.z = tmin;
				hInfo.p = r.p + tmin * r.dir;
				hInfo.N = hInfo.p;
			}
		}
		return true;
	}
	return false;
}