#include "objects.h"

bool Sphere::IntersectRay(const Ray &r, HitInfo &hInfo, int hitSide) const
{
	Ray r2 = hInfo.node->ToNodeCoords(r);

	// Ray in model space
	float a = (r2.dir).Dot(r2.dir);   // dir dot dir
	float b = 2.f*(r2.p.Dot(r2.dir)); // 2 * ray point dot dir
	float c = r2.p.Dot(r2.p) - 1.f;   // ray point dot ray point - r^2

																		// b^2 - 4ac
	float discriminant = (b * b) - (4 * a * c);
	if (discriminant >= 0.0) {
		float sqrtDis = sqrt(discriminant);
		float quotient = 1.f / (2.0f * a);
		float t1 = (-b + sqrtDis) * quotient;
		float t2 = (-b - sqrtDis) * quotient;

		// tf is how many model space rays needed to reach q. 
		float tf = (hitSide) ? max(t1, t2) : min(t1, t2);
		if (hInfo.z > tf)
			hInfo.z = tf;
		return true;
	}
	return false;
}