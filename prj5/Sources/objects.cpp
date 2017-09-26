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
		if (hitSide & HIT_BACK) {
			if (hInfo.z > tmax && tmax > bias) {
				hInfo.z = tmax;
				hInfo.p = r.p + tmax * r.dir;
				hInfo.N = hInfo.p;
				hInfo.front = false;
			}
		}
		if (hitSide & HIT_FRONT) {
			if (hInfo.z > tmin && tmin > bias) {
				hInfo.z = tmin;
				hInfo.p = r.p + tmin * r.dir;
				hInfo.N = hInfo.p;
				hInfo.front = true;
			}
		}
		return true;
	}
	return false;
}

bool Plane::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	// If dz is zero, no intersection
	//if (fabs(dz) <= .0001) return false;

	//Going to use t = -Pz / dz
	float dz = ray.dir.z;
	float Pz = ray.p.z;
	float t = -Pz / dz;
	
	/* If the ray is on the wrong side of the plane, and we're culling out that hit side, return false */
	if (Pz < 0 && (hitSide == HIT_FRONT)) return false;
	if (Pz >= 0 && (hitSide == HIT_BACK)) return false;

	cyPoint3f intersection = ray.p + (t * ray.dir);

	/* Since this is a unit plane, return false if out of bounds. */
	if (fabs(intersection.x) > 1.0) return false;
	if (fabs(intersection.y) > 1.0) return false;
	//if (fabs(intersection.z) > 1.0) return false;
	
	/* Check depth */
	if (hInfo.z > t && t > bias) {

		hInfo.N = cyPoint3f(0, 0, 1.0f);
		hInfo.p = intersection;
		hInfo.z = t;
		hInfo.front = (Pz > 0); // correct

		return true;
	}

	return false;
}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	int numFaces = NF();

	for (int i = 0; i < numFaces; ++i) {
		IntersectTriangle(ray, hInfo, hitSide, i);
	}

	return (hInfo.node != NULL);
};

// Using Moller Trumbore Triangle Ray Intersection
bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
	// Rearranged barycentric ray triangle intersection
	//                           [t] 
	// O -V0 = [-D, V1-V0, V2-V0][u] 
	//                           [v] 

	// Using Cramer's rule
	// [t]         1      [ |T, E1, E2| ]
	// [u] = ------------ [ |-D, T, E2| ]
	// [v]   |-D, E1, E2| [ |-D, E1, T| ]
	//
	// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	//
	// [t]          1       [ (T X E1) . E2 ]
	// [u] = -------------- [ (D X E2) . T  ]
	// [v]   (D X E2) . E1  [ (T X E1) . D  ]
	
	// P = (D X E2), Q = (T X E1)
	/* Get the face in question */
	const TriFace &face = F(faceID);

	/* Get the triangle verticies */
	const cyPoint3f &v0 = V(face.v[0]);
	const cyPoint3f &v1 = V(face.v[1]);
	const cyPoint3f &v2 = V(face.v[2]);

	/* Vectors lying on the sides of this triangle */
	const cyPoint3f edge1 = v1 - v0; 
	const cyPoint3f edge2 = v2- v0;

	/* Calculate determinant, (also used to calc u param) */
	const cyPoint3f pvec = ray.dir ^ edge2;
	const float det = edge1 % pvec;

	// If the determinant is negative, the triangle is backfacing. 
	if ((hitSide == HIT_FRONT) && (det < 0.f)) return false;
	if ((hitSide == HIT_BACK) && (det > 0.f)) return false;

	// If the determinant is close to 0, the ray misses the triangle.
	if (fabs(det) < .00001) return false;

	// test U parameter 
	cyPoint3f tVec = ray.p - v0;
	float u = (tVec % pvec);// *invDet;
	if (u < 0 || u > det) return false;

	cyPoint3f qVec = tVec ^ edge1;
	float v = (ray.dir % qVec); //* invDet;
	if (v < 0 || u + v > det) return false;

	/* Calculate t, scale parameters, ray intersects triangle */
	float t = (edge2 % qVec);
	float invDet = 1 / det;
	t *= invDet;
	u *= invDet;
	v *= invDet;

	if (hInfo.z > t && t > bias) {
		cyPoint3f bc = cyPoint3f(1.0 - (u + v), u, v);
		hInfo.N = GetNormal(faceID, bc);
		hInfo.p = ray.p + t * ray.dir;
		hInfo.z = t;
		hInfo.front = (det > 0.f); // correct
		return true;
	}
	
	return true;
}