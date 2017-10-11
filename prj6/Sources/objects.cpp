#include "objects.h"
#include <stack>

bool Sphere::IntersectRay(const Ray &r, HitInfo &hInfo, int hitSide) const
{
	float bias = .001;
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
		float tmax = MAX(t1, t2);
		float tmin = MIN(t1, t2);

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
	float bias = .0001;
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
		hInfo.front = (Pz > 0);
		return true;
	}

	return false;
}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	int numFaces = NF();
	//std::stack<unsigned int> nodeStack = std::stack<unsigned int>(); // Make array > 36 (32 is around 4 billion triangle)
	unsigned int nodeStack[50];
	unsigned int next = 1;

	/* Put the root node on the stack */
	nodeStack[0] = bvh.GetRootNodeID();
	
	while (next > 0) {
		/* Take the first node off the stack */
		next--;
		int id = nodeStack[next];
		
		/* If the ray misses the node, it'll also miss all 
			the children nodes of this node, so we skip it. 
			Alternatively if the ray previously hit something closer, 
			skip this node and all it's children as well.
			*/
		if (TraceBVHNode(ray, hInfo, hitSide, id) == false)
			continue;

		/* If the node we hit isn't a leaf, put the children on the stack. */
		if (!bvh.IsLeafNode(id)) {
			/*  */
			unsigned int c1, c2;
			bvh.GetChildNodes(id, c1, c2);
			
			nodeStack[next] = c2;
			next++;

			nodeStack[next] = c1;
			next++;
		}
	}

	return (hInfo.node != NULL);
};

bool TriObj::TraceBVHNode(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID) const {
	const float* b = bvh.GetNodeBounds(nodeID);
	/* Test for ray box intersection */
	cyPoint3f dirfrac;
	dirfrac.x = 1.0f / ray.dir.x;
	dirfrac.y = 1.0f / ray.dir.y;
	dirfrac.z = 1.0f / ray.dir.z;

	float t1 = (b[0] - ray.p.x) * dirfrac.x;
	float t2 = (b[3] - ray.p.x) * dirfrac.x;
	float t3 = (b[1] - ray.p.y) * dirfrac.y;
	float t4 = (b[4] - ray.p.y) * dirfrac.y;
	float t5 = (b[2] - ray.p.z) * dirfrac.z;
	float t6 = (b[5] - ray.p.z) * dirfrac.z;

	float tmin = MAX(MAX(MIN(t1, t2), MIN(t3, t4)), MIN(t5, t6));
	float tmax = MIN(MIN(MAX(t1, t2), MAX(t3, t4)), MAX(t5, t6));

	if (tmax < 0) // AABB is behind the ray
		return false;

	if (tmin > tmax) // The ray misses the box
		return false;

	if (tmin > hInfo.z)  // The box is behind something closer
		return false; 

	if (bvh.IsLeafNode(nodeID)) {
		/* If this node is a leaf, check for intersection with each of it's elements */
		int totalElems = bvh.GetNodeElementCount(nodeID);
		const unsigned int* elements = bvh.GetNodeElements(nodeID);
		bool hit = false;
		for (int i = 0; i < totalElems; ++i) {
			hit |= IntersectTriangle(ray, hInfo, hitSide, elements[i]);
		}
		return hit;
	}
	else 
		return true;
}
// Using Moller Trumbore Triangle Ray Intersection
bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
	float bias = .000008;
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
	if ((hitSide == HIT_FRONT) && (det < 0)) return false;
	if ((hitSide == HIT_BACK) && (det > 0)) return false;

	// If the determinant is close to 0, the ray misses the triangle.
	//if (fabs(det) < .000001) return false; // CAUSING BUGS

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
		hInfo.N = GetNormal(faceID, bc).GetNormalized();
		hInfo.p = ray.p + t * ray.dir;
		hInfo.z = t;
		hInfo.front = (det > 0.f); // correct
		return true;
	}
	
	return true;
}