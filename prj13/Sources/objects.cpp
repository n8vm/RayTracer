#include "objects.h"
#include "options.h"

#include <stack>

bool Sphere::IntersectRay(const Rays &rays, HitInfo &hInfo, int hitSide) const
{
	// Ray in model space
	float a = (rays.mainRay.dir).Dot(rays.mainRay.dir);   // dir dot dir
	float b = 2.f*(rays.mainRay.p.Dot(rays.mainRay.dir)); // 2 * ray point dot dir
	float c = rays.mainRay.p.Dot(rays.mainRay.p) - 1.f;   // ray point dot ray point - r^2

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
			if (hInfo.z > tmax && tmax > SPHERE_BIAS) {
				hInfo.z = tmax;
				if (hInfo.shadow) return true;

				hInfo.p = rays.mainRay.p + tmax * rays.mainRay.dir;
				hInfo.N = hInfo.p;
				hInfo.front = false;
				hInfo.uvw.x = .5 - atan2(hInfo.p.x, hInfo.p.y) / (2.0 * M_PI);
				hInfo.uvw.y = .5 + asin(hInfo.p.z) / M_PI;
				hInfo.uvw.z = 0;
				
#if USE_RAY_DIFFERENTIALS
				float tdx = ((hInfo.p - rays.difx.p) % hInfo.N) / (rays.difx.dir % hInfo.N);
				float tdy = ((hInfo.p - rays.dify.p) % hInfo.N) / (rays.dify.dir % hInfo.N);
				cyPoint3f dxi = rays.difx.p + (tdx * rays.difx.dir);
				cyPoint3f dyi = rays.dify.p + (tdy * rays.dify.dir);
				hInfo.duvw[0].x = 2.0 * (dxi.x - hInfo.p.x);
				hInfo.duvw[0].y = 2.0 * (dxi.y - hInfo.p.y);
				hInfo.duvw[0].z = 0;

				hInfo.duvw[1].x = 2.0 * (dyi.x - hInfo.p.x);
				hInfo.duvw[1].y = 2.0 * (dyi.y - hInfo.p.y);
				hInfo.duvw[1].z = 0;
#endif
			}
		}
		if (hitSide & HIT_FRONT) {
			if (hInfo.z > tmin && tmin > SPHERE_BIAS) {

				hInfo.z = tmin;
				if (hInfo.shadow) return true;
				hInfo.p = rays.mainRay.p + tmin * rays.mainRay.dir;
				hInfo.N = hInfo.p;
				hInfo.front = true;
				hInfo.uvw.x = .5 - atan2(hInfo.p.x, hInfo.p.y) / (2.0 * M_PI);
				hInfo.uvw.y = .5 + asin(hInfo.p.z) / M_PI;
				hInfo.uvw.z = 0;

#if USE_RAY_DIFFERENTIALS
				float tdx = ((hInfo.p - rays.difx.p) % hInfo.N) / (rays.difx.dir % hInfo.N);
				float tdy = ((hInfo.p - rays.dify.p) % hInfo.N) / (rays.dify.dir % hInfo.N);
				cyPoint3f dxi = rays.difx.p + (tdx * rays.difx.dir);
				cyPoint3f dyi = rays.dify.p + (tdy * rays.dify.dir);

				hInfo.duvw[0].x = 2.0 * (dxi.x - hInfo.p.x);
				hInfo.duvw[0].y = 2.0 * (dxi.y - hInfo.p.y);
				hInfo.duvw[0].z = 0;

				hInfo.duvw[1].x = 2.0 * (dyi.x - hInfo.p.x);
				hInfo.duvw[1].y = 2.0 * (dyi.y - hInfo.p.y);
				hInfo.duvw[1].z = 0;
#endif
			}
		}
		return true;
	}
	return false;
}

bool Plane::IntersectRay(const Rays &rays, HitInfo &hInfo, int hitSide) const {
	//Going to use t = -Pz / dz
	float dz = rays.mainRay.dir.z;
	float Pz = rays.mainRay.p.z;
	float t = -Pz / dz;
	
	/* If the ray is on the wrong side of the plane, and we're culling out that hit side, return false */
	if (Pz < 0 && (hitSide == HIT_FRONT)) return false;
	if (Pz >= 0 && (hitSide == HIT_BACK)) return false;

	cyPoint3f intersection = rays.mainRay.p + (t * rays.mainRay.dir);
	
	/* Since this is a unit plane, return false if out of bounds. */
	if (fabs(intersection.x) > 1.0) return false;
	if (fabs(intersection.y) > 1.0) return false;
		
	/* Check depth */
	if (hInfo.z > t && t > PLANE_BIAS) {
		hInfo.z = t;
		if (hInfo.shadow) {
			return true;
		}
		hInfo.N = cyPoint3f(0, 0, 1.0f);
		hInfo.p = intersection;
		hInfo.front = (Pz > 0);
		hInfo.uvw.x = (intersection.x + 1.f) * .5f;
		hInfo.uvw.y = (intersection.y + 1.f) * .5f;
		hInfo.uvw.z = 0;

#if USE_RAY_DIFFERENTIALS
		float tdx = -rays.difx.p.z / rays.difx.dir.z;
		float tdy = -rays.dify.p.z / rays.dify.dir.z;
		cyPoint3f dxi = rays.difx.p + (tdx * rays.difx.dir);
		cyPoint3f dyi = rays.dify.p + (tdy * rays.dify.dir);
		hInfo.duvw[0].x = 2.0 * (dxi.x - intersection.x);
		hInfo.duvw[0].y = 2.0 * (dxi.y - intersection.y);
		hInfo.duvw[0].z = 0;

		hInfo.duvw[1].x = 2.0 * (dyi.x - intersection.x);
		hInfo.duvw[1].y = 2.0 * (dyi.y - intersection.y);
		hInfo.duvw[1].z = 0;
#endif

		return true;
	}

	return false;
}

bool TriObj::IntersectRay(const Rays &rays, HitInfo &hInfo, int hitSide) const {
	int numFaces = NF();
	//std::stack<unsigned int> nodeStack = std::stack<unsigned int>(); // Make array > 36 (32 is around 4 billion triangle)
	unsigned int nodeStack[50];
	unsigned int next = 1;

	/* Put the root node on the stack */
	nodeStack[0] = bvh.GetRootNodeID();
	
	float originalZ = hInfo.z;
	while (next > 0) {
		/* Take the first node off the stack */
		next--;
		int id = nodeStack[next];
		
		/* If the ray misses the node, it'll also miss all 
			the children nodes of this node, so we skip it. 
			Alternatively if the ray previously hit something closer, 
			skip this node and all it's children as well.
			*/
		if (TraceBVHNode(rays, hInfo, hitSide, id) == false)
			continue;

		if (hInfo.shadow && hInfo.z != originalZ) return true;

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

//bool TriObj::TraceBVHNode(const Rays &rays, HitInfo &hInfo, int hitSide, unsigned int nodeID) const {
//	const float* b = bvh.GetNodeBounds(nodeID);
//	/* Test for ray box intersection */
//	cyPoint3f dirfrac;
//	dirfrac.x = 1.0f / rays.mainRay.dir.x;
//	dirfrac.y = 1.0f / rays.mainRay.dir.y;
//	dirfrac.z = 1.0f / rays.mainRay.dir.z;
//
//	float t1 = (b[0] - rays.mainRay.p.x) * dirfrac.x;
//	float t2 = (b[3] - rays.mainRay.p.x) * dirfrac.x;
//	float t3 = (b[1] - rays.mainRay.p.y) * dirfrac.y;
//	float t4 = (b[4] - rays.mainRay.p.y) * dirfrac.y;
//	float t5 = (b[2] - rays.mainRay.p.z) * dirfrac.z;
//	float t6 = (b[5] - rays.mainRay.p.z) * dirfrac.z;
//
//	float tmin = fmax(fmax(fmin(t1, t2), fmin(t3, t4)), fmin(t5, t6));
//	float tmax = fmin(fmin(fmax(t1, t2), fmax(t3, t4)), fmax(t5, t6));
//
//	if (tmax < 0) // AABB is behind the ray
//		return false;
//
//	if (tmin > tmax) // The ray misses the box
//		return false;
//
//	if (tmin > hInfo.z)  // The box is behind something closer
//		return false; 
//
//	if (bvh.IsLeafNode(nodeID)) {
//		/* If this node is a leaf, check for intersection with each of it's elements */
//		int totalElems = bvh.GetNodeElementCount(nodeID);
//		const unsigned int* elements = bvh.GetNodeElements(nodeID);
//		bool hit = false;
//		for (int i = 0; i < totalElems; ++i) {
//			hit |= IntersectTriangle(rays, hInfo, hitSide, elements[i]);
//		}
//		return hit;
//	}
//	else 
//		return true;
//}

bool TriObj::TraceBVHNode(const Rays &rays, HitInfo &hInfo, int hitSide, unsigned int nodeID) const {
	const float* b = bvh.GetNodeBounds(nodeID);
	
	/* Test for ray box intersection */
	double tmin = -INFINITY, tmax = INFINITY;

	for (int i = 0; i < 3; ++i) {
		double t1 = (b[i] - rays.mainRay.p[i])*rays.mainRay.dir_inv[i];
		double t2 = (b[i + 3] - rays.mainRay.p[i])*rays.mainRay.dir_inv[i];

		tmin = MAX(tmin, MIN(t1, t2));
		tmax = MIN(tmax, MAX(t1, t2));
	}

	if (tmax < PLANE_BIAS) // AABB is behind the ray
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
				hit |= IntersectTriangle(rays, hInfo, hitSide, elements[i]);
			}
			return hit;
		}
		else 
			return true;
}

bool TriObj::IntersectTriangle(const Rays &rays, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
	/* Get the face in question */
	const TriFace &face = F(faceID);

	/* Get the triangle verticies */
	const cyPoint3d &p0 = cyPoint3d(V(face.v[0]));
	const cyPoint3d &p1 = cyPoint3d(V(face.v[1]));
	const cyPoint3d &p2 = cyPoint3d(V(face.v[2]));

	/* Vectors lying on the sides of this triangle */
	const cyPoint3d edge1 = p1 - p0; 
	const cyPoint3d edge2 = p2- p0;

	/* Calculate determinant, (also used to calc u param) */
	const cyPoint3d pvec1 = cyPoint3d(rays.mainRay.dir) ^ edge2;
	const double det1 = edge1 % pvec1;
	// If the determinant is negative, the triangle is backfacing. 
	if (((hitSide & HIT_BACK) == 0) && (det1 < 0.00000001)) return false;
	if (((hitSide & HIT_FRONT) == 0) && (fabs(det1) < 0.00000001)) return false;

	double invDet1 = 1.0 / det1;

	// test U parameter 
	cyPoint3d tVec1 = cyPoint3d(rays.mainRay.p) - p0;
	double u1 = (tVec1 % pvec1) * invDet1; 
	if (u1 < 0.0 || u1 > 1.0) return false;

	cyPoint3d qVec1 = tVec1 ^ edge1;
	double v1 = (cyPoint3d(rays.mainRay.dir) % qVec1) * invDet1;
	if (v1 < 0.0 || u1 + v1 > 1.0) return false;
	
	/* Calculate t, scale parameters, ray intersects triangle */
	double t = (edge2 % qVec1) * invDet1;

#if USE_RAY_DIFFERENTIALS
	const cyPoint3d pvec2 = cyPoint3d(rays.difx.dir) ^ edge2;
	const cyPoint3d pvec3 = cyPoint3d(rays.dify.dir) ^ edge2;
	const double det2 = edge1 % pvec2;
	const double det3 = edge1 % pvec3;
	cyPoint3d tVec2 = cyPoint3d(rays.difx.p) - p0;
	cyPoint3d tVec3 = cyPoint3d(rays.dify.p) - p0;
	double u2 = (tVec2 % pvec2);
	double u3 = (tVec3 % pvec3);
	cyPoint3d qVec2 = tVec2 ^ edge1;
	cyPoint3d qVec3 = tVec3 ^ edge1;
	double v2 = (cyPoint3d(rays.mainRay.dir) % qVec1);
	double v3 = (cyPoint3d(rays.mainRay.dir) % qVec1);
	double invDet2 = 1 / det2;
	double invDet3 = 1 / det3;
	u2 *= invDet2;
	v2 *= invDet2;
	u3 *= invDet3;
	v3 *= invDet3;
#endif

	if (hInfo.z > t && t > TRIANGLE_BIAS) {
		cyPoint3f bc1 = cyPoint3f(1.0 - (u1 + v1), u1, v1);
		cyPoint3f uvw;
		if (HasTextureVertices()) {
			uvw = GetTexCoord(faceID, bc1);
			hInfo.uvw = uvw;
		}
		hInfo.N = GetNormal(faceID, bc1).GetNormalized();
		hInfo.p = rays.mainRay.p + t * rays.mainRay.dir;
		hInfo.z = t;
		hInfo.front = (rays.mainRay.dir % hInfo.N) < 0;// (det1 > 0.f); // correct

#if USE_RAY_DIFFERENTIALS
		cyPoint3f bc2 = cyPoint3f(1.0 - (u2 + v2), u2, v2);
		cyPoint3f bc3 = cyPoint3f(1.0 - (u3 + v3), u3, v3);
		if (HasTextureVertices()) {
			cyPoint3f dxuvw = GetTexCoord(faceID, bc2);
			cyPoint3f dyuvw = GetTexCoord(faceID, bc3);
			hInfo.duvw[0].x = 2.0 * (dxuvw.x - uvw.x);
			hInfo.duvw[0].y = 2.0 * (dxuvw.y - uvw.y);
			hInfo.duvw[0].z = 0;
			hInfo.duvw[1].x = 2.0 * (dyuvw.x - uvw.x);
			hInfo.duvw[1].y = 2.0 * (dyuvw.y - uvw.y);
			hInfo.duvw[1].z = 0;
		}
#endif

		return true;
	}
	
	return true;
}
