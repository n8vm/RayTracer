#include "objects.h"
#include <stack>

bool Sphere::IntersectRay(const Rays &rays, HitInfo &hInfo, int hitSide) const
{
	float bias = .001;
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
			if (hInfo.z > tmax && tmax > bias) {
				hInfo.z = tmax;
				hInfo.p = rays.mainRay.p + tmax * rays.mainRay.dir;
				hInfo.N = hInfo.p;
				hInfo.front = false;
				hInfo.uvw.x = .5 - atan2(hInfo.p.x, hInfo.p.y) / (2.0 * M_PI);
				hInfo.uvw.y = .5 + asin(hInfo.p.z) / M_PI;
				hInfo.uvw.z = 0;
				
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
			}
		}
		if (hitSide & HIT_FRONT) {
			if (hInfo.z > tmin && tmin > bias) {
				hInfo.z = tmin;
				hInfo.p = rays.mainRay.p + tmin * rays.mainRay.dir;
				hInfo.N = hInfo.p;
				hInfo.front = true;
				hInfo.uvw.x = .5 - atan2(hInfo.p.x, hInfo.p.y) / (2.0 * M_PI);
				hInfo.uvw.y = .5 + asin(hInfo.p.z) / M_PI;
				hInfo.uvw.z = 0;

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
			}
		}
		return true;
	}
	return false;
}

bool Plane::IntersectRay(const Rays &rays, HitInfo &hInfo, int hitSide) const {
	float bias = .0001;
	// If dz is zero, no intersection
	//if (fabs(dz) <= .0001) return false;

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
	if (hInfo.z > t && t > bias) {
		float tdx = -rays.difx.p.z / rays.difx.dir.z;
		float tdy = -rays.dify.p.z / rays.dify.dir.z;
		cyPoint3f dxi = rays.difx.p + (tdx * rays.difx.dir);
		cyPoint3f dyi = rays.dify.p + (tdy * rays.dify.dir);

		//float dtdx = -(rays.difx.p + t * rays.difx.dir) % cyPoint3f(0, 0, 1.0f) / (rays.mainRay.dir % cyPoint3f(0, 0, 1.0f));
		//float dtdy = -(rays.dify.p + t * rays.dify.dir) % cyPoint3f(0, 0, 1.0f) / (rays.mainRay.dir % cyPoint3f(0, 0, 1.0f));

		//cyPoint3f dpdx = rays.difx.p + (t * rays.difx.dir) + (dtdx * rays.mainRay.dir);
		//cyPoint3f dpdy = rays.dify.p + (t * rays.dify.dir) + (dtdy * rays.mainRay.dir);


		hInfo.N = cyPoint3f(0, 0, 1.0f);
		hInfo.p = intersection;
		hInfo.z = t;
		hInfo.front = (Pz > 0);
		hInfo.uvw.x = (intersection.x + 1.f) * .5f;
		hInfo.uvw.y = (intersection.y + 1.f) * .5f;
		hInfo.uvw.z = 0;

		// deltaTx = [T(x + deltax, y) - T(x,y)]
		// deltaTx = [T(x, y + deltay) - T(x,y)]
		hInfo.duvw[0].x = 2.0 * (dxi.x - intersection.x);
		hInfo.duvw[0].y = 2.0 * (dxi.y - intersection.y);
		hInfo.duvw[0].z = 0;

		hInfo.duvw[1].x = 2.0 * (dyi.x - intersection.x);
		hInfo.duvw[1].y = 2.0 * (dyi.y - intersection.y);
		hInfo.duvw[1].z = 0;

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

bool TriObj::TraceBVHNode(const Rays &rays, HitInfo &hInfo, int hitSide, unsigned int nodeID) const {
	const float* b = bvh.GetNodeBounds(nodeID);
	/* Test for ray box intersection */
	cyPoint3f dirfrac;
	dirfrac.x = 1.0f / rays.mainRay.dir.x;
	dirfrac.y = 1.0f / rays.mainRay.dir.y;
	dirfrac.z = 1.0f / rays.mainRay.dir.z;

	float t1 = (b[0] - rays.mainRay.p.x) * dirfrac.x;
	float t2 = (b[3] - rays.mainRay.p.x) * dirfrac.x;
	float t3 = (b[1] - rays.mainRay.p.y) * dirfrac.y;
	float t4 = (b[4] - rays.mainRay.p.y) * dirfrac.y;
	float t5 = (b[2] - rays.mainRay.p.z) * dirfrac.z;
	float t6 = (b[5] - rays.mainRay.p.z) * dirfrac.z;

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
			hit |= IntersectTriangle(rays, hInfo, hitSide, elements[i]);
		}
		return hit;
	}
	else 
		return true;
}

bool TriObj::IntersectTriangle(const Rays &rays, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
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
	const cyPoint3f &p0 = V(face.v[0]);
	const cyPoint3f &p1 = V(face.v[1]);
	const cyPoint3f &p2 = V(face.v[2]);

	/* Vectors lying on the sides of this triangle */
	const cyPoint3f edge1 = p1 - p0; 
	const cyPoint3f edge2 = p2- p0;

	/* Calculate determinant, (also used to calc u param) */
	const cyPoint3f pvec1 = rays.mainRay.dir ^ edge2;
	const cyPoint3f pvec2 = rays.difx.dir ^ edge2;
	const cyPoint3f pvec3 = rays.dify.dir ^ edge2;
	const float det1 = edge1 % pvec1;
	const float det2 = edge1 % pvec2;
	const float det3 = edge1 % pvec3;

	// If the determinant is negative, the triangle is backfacing. 
	if ((hitSide == HIT_FRONT) && (det1 < 0)) return false;
	if ((hitSide == HIT_BACK) && (det1 > 0)) return false;

	// If the determinant is close to 0, the ray misses the triangle.
	//if (fabs(det) < .000001) return false; // CAUSING BUGS

	// test U parameter 
	cyPoint3f tVec1 = rays.mainRay.p - p0;
	cyPoint3f tVec2 = rays.difx.p - p0;
	cyPoint3f tVec3 = rays.dify.p - p0;
	float u1 = (tVec1 % pvec1);
	if (u1 < 0 || u1 > det1) return false;
	float u2 = (tVec2 % pvec2);
	float u3 = (tVec3 % pvec3);

	cyPoint3f qVec1 = tVec1 ^ edge1;
	float v1 = (rays.mainRay.dir % qVec1);
	if (v1 < 0 || u1 + v1 > det1) return false;
	cyPoint3f qVec2 = tVec2 ^ edge1;
	cyPoint3f qVec3 = tVec3 ^ edge1;
	float v2 = (rays.mainRay.dir % qVec1);
	float v3 = (rays.mainRay.dir % qVec1);

	/* Calculate t, scale parameters, ray intersects triangle */
	float t = (edge2 % qVec1);
	float invDet1 = 1 / det1;
	float invDet2 = 1 / det2;
	float invDet3 = 1 / det3;
	t *= invDet1;
	u1 *= invDet1;
	v1 *= invDet1;

	u2 *= invDet2;
	v2 *= invDet2;

	u3 *= invDet3;
	v3 *= invDet3;
	if (hInfo.z > t && t > bias) {
		cyPoint3f bc1 = cyPoint3f(1.0 - (u1 + v1), u1, v1);
		cyPoint3f bc2 = cyPoint3f(1.0 - (u2 + v2), u2, v2);
		cyPoint3f bc3 = cyPoint3f(1.0 - (u3 + v3), u3, v3);
		cyPoint3f uvw = GetTexCoord(faceID, bc1);
		hInfo.N = GetNormal(faceID, bc1).GetNormalized();
		hInfo.uvw = uvw;
		hInfo.p = rays.mainRay.p + t * rays.mainRay.dir;
		hInfo.z = t;
		hInfo.front = (det1 > 0.f); // correct

		cyPoint3f dxuvw = GetTexCoord(faceID, bc2);
		cyPoint3f dyuvw = GetTexCoord(faceID, bc3);

		/*float tdx = ((hInfo.p - rays.difx.p) % hInfo.N) / (rays.difx.dir % hInfo.N);
		float tdy = ((hInfo.p - rays.dify.p) % hInfo.N) / (rays.dify.dir % hInfo.N);
		cyPoint3f dxi = rays.difx.p + (tdx * rays.difx.dir);
		cyPoint3f dyi = rays.dify.p + (tdy * rays.dify.dir);
*/
		hInfo.duvw[0].x = 2.0 * (dxuvw.x - uvw.x);
		hInfo.duvw[0].y = 2.0 * (dxuvw.y - uvw.y);
		hInfo.duvw[0].z = 0;

		hInfo.duvw[1].x = 2.0 * (dyuvw.x - uvw.x);
		hInfo.duvw[1].y = 2.0 * (dyuvw.y - uvw.y);
		hInfo.duvw[1].z = 0;

		return true;
	}
	
	return true;
}