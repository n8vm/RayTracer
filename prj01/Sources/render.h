#pragma once
#include "scene.h"
#include "objects.h"
#include <iostream>

/* Common calculations among camera rays*/
struct CommonRayInfo {
	float h, w, l;
	cyPoint3f startPos;
	cyPoint3f up, left, forward;
	cyPoint3f nearPlaneBottomLeft;
	cyPoint3f dXPixel, dYPixel;
};

CommonRayInfo getCommonCameraRayInfo();
Ray getCameraRay(int i, int j, const CommonRayInfo &cri);
bool Trace(Ray r, Node &n, HitInfo &info);

inline bool compareRay(Ray &a, Ray &b) {
	if (a.p.x - b.p.x > .00001f) return false;
	if (a.p.y - b.p.y > .00001f) return false;
	if (a.p.z - b.p.z > .00001f) return false;
	if (a.dir.x - b.dir.x > .00001f) return false;
	if (a.dir.y - b.dir.y > .00001f) return false;
	if (a.dir.z - b.dir.z >.00001f) return false;
	return true;
}
