#ifndef RENDER_H
#define RENDER_H

#include "scene.h"
#include "objects.h"
#include <iostream>
#include <mutex>
#include "cyCodeBase/cyIrradianceMap.h"

extern RenderImage renderImage;

/* Common calculations among camera rays*/
struct CommonRayInfo {
	float h, w, l;
	cyPoint3f startPos;
	cyPoint3f up, left, forward;
	cyPoint3f nearPlaneTopLeft;
	cyPoint3f dXPixel, dYPixel;
};

CommonRayInfo getCommonCameraRayInfo();
Ray getCameraRay(float i, float j, float ddi, float ddj, float dpi, float dpj, const CommonRayInfo &cri);
bool Trace(Rays rays, Node &n, HitInfo &info, int hitSide=HIT_FRONT, float t_max = BIGFLOAT);
//void Trace_internal(Rays rays, Node &n, HitInfo &info, int hitSide = HIT_FRONT, float t_max = BIGFLOAT);
void render_pixel(int tid, int x, int y, Color &avgColor, int &totalSamples, float &depth, cyPoint3f &normal, bool use_indirect);
void render();
inline bool compareRay(Ray &a, Ray &b) {
	if (a.p.x - b.p.x > .00001f) return false;
	if (a.p.y - b.p.y > .00001f) return false;
	if (a.p.z - b.p.z > .00001f) return false;
	if (a.dir.x - b.dir.x > .00001f) return false;
	if (a.dir.y - b.dir.y > .00001f) return false;
	if (a.dir.z - b.dir.z >.00001f) return false;
	return true;
}

inline bool comparePoint(cyPoint3f &a, cyPoint3f &b) {
	if (a.x - b.x > .00001f) return false;
	if (a.y - b.y > .00001f) return false;
	if (a.z - b.z > .00001f) return false;
	return true;
}

inline cyPoint3f invertPoint(cyPoint3f in) {
	cyPoint3f out;
	out[0] = 1.0 / in[0];
	out[1] = 1.0 / in[1];
	out[2] = 1.0 / in[2];
	return out;
}

class HaltonIDX {
private:
	int _AAidx = 0;
	int _DOFidx = 0;
	int _SSidx = 0;
	int _GIidx[TOTAL_BOUNCES] = {};
	int _Reflidx[TOTAL_BOUNCES] = {};
	int _Refridx[TOTAL_BOUNCES] = {};
public:
	inline int AAidx() {
#if USE_HALTON 
		return ++_AAidx;
#else 
		return rand() >> 1;
#endif
	}
	inline int DOFidx() {
#if USE_HALTON 
		return ++_DOFidx;
#else 
		return rand() >> 1;
#endif
	}
	inline int SSidx() {
#if USE_HALTON 
		return ++_SSidx;
#else 
		return rand() >> 1;
#endif
	}
	inline int GIidx(int bounce) {
#if USE_HALTON 
		return ++_GIidx[bounce];
#else 
		return rand() >> 1;
#endif
	}
	inline int Reflidx(int bounce) {
#if USE_HALTON 
		return ++_Reflidx[bounce];
#else 
		return rand() >> 1;
#endif
	}
	inline int Refridx(int bounce) {
#if USE_HALTON 
		return ++_Refridx[bounce];
#else 
		return rand() >> 1;
#endif
	}
};


class IrradianceMap : public cyIrradianceMapColorZNormal {
	std::mutex irradiance_mutex;
public:
	IrradianceMap(float _thresholdColor = 1.0e30f, float _thresholdZ = 1.0e30f, float _thresholdN = 0.7f) 
		: cyIrradianceMapColorZNormal(_thresholdColor, _thresholdZ, _thresholdN) {}
	
	int previousPhase = -1;
	void ComputePoint(cyColorZNormal &data, float x, float y, int threadID) {
		int totalSamples;
		render_pixel(threadID, x, y, data.c, totalSamples, data.z, data.N, true);
		int width = renderImage.GetWidth();
		int height = renderImage.GetHeight();
		int ix = x, iy = y;
		if (ix < width && iy < height && ix >= 0 && iy >= 0)
			renderImage.GetIrradianceComputationImage()[ix + iy*width] = 255;
		
		if (threadID == 0 && currentPhase != previousPhase) {
			irradiance_mutex.lock();
			previousPhase = currentPhase;
			std::cout << "phase: " + std::to_string(currentPhase) +  " currentSubdiv: " + std::to_string(currentSubdiv) << std::endl;
			irradiance_mutex.unlock();
		}
	}
};
#endif
