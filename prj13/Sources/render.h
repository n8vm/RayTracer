#ifndef RENDER_H
#define RENDER_H

#include "scene.h"
#include "objects.h"
#include <iostream>
#include <mutex>
#include "cyCodeBase/cyIrradianceMap.h"
#include "cyCodeBase/cyPhotonMap.h"

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
void render_pixel(int tid, int x, int y, Color &avgColor, int &totalSamples, float &depth, cyPoint3f &normal, IlluminationType directType, IlluminationType indirectType);
void render_pt();
void render_sppm();

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
	int _PM = 0;
	int _GIidx[TOTAL_BOUNCES] = {};
	int _Reflidx[TOTAL_BOUNCES] = {};
	int _Refridx[TOTAL_BOUNCES] = {};
public:
#if USE_HALTON 
	inline int AAidx() { _AAidx++; if (_AAidx > 1024) _AAidx = 0; return ++_AAidx; }
	inline int DOFidx() { return ++_DOFidx; }
	inline int SSidx() { return ++_SSidx; }
	inline int GIidx(int bounce) { return ++_GIidx[bounce]; }
	inline int PMidx() { return ++_PM; }
	inline int Reflidx(int bounce) { return ++_Reflidx[bounce]; }
	inline int Refridx(int bounce) { return ++_Refridx[bounce]; }
#else
	inline int AAidx() { return rand() >> 1; }
	inline int DOFidx() { return rand() >> 1; }
	inline int SSidx() { return rand() >> 1; }
	inline int PMidx() { return rand() >> 1; }
	inline int GIidx(int bounce) { return rand() >> 1; }
	inline int Reflidx(int bounce) { return rand() >> 1; }
	inline int Refridx(int bounce) { return rand() >> 1; }
#endif
};


class IrradianceMap : public cyIrradianceMapColorZNormal {
	std::mutex irradiance_mutex;
public:
	IrradianceMap(float _thresholdColor = 1.0e30f, float _thresholdZ = 1.0e30f, float _thresholdN = 0.7f) 
		: cyIrradianceMapColorZNormal(_thresholdColor, _thresholdZ, _thresholdN) {}
	
	int previousPhase = -1;
	void ComputePoint(cyColorZNormal &data, float x, float y, int threadID) {
		int totalSamples;
		render_pixel(threadID, x, y, data.c, totalSamples, data.z, data.N, IRRADIANCE_DIRECT_TYPE, INDIRECT_TYPE);
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
