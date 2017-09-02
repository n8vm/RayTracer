#include "render.h"
#include <thread>

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;

std::thread *renderThread;

inline Color24 getWhite() {
	Color24 c;
	c.r = c.g = c.b = 255;
	return c;
}

inline Color24 getBlack() {
	Color24 c;
	c.r = c.g = c.b = 0;
	return c;
}

/* Initializes a struct of common camera ray information */
CommonRayInfo getCommonCameraRayInfo() {
	CommonRayInfo cri;
	cri.startPos = camera.pos;

	/* Camera reference frame */
	cri.forward = camera.dir;
	cri.up = camera.up;
	cri.left = camera.up.Cross(camera.dir).GetNormalized();

	/* height and width of near plane */
	float radfov = camera.fov * 3.141592653589793 / 180.0;
	cri.l = 1.0;
	cri.h = tan(radfov * .5f) * (2.0f * cri.l);
	cri.w = cri.h * camera.imgWidth / (float)camera.imgHeight;

	/* All rays start at bottom left */
	cyPoint3f B = camera.pos + (cri.l * camera.dir) + (cri.h / 2.0f * -camera.up);
	cri.nearPlaneBottomLeft = B + cri.w / 2.0f * (cri.left);
	cri.dXPixel = -cri.left * (cri.w / (float)camera.imgWidth);
	cri.dYPixel = cri.up * (cri.w / (float)camera.imgWidth);

	return cri;
}

Ray getCameraRay(int i, int j, const CommonRayInfo &cri) {
	Ray r;
	r.p = cri.startPos;
	r.dir = (cri.nearPlaneBottomLeft + (i + .5f) * cri.dXPixel + (j + .5f) * cri.dYPixel) - r.p;
	r.Normalize();
	return r;
}

void render() {
	Color24* img = renderImage.GetPixels();
	float* zBuffer = renderImage.GetZBuffer();
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	const CommonRayInfo cri = getCommonCameraRayInfo();
	renderImage.ResetNumRenderedPixels();
	// TODO: parallize this and make asynchronous
	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			Ray r = getCameraRay(x, y, cri);
			HitInfo info;
			bool hit = Trace(r, rootNode, info);
			img[x + y*width] = (hit) ? getWhite() : getBlack();
			zBuffer[x + y*width] = info.z;

			renderImage.IncrementNumRenderPixel(1);
		}
	}
	renderImage.ComputeZBufferImage();
}

// Called to start rendering (renderer must run in a separate thread)
void BeginRender() {
	/* Render asynchronously */
	renderThread = new std::thread(render);
}

// Called to end rendering (if it is not already finished)
void StopRender() {
	renderThread->join();
	renderThread->~thread();
}

bool Trace(Ray r, Node &n, HitInfo &info) {
	info.node = &n;
	bool hit = false;
	Ray r2 = n.ToNodeCoords(r);
	if (n.GetNodeObj() != nullptr) {
		hit |= n.GetNodeObj()->IntersectRay(r, info, 0);
	}

	for (int i = 0; i < n.GetNumChild(); ++i) {
		hit |= Trace(r2, *n.GetChild(i), info);
	}


	return hit;
}

