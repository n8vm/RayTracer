#include "render.h"
#include <thread>
#include "scene.h"

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;

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

inline Color24 toColor24(Color c) {
	Color24 newCol = Color24();
	newCol.r = min(c.r * 255.f, 255);
	newCol.g = min(c.g * 255.f, 255);
	newCol.b = min(c.b * 255.f, 255);
	return newCol;
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

	/* All rays start at top left */
	cyPoint3f B = camera.pos + (cri.l * camera.dir) + (cri.h / 2.0f * cri.up);
	cri.nearPlaneTopLeft = B + cri.w / 2.0f * (cri.left);
	cri.dXPixel = -cri.left * (cri.w / (float)camera.imgWidth);
	cri.dYPixel = -cri.up * (cri.w / (float)camera.imgWidth);

	return cri;
}

Ray getCameraRay(int i, int j, const CommonRayInfo &cri) {
	Ray r;
	r.p = cri.startPos;
	r.dir = (cri.nearPlaneTopLeft + (i + .5f) * cri.dXPixel + (j + .5f) * cri.dYPixel) - r.p;
	r.Normalize();
	return r;
}

static int pickedx = -1, pickedy = -1;
static bool showNormals = false;
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
			if (pickedx == x && pickedy == y) {
				std::cout << std::endl;
			}

			Ray r = getCameraRay(x, y, cri);
			HitInfo info;
			Trace(r, rootNode, info, false);
			Color24 pixelColor = getBlack();
			if (info.node) {
				const Material *hitMat = info.node->GetMaterial();
				if (showNormals) {
					pixelColor.r = (info.N.x + 1.0f) * 128;
					pixelColor.g = (info.N.y + 1.0f) * 128;
					pixelColor.b = (info.N.z + 1.0f) * 128;
				} else 
					pixelColor = toColor24(hitMat->Shade(r, info, lights, 20));
			}
			img[x + y*width] = pixelColor; 
			zBuffer[x + y*width] = info.z;

			renderImage.IncrementNumRenderPixel(1);
		}
	}
	renderImage.ComputeZBufferImage();
	renderImage.SaveImage("Image.png");
	renderImage.SaveZImage("zImage.png");
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

void Trace(Ray r, Node &n, HitInfo &info, bool hitBackFaces) {	
	Ray r2 = n.ToNodeCoords(r);
	
	/* If this node has an obj */
	if (n.GetNodeObj()) {
		float oldZ = info.z;
		n.GetNodeObj()->IntersectRay(r2, info, (hitBackFaces) ? 3 : 0);
		/* If the z value decreased, this node is the closest node that the ray hit */
		if (info.z < oldZ) {
			info.node = &n;
		}
	}

	/* Recurse through the children */
	for (int i = 0; i < n.GetNumChild(); ++i) {
		const Node *previous = info.node;
		Trace(r2, *n.GetChild(i), info, hitBackFaces);
		if (info.node != previous) {
			n.GetChild(i)->FromNodeCoords(info);
		}
	}
}

