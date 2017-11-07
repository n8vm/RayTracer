#include "render.h"
#include <thread>
#include "scene.h"

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;
extern TexturedColor background;
extern TexturedColor environment;

std::thread *renderThread;
std::atomic_int pixelsDone;

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

Ray getCameraRay(float i, float j, const CommonRayInfo &cri) {
	Ray r;
	r.p = cri.startPos;
	r.dir = (cri.nearPlaneTopLeft + (i + .5f) * cri.dXPixel + (j + .5f) * cri.dYPixel) - r.p;
	//r.Normalize();
	return r;
}

// From section 3 of "Tracing Ray Differentials"
Ray getCameraRayDX(Ray r, cyPoint3f right) {
	cyPoint3f dx = (r.dir % r.dir) * right - (r.dir % right) / sqrt(pow((r.dir % r.dir), 3));
	return Ray(cyPoint3f(0, 0, 0), dx);
}

// From section 3 of "Tracing Ray Differentials"
Ray getCameraRayDY(Ray r, cyPoint3f up) {
	cyPoint3f dy = (r.dir % r.dir) * up - (r.dir % up) / sqrt(pow((r.dir % r.dir), 3));
	return Ray(cyPoint3f(0, 0, 0), dy);
}

static int pickedx = -1, pickedy = -1;
static bool showNormals = false;
static int totalBounces = 4;
void render_internal() {
	Color24* img = renderImage.GetPixels();
	float* zBuffer = renderImage.GetZBuffer();
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	const CommonRayInfo cri = getCommonCameraRayInfo();

	while (pixelsDone < width * height) {
		/* Get pixel location*/
		int pixel = std::atomic_fetch_add(&pixelsDone, 1);
		int x = pixel / height;
		int y = pixel % height;
		
		if (pickedx == x && pickedy == y) {
			std::cout << std::endl;
		}

		Rays rays;
		rays.mainRay = getCameraRay(x, y, cri);
		//rays.difx = getCameraRay(rays.mainRay, -cri.left);
		//rays.dify = Ray(cyPoint3f(0, 0, 0), cyPoint3f(0, .5, 0)); //getCameraRayDY(rays.mainRay, cri.up);
		rays.difx = getCameraRay(x + .5, y, cri);
		rays.dify = getCameraRay(x, y + .5, cri);


		HitInfo info;
		Trace(rays, rootNode, info, HIT_FRONT);
		
		Color24 pixelColor = Color24();
		pixelColor.SetBlack();
		/* If we hit something */
		if (info.node) {
			const Material *hitMat = info.node->GetMaterial();
			if (showNormals) {
				pixelColor.r = (info.N.x + 1.0f) * 128;
				pixelColor.g = (info.N.y + 1.0f) * 128;
				pixelColor.b = (info.N.z + 1.0f) * 128;
			}
			else
				pixelColor = Color24(hitMat->Shade(rays, info, lights, totalBounces));
		}
		/* Otherwise sample from the background */
		else {
			cyPoint3f uvw;
			uvw.x = x / (float)width;
			uvw.y = y / (float)height;
			uvw.z = 0;
			pixelColor = Color24(background.Sample(uvw));
		}

		img[x + y*width] = pixelColor;
		zBuffer[x + y*width] = info.z;

		renderImage.IncrementNumRenderPixel(1);
	}
}

static bool multithreaded = true;
void render() {
	renderImage.ResetNumRenderedPixels();
	pixelsDone = 0;

	std::thread t1 = std::thread(render_internal);
	if (multithreaded) {
		std::thread t2 = std::thread(render_internal);
		std::thread t3 = std::thread(render_internal);
		std::thread t4 = std::thread(render_internal);
		t2.join();
		t3.join();
		t4.join();
	}
	t1.join();
	
	
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

void Trace(Rays rays, Node &n, HitInfo &info, int hitSide) {
	const Box &bb = rootNode.GetChildBoundBox();
	if (bb.IntersectRay(rays.mainRay, BIGFLOAT)) {
		Trace_internal(rays, n, info, hitSide);
	}
}


void Trace_internal(Rays rays, Node &n, HitInfo &info, int hitSide) {
	Rays newRays;
	
	newRays.mainRay = n.ToNodeCoords(rays.mainRay);
	newRays.difx = n.ToNodeCoords(rays.difx);
	newRays.dify = n.ToNodeCoords(rays.dify);
	
	/* If this node has an obj */
	if (n.GetNodeObj()) {
		Box bb = n.GetNodeObj()->GetBoundBox();
		if (bb.IntersectRay(newRays.mainRay, info.z)) {
			float oldZ = info.z;
			n.GetNodeObj()->IntersectRay(newRays, info, hitSide);
			/* If the z value decreased, this node is the closest node that the ray hit */
			if (info.z < oldZ) {
				info.node = &n;
			}
		}
	} 

	/* Recurse through the children */
	const Box &bb = n.GetChildBoundBox();
	if (bb.IntersectRay(rays.mainRay, BIGFLOAT)) {
		for (int i = 0; i < n.GetNumChild(); ++i) {
			const Node *previous = info.node;
			Trace_internal(newRays, *n.GetChild(i), info, hitSide);
			//bb.IntersectRay(r2, BIGFLOAT);
			if (info.node != previous) {
				n.GetChild(i)->FromNodeCoords(info);
			}
		}
	}
}
