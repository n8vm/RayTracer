#include "options.h"
#include "render.h"
#include <thread>
#include <mutex>
#include "scene.h"
#include "lights.h"

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;
extern TexturedColor background;
extern TexturedColor environment;

IrradianceMap *irradianceMap;
HaltonIDX haltonIDX[TOTAL_THREADS] = {};

std::thread *renderThread;
std::atomic_int pixelsDone;
std::mutex g_display_mutex;
static std::vector<std::thread> threads;

/* CAMERA STUFF */
CommonRayInfo getCommonCameraRayInfo() {
	CommonRayInfo cri;
	cri.startPos = camera.pos;

	/* Camera reference frame */
	cri.forward = camera.dir;
	cri.up = camera.up;
	cri.left = camera.up.Cross(camera.dir).GetNormalized();

	/* height and width of near plane */
	float radfov = camera.fov * 3.141592653589793 / 180.0;
	cri.l = camera.focaldist;
	cri.h = tan(radfov * .5f) * (2.0f * cri.l);
	cri.w = cri.h * camera.imgWidth / (float)camera.imgHeight;

	/* All rays start at top left */
	cyPoint3f B = camera.pos + (cri.l * camera.dir) + (cri.h / 2.0f * cri.up);
	cri.nearPlaneTopLeft = B + cri.w / 2.0f * (cri.left);
	cri.dXPixel = -cri.left * (cri.w / (float)camera.imgWidth);
	cri.dYPixel = -cri.up * (cri.w / (float)camera.imgWidth);

	return cri;
}
Ray getCameraRay(float i, float j, float ddi, float ddj, float dpi, float dpj, const CommonRayInfo &cri) {
	Ray r;
	r.p = cri.startPos;
	r.p += cri.left * dpi + cri.up * dpj;
	r.dir = (cri.nearPlaneTopLeft + (i + ddi) * cri.dXPixel + (j + ddj) * cri.dYPixel) - r.p;
	r.dir.Normalize();

	// For speeding up box intersections
	r.dir_inv = invertPoint(r.dir);
	return r;
}
Rays getCameraRays(float x, float y, float ddx, float ddy, float dpx, float dpy, const CommonRayInfo &cri) {
	Rays rays;
	rays.mainRay = getCameraRay(x, y, ddx, ddy, dpx, dpy, cri);
	rays.difx = getCameraRay(x, y, ddx + .5 * RECONSTRUCTION_FILTER_SIZE, ddy, dpx, dpy, cri);
	rays.dify = getCameraRay(x, y, ddx, ddy + .5 * RECONSTRUCTION_FILTER_SIZE, dpx, dpy, cri);
	return rays;
}

/* RENDER */
void render_pixel(int tid, int x, int y, Color &avgColor, int &totalSamples, float &depth, cyPoint3f &normal, bool use_indirect) {
	float maxVariance = 0;
	const CommonRayInfo cri = getCommonCameraRayInfo(); // FIX THIS
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();

	/* Initialize online variance */
	avgColor = Color::Black();
	Color avg2 = Color::Black();
	totalSamples = MIN_SAMPLES;

	/* For each sample being taken of this pixel */
	for (int i = 1; i <= totalSamples; i++) {
		/* Determine the ray direction's random shift relative to the center of the pixel */
		int aaIdx = haltonIDX[tid].AAidx();
		float ddx = Halton(aaIdx, 2) * RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5);
		float ddy = Halton(aaIdx, 3) * RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5);

		/* Determine the ray point's random shift relative to the eye (a disk modeling a lense) */
		int dofIdx = haltonIDX[tid].DOFidx();
		float t = 2 * M_PI*Halton(dofIdx, 2);
		float u = Halton(dofIdx, 3) + Halton(dofIdx, 5);
		float r = (u > 1.0) ? 2 - u : u;
		float dpx = r*cos(t) * camera.dof;
		float dpy = r*sin(t) * camera.dof;

		/* Generate the ray */
		Rays rays = getCameraRays(x, y, ddx, ddy, dpx, dpy, cri);

		/* Shoot that ray through the scene */
		HitInfo info = HitInfo(tid);
		Color sampleColor = Color::Black();
		bool hit = Trace(rays, rootNode, info, HIT_FRONT_AND_BACK, BIGFLOAT);

		/* If we hit something, shade that point */
		if (hit) {
			const Material *hitMat = info.node->GetMaterial();
			normal = info.N;
			if (SHOW_NORMALS) { sampleColor.r = (info.N.x + 1.0f); sampleColor.g = (info.N.y + 1.0f); sampleColor.b = (info.N.z + 1.0f); }
			else sampleColor = hitMat->Shade(rays, info, lights, TOTAL_BOUNCES, use_indirect);
		}
		/* Otherwise sample from the background */
		else {
			cyPoint3f uvw; uvw.x = x / (float)width; uvw.y = y / (float)height; uvw.z = 0;
			sampleColor = Color(background.Sample(uvw));
		}

		/* Assume the last sample's depth is the pixel depth */
		depth = info.z;

		/* Compute online variance of grayscale pixel color, for adaptive sampling */
		Color diffFromAvg = sampleColor - avgColor;
		avgColor += diffFromAvg / (float)i;
		Color diffFromNewAvg = sampleColor - avgColor;
		avg2 += diffFromAvg * diffFromNewAvg;

		/* not sure if this is needed... */
		if (i == 1) continue;

		Color currentVariance = avg2 / (float)(i - 1.0);
		maxVariance = MAX(currentVariance.Gray(), maxVariance);

		/* If we're above the variance threshold, add more samples. */
		if (currentVariance.Gray() <= MIN_THRESHOLD) continue;
		float temp = MIN(currentVariance.Gray(), MAX_THRESHOLD);
		temp -= MIN_THRESHOLD;
		temp /= (MAX_THRESHOLD - MIN_THRESHOLD);
		totalSamples = MIN_SAMPLES + (MAX_SAMPLES - MIN_SAMPLES) * temp;
	}
}

void render_internal(int tid) {
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	int renderHeight = (RENDER_SUBIMAGE) ? YEND - YSTART : height;
	haltonIDX[tid] = HaltonIDX();

	/* While there are pixels to render */
	while (pixelsDone < width * height) {

		/* Get pixel location*/
		int pixel = std::atomic_fetch_add(&pixelsDone, 1);
		int x = pixel / height, y = pixel % height;

		/* If we're not rendering this portion of the image, continue. */
		if (RENDER_SUBIMAGE && (x < XSTART || y < YSTART || x >= XEND || y >= YEND)) {
			renderImage.GetPixels()[x + y*width] = Color24::Black();
			renderImage.GetZBuffer()[x + y*width] = BIGFLOAT;
			renderImage.GetSampleCount()[x + y*width] = 0;
			renderImage.IncrementNumRenderPixel(1);
			continue;
		}

		/* Render the pixel */
		Color avgColor;
		cyPoint3f normal;
		int totalSamples;
		float depth;
		render_pixel(tid, x, y, avgColor, totalSamples, depth, normal, !USE_IRRADIANCE_CACHE);

#if USE_IRRADIANCE_CACHE
		if (depth < BIGFLOAT)
			avgColor += irradianceMap->Sample(x, y).c;
#endif

		/* Do gamma correction */
		Color24 correctedColor = Color24(avgColor);
		correctedColor.r = 255.0 * pow((correctedColor.r / 255.0), (1.0 / GAMMA));
		correctedColor.g = 255.0 * pow((correctedColor.g / 255.0), (1.0 / GAMMA));
		correctedColor.b = 255.0 * pow((correctedColor.b / 255.0), (1.0 / GAMMA));

		/* Write data to image */


		renderImage.GetPixels()[x + y*width] = correctedColor;
		renderImage.GetZBuffer()[x + y*width] = depth;
		renderImage.GetSampleCount()[x + y*width] = ((totalSamples - MIN_SAMPLES) / (float)(MAX_SAMPLES - MIN_SAMPLES)) * 256;
		renderImage.IncrementNumRenderPixel(1);

		/* Progress report */
		if ((pixelsDone % renderHeight) == 0) {
			g_display_mutex.lock();
			std::cout << pixelsDone / (float)(width * height) << " percent completed " << std::endl;
			g_display_mutex.unlock();
		}
	}

}

void computeIrradiance(int tid) {
	g_display_mutex.lock();
	std::cout << "tid " << tid << " computing irradiance map" << std::endl;
	g_display_mutex.unlock();
	while (irradianceMap->ComputeNextPoint(tid));
}

void render() {
	renderImage.ResetNumRenderedPixels();
	pixelsDone = 0;


#if USE_IRRADIANCE_CACHE
	cyColorZNormal czn = cyColorZNormal();
	irradianceMap = new IrradianceMap(COLOR_THRESHOLD, Z_THRESHOLD, NORMAL_THRESHOLD);
	renderImage.AllocateIrradianceComputationImage();
	irradianceMap->Initialize(renderImage.GetWidth(), renderImage.GetHeight(), MIN_LEVEL, MAX_LEVEL);

	threads = std::vector<std::thread>(TOTAL_THREADS);
	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i) = std::thread(computeIrradiance, i);
	}

	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i).join();
		threads.at(i).~thread();
	}
#endif

	threads = std::vector<std::thread>(TOTAL_THREADS);
	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i) = std::thread(render_internal, i);
	}

	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i).join();
		threads.at(i).~thread();
	}

	renderImage.ComputeZBufferImage();
	renderImage.ComputeSampleCountImage();
	renderImage.SaveImage("Image.png");
	renderImage.SaveZImage("zImage.png");
	renderImage.SaveSampleCountImage("SImage.png");

	delete(irradianceMap);
}

/* RAY TRACE */
//void Trace(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
//	const Box &bb = rootNode.GetChildBoundBox();
//	if (bb.IntersectRay(rays.mainRay, t_max)) {
//		Trace_internal(rays, n, info, hitSide, t_max);
//	}
//}
//void Trace_internal(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
//	Rays newRays;
//	
//	newRays.mainRay = n.ToNodeCoords(rays.mainRay);
//	newRays.difx = n.ToNodeCoords(rays.difx);
//	newRays.dify = n.ToNodeCoords(rays.dify);
//	
//	/* If this node has an obj */
//	if (n.GetNodeObj()) {
//		Box bb = n.GetNodeObj()->GetBoundBox();
//		if (bb.IntersectRay(newRays.mainRay, MAX(info.z, t_max))) {
//			float oldZ = info.z;
//			n.GetNodeObj()->IntersectRay(newRays, info, hitSide);
//			/* If the z value decreased, this node is the closest node that the ray hit */
//			if (info.z < oldZ) {
//				info.node = &n;
//				/* Shadow rays can quit tracing at this point. */
//				if (info.shadow && info.z < t_max) 
//					return;
//			}
//		}
//	} 
//
//	/* Recurse through the children */
//	const Box &bb = n.GetChildBoundBox();
//	if (bb.IntersectRay(rays.mainRay, MAX(info.z, t_max))) {
//		for (int i = 0; i < n.GetNumChild(); ++i) {
//			const Node *previous = info.node;
//			Trace_internal(newRays, *n.GetChild(i), info, hitSide, MAX(info.z, t_max));
//
//			/* If this is a shadow trace and we hit something, return. */
//			if (info.shadow && info.z <= t_max) 
//				return;
//
//			//bb.IntersectRay(r2, BIGFLOAT);
//			if (info.node != previous) {
//				n.GetChild(i)->FromNodeCoords(info);
//			}
//		}
//	}
//}


//void Trace(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
//	const Box &bb = rootNode.GetChildBoundBox();
//		Trace_internal(rays, n, info, hitSide, t_max);
//	}
//}

bool Trace(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
	bool hit = false;

	/* If the given ray doesn't hit the bounding box, quit. */
	const Box &bb = n.GetChildBoundBox();
	if (!bb.IntersectRay(rays.mainRay, t_max)) return false;

	/* Otherwise, transform the main ray from parent space to child space */
	Rays transformedRays = n.ToNodeCoords(rays);

	/* If this node has an obj */
	if (n.GetNodeObj()) {
		/* If we hit that object's bounding box, test it*/
		Box bb = n.GetNodeObj()->GetBoundBox();
		if (bb.IntersectRay(transformedRays.mainRay, MIN(info.z, t_max))) {
			float oldZ = info.z;
			n.GetNodeObj()->IntersectRay(transformedRays, info, hitSide);
			/* If the z value decreased, this node is the closest node that the ray hit */
			if (info.z < oldZ) {
				info.node = &n;
				hit = true;
				/* Shadow rays can quit tracing at this point. */
				if (info.shadow && info.z < t_max) 
					return true;
			}
		}
	}

	/* Recurse through the children */
	for (int i = 0; i < n.GetNumChild(); ++i) {
		const Node *previous = info.node;
		if (!Trace(transformedRays, *n.GetChild(i), info, hitSide, MIN(info.z, t_max))) continue;
		/* If this is a shadow trace and we hit something, return. */
		else if (info.shadow && info.z < t_max) return true;
		else if (info.z < t_max) hit = true;

		/* Transform hit data from child space to parent space */
		n.GetChild(i)->FromNodeCoords(info);
	}
	return hit;
}

/* GUI */
void BeginRender() {
	/* Render asynchronously */
	renderThread = new std::thread(render);
}
void StopRender() {
	pixelsDone = camera.imgWidth * camera.imgHeight;
}

