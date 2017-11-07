#include "options.h"
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

void render_internal() {
	Color24* img = renderImage.GetPixels();
	float* zBuffer = renderImage.GetZBuffer();
	uchar* sampleBuffer = renderImage.GetSampleCount();
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	const CommonRayInfo cri = getCommonCameraRayInfo();

	float maxVariance = 0;

	while (pixelsDone < width * height) {
		/* Get pixel location*/
		int pixel = std::atomic_fetch_add(&pixelsDone, 1);
		int x = pixel / height, y = pixel % height;
		
		float meanZ = 0;
		Color mean = Color::Black(), m2 = Color::Black();
		unsigned int totalSamples = MIN_SAMPLES;

		/* Use Halton Sequence for antialiasing. */
		for (int i = 1; i <= totalSamples; i++) {
			float ddx = Halton(i, 2) * RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5);
			float ddy = Halton(i, 3) * RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5);
			float t = 2 * M_PI*Halton(i, 2);
			float u = Halton(i, 3) + Halton(i, 3);
			float r = (u > 1.0) ? 2 - u : u;
			float dpx = r*cos(t) * camera.dof;
			float dpy = r*sin(t) * camera.dof;

			Rays rays;
			rays.mainRay = getCameraRay(x, y, ddx, ddy, dpx, dpy, cri);
			rays.difx = getCameraRay(x, y, ddx + .5 * RECONSTRUCTION_FILTER_SIZE, ddy, dpx, dpy, cri);
			rays.dify = getCameraRay(x, y, ddx, ddy + .5 * RECONSTRUCTION_FILTER_SIZE, dpx, dpy, cri);

			HitInfo info = HitInfo();
			Trace(rays, rootNode, info, HIT_FRONT_AND_BACK);

			Color pixelColor = Color();
			pixelColor.SetBlack();
			/* If we hit something */
			if (info.node) {
				const Material *hitMat = info.node->GetMaterial();
				if (SHOW_NORMALS) {
					pixelColor.r = (info.N.x + 1.0f);
					pixelColor.g = (info.N.y + 1.0f);
					pixelColor.b = (info.N.z + 1.0f);
				}
				else
					pixelColor = hitMat->Shade(rays, info, lights, TOTAL_BOUNCES);
			}
			/* Otherwise sample from the background */
			else {
				cyPoint3f uvw;
				uvw.x = x / (float)width;
				uvw.y = y / (float)height;
				uvw.z = 0;
				pixelColor = background.Sample(uvw);
			}

			/* Online Variance */
			meanZ = info.z;

			Color delta = pixelColor - mean;
			mean += delta / (float)i;
			Color delta2 = pixelColor - mean;
			m2 += delta * delta2;

			if (i != 1) {
				Color currentVariance = m2 / (float)(i - 1.0);
				
				maxVariance = MAX(currentVariance.Gray(), maxVariance);
				if (currentVariance.Gray() > MIN_THRESHOLD) {
					float temp = MIN(currentVariance.Gray(), MAX_THRESHOLD);
					temp -= MIN_THRESHOLD;
					temp /= (MAX_THRESHOLD - MIN_THRESHOLD);
					totalSamples = MIN_SAMPLES + (MAX_SAMPLES - MIN_SAMPLES) * temp;
				}
			}
		}

		img[x + y*width] = Color24(mean);
		zBuffer[x + y*width] = meanZ;
		sampleBuffer[x + y*width] = ((totalSamples - MIN_SAMPLES) / (float)(MAX_SAMPLES - MIN_SAMPLES)) * 256;

		renderImage.IncrementNumRenderPixel(1);
	}

}

static std::vector<std::thread> threads;
void render() {
	renderImage.ResetNumRenderedPixels();
	pixelsDone = 0;

	 threads = std::vector<std::thread>(TOTAL_THREADS);

	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i) = std::thread(render_internal);
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
}



// Called to start rendering (renderer must run in a separate thread)
void BeginRender() {
	/* Render asynchronously */
	renderThread = new std::thread(render);
}

// Called to end rendering (if it is not already finished)
void StopRender() {
	pixelsDone = camera.imgWidth * camera.imgHeight;
}

void Trace(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
	const Box &bb = rootNode.GetChildBoundBox();
	if (bb.IntersectRay(rays.mainRay, t_max)) {
		Trace_internal(rays, n, info, hitSide, t_max);
	}
}
void Trace_internal(Rays rays, Node &n, HitInfo &info, int hitSide, float t_max) {
	Rays newRays;
	
	newRays.mainRay = n.ToNodeCoords(rays.mainRay);
	newRays.difx = n.ToNodeCoords(rays.difx);
	newRays.dify = n.ToNodeCoords(rays.dify);
	
	/* If this node has an obj */
	if (n.GetNodeObj()) {
		Box bb = n.GetNodeObj()->GetBoundBox();
		if (bb.IntersectRay(newRays.mainRay, MAX(info.z, t_max))) {
			float oldZ = info.z;
			n.GetNodeObj()->IntersectRay(newRays, info, hitSide);
			/* If the z value decreased, this node is the closest node that the ray hit */
			if (info.z < oldZ) {
				info.node = &n;
				/* Shadow rays can quit tracing at this point. */
				if (info.shadow) 
					return;
			}
		}
	} 

	/* Recurse through the children */
	const Box &bb = n.GetChildBoundBox();
	if (bb.IntersectRay(rays.mainRay, MAX(info.z, t_max))) {
		for (int i = 0; i < n.GetNumChild(); ++i) {
			const Node *previous = info.node;
			Trace_internal(newRays, *n.GetChild(i), info, hitSide, MAX(info.z, t_max));

			/* If this is a shadow trace and we hit something, return. */
			if (info.shadow && info.z != BIGFLOAT) 
				return;

			//bb.IntersectRay(r2, BIGFLOAT);
			if (info.node != previous) {
				n.GetChild(i)->FromNodeCoords(info);
			}
		}
	}
}
