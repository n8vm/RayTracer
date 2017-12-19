#include "options.h"
#include "render.h"
#include <thread>
#include <mutex>
#include "scene.h"
#include "lights.h"
#include "materials.h"
#include "trace.h"
#include "morton.h"
#include <time.h>

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;
extern TexturedColor background;
extern TexturedColor environment;

IrradianceMap *irradianceMap;
cy::PhotonMap *photonMap;
cy::PhotonMap *refractionMap;
cy::PhotonMap *reflectionMap;
HaltonIDX haltonIDX[TOTAL_THREADS] = {};

std::thread *renderThread;
std::atomic_int recordedPhotons;
std::atomic_int pixelsDone;
std::mutex g_display_mutex;
static std::vector<std::thread> threads;
std::atomic_bool rendering;

/* CAMERA CODE */
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
uint32_t calcZOrder(uint16_t xPos, uint16_t yPos)
{
	static const uint32_t MASKS[] = { 0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF };
	static const uint32_t SHIFTS[] = { 1, 2, 4, 8 };

	uint32_t x = xPos;  // Interleave lower 16 bits of x and y, so the bits of x
	uint32_t y = yPos;  // are in the even positions and bits from y in the odd;

	x = (x | (x << SHIFTS[3])) & MASKS[3];
	x = (x | (x << SHIFTS[2])) & MASKS[2];
	x = (x | (x << SHIFTS[1])) & MASKS[1];
	x = (x | (x << SHIFTS[0])) & MASKS[0];

	y = (y | (y << SHIFTS[3])) & MASKS[3];
	y = (y | (y << SHIFTS[2])) & MASKS[2];
	y = (y | (y << SHIFTS[1])) & MASKS[1];
	y = (y | (y << SHIFTS[0])) & MASKS[0];

	const uint32_t result = x | (y << 1);
	return result;
}
int getZPixel(int pixelIdx, int numLevels, int &x, int &y) {
	int zIdx = flipLevels(pixelIdx, numLevels);
	x = DecodeMorton2X(zIdx);
	y = DecodeMorton2Y(zIdx);
	return zIdx;
}
void getPixel(int pixelIdx, int width, int &x, int &y) {
	x = pixelIdx % width;
	y = pixelIdx / width;
}


/* GENERAL RENDER */
int sppmItteration = 1;
std::vector<std::vector<HitInfo>> hitPoints;
std::mutex hitPointMutex;

int debug = 0;
std::vector<PixelStatistics> sharedStatistics;

void render_pixel(int tid, int x, int y, Color &avgColor, int &totalSamples, float &depth, cyPoint3f &normal, IlluminationType directType, IlluminationType indirectType) {
	float maxVariance = 0;
	const CommonRayInfo cri = getCommonCameraRayInfo(); // FIX THIS
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();

	/* Initialize online variance */
	avgColor = Color::Black();
	Color avg2 = Color::Black();
	totalSamples = MIN_SAMPLES;


	PixelStatistics &statistics = sharedStatistics[x + width * y];


	/* For each sample being taken of this pixel */
	for (int i = 1; i <= /*(USE_SPPM) ? 1 :*/ totalSamples; i++) {
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
			else if (USE_SPPM) {
				statistics.totalPhotons = sppmItteration * TOTAL_PHOTONS;
				statistics.totalRefractivePhotons = sppmItteration * TOTAL_REFRACTION_PHOTONS;
				statistics.totalReflectivePhotons = sppmItteration * TOTAL_REFLECTION_PHOTONS;
				hitMat->Shade(rays, info, lights, TOTAL_BOUNCES, statistics, IlluminationType::PATH_TRACING, IlluminationType::PHOTON);
				statistics.additionalFlux /= (float)statistics.totalHits;
				statistics.additionalReflectiveFlux /= (float)statistics.totalReflHits;
				statistics.additionalRefractiveFlux /= (float)statistics.totalRefrHits;
				statistics.Update();

				sampleColor = statistics.directContribution;
				if (statistics.totalHits != 0)
					sampleColor += (statistics.indirectContribution / (float)statistics.totalHits);
				if (statistics.totalRefrHits != 0)
					sampleColor += (statistics.indirectRefractionContribution / (float)statistics.totalRefrHits);
				if (statistics.totalReflHits != 0)
					sampleColor += (statistics.indirectReflectionContribution / (float)statistics.totalReflHits);
				statistics.totalHits = 0;
				statistics.totalRefrHits = 0;
				statistics.totalReflHits = 0;
			}
			else sampleColor = hitMat->Shade(rays, info, lights, TOTAL_BOUNCES, statistics, directType, indirectType);
			//statistics.Update();
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
void render_internal(int tid, bool fillEmpty = true) {
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	int widthPow2 = nextPow2(width);
	int heightPow2 = nextPow2(height);

	int levels = log2(widthPow2);

	int renderHeight = (RENDER_SUBIMAGE) ? YEND - YSTART : height;
	//haltonIDX[tid] = HaltonIDX(); // Now done outside this function
	fillEmpty = false;
	/* While there are pixels to render */
	while (pixelsDone < widthPow2 * widthPow2) {

		/* Get pixel location*/
		int pixel = std::atomic_fetch_add(&pixelsDone, 1);
		int x, y;
		int zIdx = getZPixel(pixel, levels, x, y);
		if (x >= width || y >= height) continue;

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
		render_pixel(tid, x, y, avgColor, totalSamples, depth, normal, DIRECT_TYPE, (USE_IRRADIANCE_CACHE) ? IlluminationType::NO_ILLUMINATION : INDIRECT_TYPE);

#if USE_IRRADIANCE_CACHE
		if (depth < BIGFLOAT) avgColor += irradianceMap->Sample(x, y).c;
#endif

		/* Do gamma correction */
		Color correctedColor;
		correctedColor.r = powf((avgColor.r), (1.0 / GAMMA));
		correctedColor.g = powf((avgColor.g), (1.0 / GAMMA));
		correctedColor.b = powf((avgColor.b), (1.0 / GAMMA));

		int pixelsPerDim = pow(2, levels - getLevel(zIdx, levels));
		
		/* Write data to image */
		if (fillEmpty)
			for (int yi = y; yi < y + pixelsPerDim; ++yi) {
				if (yi >= height) continue;
				for (int xi = x; xi < x + pixelsPerDim; ++xi) {
					if (xi >= width) continue;
					renderImage.GetPixels()[xi + yi*width] = Color24(correctedColor);
				}
			}
		else renderImage.GetPixels()[x + y*width] = Color24(correctedColor);

		renderImage.GetZBuffer()[x + y*width] = depth;
		renderImage.GetSampleCount()[x + y*width] = ((totalSamples - MIN_SAMPLES) / (float)(MAX_SAMPLES - MIN_SAMPLES)) * 256;
		
		if (pixel != 0)
			renderImage.IncrementNumRenderPixel(1);
		
		/* Progress report */
		if ((pixelsDone % 10) == 0) {
			g_display_mutex.lock();
			std::cout << "\rImage Render: " << 100 * pixelsDone / (float)(widthPow2 * widthPow2) << " percent completed    ";
			g_display_mutex.unlock();
		}
	}
	renderImage.ResetNumRenderedPixels();

}

/* IRRADIANCE CACHING */
void computeIrradiance(int tid) {
	g_display_mutex.lock();
	std::cout << "tid " << tid << " computing irradiance map" << std::endl;
	g_display_mutex.unlock();
	while (irradianceMap->ComputeNextPoint(tid));
}

/* PHOTON MAPPING */
void initPhotonRays(int tid, cy::PhotonMap *map, int totalPhotons, float scale, MapType mapType, std::string consoleKey) {
	using namespace std;
	recordedPhotons = 0;

	/* Create a sublist of all lights which emit photons */
	vector<Light*> photonLights;
	for (int i = 0; i < lights.size(); ++i) {
		if (lights.at(i)->IsPhotonSource()) photonLights.push_back(lights.at(i));
	}

	vector<float> intensities;
	vector<float> prefixSum;
	vector<float> probabilities;
	prefixSum.push_back(0);
	for (int l = 0; l < photonLights.size(); ++l) {
		intensities.push_back(photonLights.at(l)->GetPhotonIntensity().Gray());
		prefixSum.push_back(prefixSum.at(l) + intensities.at(l));
	}
	for (int l = 0; l < photonLights.size(); ++l) {
		probabilities.push_back(intensities[l] / prefixSum[prefixSum.size() - 1]);
	}
	srand(time(NULL));
	while (recordedPhotons < totalPhotons) {
		/* Create a random number between 0 and the sum of all intensities.*/
		float r = (rand() / (float)(RAND_MAX)) * prefixSum.at(prefixSum.size() - 1);

		/* Determine what light the random number goes to. */
		int lightIdx = 0;
		while (lightIdx < prefixSum.size() && r < prefixSum[lightIdx])
			++lightIdx;

		/* Create an initial photon */
		debug = 1;
		BounceInfo bInfo;
		bInfo.ray = photonLights[lightIdx]->GenerateRandomPhoton(tid);
		bInfo.power = photonLights[lightIdx]->GetPhotonIntensity() / probabilities[lightIdx];
		bInfo.bounceType = BounceType::NONE;
		bInfo.mapType = mapType;
		bInfo.map = map;

		/* Trace that photon */
		for (bInfo.bounceIdx = 0; bInfo.bounceIdx < PHOTON_BOUNCES; ++bInfo.bounceIdx) {
			HitInfo hitInfo = HitInfo(tid);
			bool hit = Trace(bInfo.ray, rootNode, hitInfo, HIT_FRONT_AND_BACK, BIGFLOAT);

			//std::cout << "ray " << bInfo.ray.p.x << " " << bInfo.ray.p.y << " " << bInfo.ray.p.z << " " << std::endl;
		//	std::cout << "ray " << bInfo.ray.dir.x << " " << bInfo.ray.dir.y << " " << bInfo.ray.dir.z << " " << std::endl;
		//	std::cout << "hit " << hit << std::endl;

			/* If our photon flew away, get a new one. */
			if (!hit) break;

			/* Otherwise, bounce the photon off the material it hit. */
			bool bounced = hitInfo.node->GetMaterial()->BouncePhoton(bInfo, hitInfo);
			if (bInfo.photonRecorded) {
				recordedPhotons++;
				if (tid == 0 && recordedPhotons % 10 == 0) std::cout << "\r" << consoleKey << ": " << recordedPhotons<<" / " << totalPhotons << " percent done    ";
			}
			if (!bounced) break;
		}
	}

	if (tid == 0) {
		std::cout << std::endl;
#if !USE_SPPM
		map->ScalePhotonPowers(scale / (float)totalPhotons);
#endif
		if (mapType == MapType::GLOBAL_ILLUMINATION) {
			FILE *fp = fopen("photonmap.dat", "wb");
			fwrite(photonMap->GetPhotons(), sizeof(cyPhotonMap::Photon), photonMap->NumPhotons(), fp);
			fclose(fp);
		}

		if (mapType == MapType::CAUSTIC_REFLECTIONS) {
			FILE *fp = fopen("reflmap.dat", "wb");
			fwrite(reflectionMap->GetPhotons(), sizeof(cyPhotonMap::Photon), reflectionMap->NumPhotons(), fp);
			fclose(fp);
		}

		if (mapType == MapType::CAUSTIC_REFRACTIONS) {
			FILE *fp = fopen("refrmap.dat", "wb");
			fwrite(refractionMap->GetPhotons(), sizeof(cyPhotonMap::Photon), refractionMap->NumPhotons(), fp);
			fclose(fp);
		}
	}
}
int readPhotonMap(std::string fname, cy::PhotonMap *map) {
	int numPhotons = 0;
	/////////////////////////////////////////////////////////////////////////////////
	// Read the photon map
	/////////////////////////////////////////////////////////////////////////////////
	FILE *fp = fopen(fname.c_str(), "rb");
	if (fp == NULL) {
		printf("ERROR: Cannot open file \"%s\".\n", fname.c_str());
		return -1;
	}
	int n = 0;
	cy::PhotonMap::Photon buffer;
	for (; !feof(fp); n++) {
		fread(&buffer, sizeof(cy::PhotonMap::Photon), 1, fp);
	}
	n--;

	if (n <= 0) {
		printf("ERROR: No photons found.\n");
	}
	else {
		map->AllocatePhotons(n);

		//photons = new Photon[n];
		rewind(fp);
		int np = fread(map->GetPhotons(), sizeof(cy::PhotonMap::Photon), n, fp);
		numPhotons = np;
		//printf("%d photons read.\n", np);
	}
	fclose(fp);
	if (n <= 0) return -2;

	std::cout << "Balancing " + fname + " Photon Map" << std::endl;
	map->PrepareForIrradianceEstimation();

}
void renderPhotons(int tid) {
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	int width2 = nextPow2(width);
	int height2 = nextPow2(height);
	int levels = log2(width2);

	while (pixelsDone < width2 * height2) {
		int idx = std::atomic_fetch_add(&pixelsDone, 1);
		int x, y;
		int zIdx = getZPixel(idx, levels, x, y);
		if (x >= width || y >= height) continue;

		std::vector<HitInfo> &pixelHitPts = hitPoints.at(x + y*width);
		PixelStatistics &stats = sharedStatistics[x + y * width];



		for (int i = 0; i < pixelHitPts.size(); ++i) {
			HitInfo &info = pixelHitPts[i];
			stats.totalPhotons = sppmItteration * TOTAL_PHOTONS;
			info.node->GetMaterial()->ShadePoint(info, stats);
		}
		if (stats.itteration == 1)
			stats.totalFlux /= pixelHitPts.size();
		stats.additionalFlux /= pixelHitPts.size();
		stats.Update();

		Color correctedColor = stats.directContribution + (stats.indirectContribution / (float) pixelHitPts.size());
		correctedColor.r = powf(correctedColor.r, (1.0 / GAMMA));
		correctedColor.g = powf(correctedColor.g, (1.0 / GAMMA));
		correctedColor.b = powf(correctedColor.b, (1.0 / GAMMA));
		//pixelColor *= stats.px_weight;
		//pixelColor *= (float)PHOTON_ALPHA;

		if (sppmItteration == 1) {
			int pixelsPerDim = pow(2, levels - getLevel(zIdx, levels));
			/* Write data to image */
			for (int yi = y; yi < y + pixelsPerDim; ++yi) {
				if (yi >= height) continue;
				for (int xi = x; xi < x + pixelsPerDim; ++xi) {
					if (xi >= width) continue;
					renderImage.GetPixels()[xi + yi*width] = Color24(correctedColor);
				}
			}
		}
		else 
			renderImage.GetPixels()[stats.x + stats.y*width] = Color24(correctedColor);
		if (idx != 0)
			renderImage.IncrementNumRenderPixel(1);
	}
	renderImage.ResetNumRenderedPixels();
}
std::vector<HitInfo> getPixelDiffuseHits(int tid, int x, int y, int numSamples, IlluminationType directType, IlluminationType indirectType) {
	std::vector<HitInfo> localHitPoints;
	const CommonRayInfo cri = getCommonCameraRayInfo();
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	
	for (int i = 0; i < numSamples; ++i) {

		/* Determine the ray direction's random shift relative to the center of the pixel */
		int aaIdx = haltonIDX[tid].AAidx();
		float ddx = Halton(sppmItteration + ((x * y) % 1024), 7) * (RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5));
		float ddy = Halton(sppmItteration + ((x * y) % 1024), 5) * (RECONSTRUCTION_FILTER_SIZE - (RECONSTRUCTION_FILTER_SIZE * 0.5));
		
		
		if (x == PICKED_X && y == PICKED_Y)
 			std::cout << "itteration: " << sppmItteration << std::endl;

		/* Determine the ray point's random shift relative to the eye (a disk modeling a lense) */
		int dofIdx = haltonIDX[tid].DOFidx();
		float t = 2 * M_PI*Halton(dofIdx, 2);
		float u = Halton(dofIdx, 3) + Halton(dofIdx, 5);
		float r = (u > 1.0) ? 2 - u : u;
		float dpx = r*cos(t) * camera.dof;
		float dpy = r*sin(t) * camera.dof;

		/* Generate the ray */
		Rays rays = getCameraRays(x, y, ddx, ddy, dpx, dpy, cri);

		/* Shoot that ray into the scene */
		HitInfo info = HitInfo(tid);
		Color sampleColor = Color::Black();
		bool hit = Trace(rays, rootNode, info, HIT_FRONT_AND_BACK, BIGFLOAT);


		/* If we didn't hit anything at this point, we can return.*/
		if (!hit) continue;

		/* Get the diffuse hitpoints throughout the scene using the material. */
		info.node->GetMaterial()->GetDiffuseHits(localHitPoints, rays.mainRay, info, TOTAL_BOUNCES);
	}
	return localHitPoints;
}
void getDiffuseHitPoints(int tid, bool initSharedStatistics = true, int numSamples = 1) {
	const int width = renderImage.GetWidth(), height = renderImage.GetHeight();
	int width2 = nextPow2(width), height2 = nextPow2(height);
	int levels = log2(width2);

	while (pixelsDone < width * height) {
		/* Get the pixel */
		int pixel = std::atomic_fetch_add(&pixelsDone, 1), x = 0, y = 0;
		getPixel(pixel, width, x, y);
		if (x >= width || y >= height) continue;

		/* Get all non-specular hit points for that pixel */
		std::vector<HitInfo> pixelHitPoints = getPixelDiffuseHits(tid, x, y, numSamples, DIRECT_TYPE, INDIRECT_TYPE);
		
		/* Add those hitpoints to the list. */
		hitPoints[pixel] = pixelHitPoints;
		if (initSharedStatistics) {
			sharedStatistics[pixel].R = PHOTON_SPHERE_RADIUS;
			sharedStatistics[pixel].totalFlux = Color::Black();
			sharedStatistics[pixel].SharedLocalPhotonCount = 0;
			sharedStatistics[pixel].x = x;
			sharedStatistics[pixel].y = y;
			sharedStatistics[pixel].px_weight = 1.0 / (float)numSamples;
		}
	}
}
void render_sppm() {
	std::cout << "Beginning render" << std::endl;

	rendering = true;

	/* Initialize the image as black */
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	std::fill(renderImage.GetPixels(), renderImage.GetPixels() + (width * height & sizeof(Color24)), Color24::Black());

	/* Reserve the shared statistics */
	//hitPoints.resize(width*height);
	sharedStatistics.resize(width*height);

	for (int i = 0; i < TOTAL_THREADS; ++i) haltonIDX[i] = HaltonIDX();

	while (rendering) {
		/* Photon pass */
		std::cout << "Pass " << sppmItteration << std::endl;
		std::cout << "Photon Mapping " << sppmItteration << std::endl;
		photonMap = new cyPhotonMap();
		reflectionMap = new cyPhotonMap();
		refractionMap = new cyPhotonMap();
		if (USE_PHOTON_MAPPING) {
			photonMap->AllocatePhotons(TOTAL_PHOTONS);
			threads = std::vector<std::thread>(TOTAL_THREADS);
			for (int i = 0; i < TOTAL_THREADS; ++i)
				threads.at(i) = std::thread(initPhotonRays, i, photonMap, TOTAL_PHOTONS, PHOTON_SCALE, MapType::GLOBAL_ILLUMINATION, "Photon Map");
			for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();
			photonMap->PrepareForIrradianceEstimation();
		}
		if (USE_CAUSTIC_REFLECTIONS) {
			reflectionMap->AllocatePhotons(TOTAL_REFLECTION_PHOTONS);
			threads = std::vector<std::thread>(TOTAL_THREADS);
			for (int i = 0; i < TOTAL_THREADS; ++i)
				threads.at(i) = std::thread(initPhotonRays, i, reflectionMap, TOTAL_REFLECTION_PHOTONS, CAUSTIC_REFRACTION_SCALE, MapType::CAUSTIC_REFLECTIONS, "Reflective Caustic Map");
			for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();
			reflectionMap->PrepareForIrradianceEstimation();
		}
		if (USE_CAUSTIC_REFRACTIONS) {
			refractionMap->AllocatePhotons(TOTAL_REFRACTION_PHOTONS);
			threads = std::vector<std::thread>(TOTAL_THREADS);
			for (int i = 0; i < TOTAL_THREADS; ++i)
				threads.at(i) = std::thread(initPhotonRays, i, refractionMap, TOTAL_REFRACTION_PHOTONS, CAUSTIC_REFRACTION_SCALE, MapType::CAUSTIC_REFRACTIONS, "Refractive Caustic Map");
			for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();
			refractionMap->PrepareForIrradianceEstimation();
		}


		/* Ray trace pass */
		std::cout << "Ray Tracing " << sppmItteration << std::endl;
		pixelsDone = 0;
		threads = std::vector<std::thread>(TOTAL_THREADS);
		for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i) = std::thread(render_internal, i, (sppmItteration == 1));
		for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();

		///* Update View */
		//renderImage.ResetNumRenderedPixels();
		//pixelsDone = 0;
		//threads = std::vector<std::thread>(TOTAL_THREADS);
		//for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i) = std::thread(renderPhotons, i);
		//for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();

		///* Distributed Ray Tracing Pass */
		//pixelsDone = 0;
		//threads = std::vector<std::thread>(TOTAL_THREADS);
		//for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i) = std::thread(getDiffuseHitPoints, i, false, MAX_SAMPLES);
		//for (int i = 0; i < TOTAL_THREADS; ++i) threads.at(i).join();

		sppmItteration++;
		delete(photonMap);
		delete(reflectionMap);
		delete(refractionMap);
		renderImage.SaveImage("Image.png");
	}
}

/* PATH TRACING */
void render_pt() {
	std::cout << "Beginning render" << std::endl;

	renderImage.ResetNumRenderedPixels();
	pixelsDone = 0;

	/* Reserve the shared statistics */
	//hitPoints.resize(width*height);


//#if USE_PHOTON_MAPPING || USE_CAUSTIC_REFRACTIONS || USE_CAUSTIC_REFLECTIONS
//	photonMap = new cyPhotonMap();
//	reflectionMap = new cyPhotonMap();
//	refractionMap = new cyPhotonMap();
//
//	if (USE_PHOTON_MAPPING && USE_CACHED_PHOTON_MAP) {
//		readPhotonMap("photonmap.dat", photonMap);
//		std::cout << "Photon Map: 100 percent" << std::endl;
//	}
//	else if (USE_PHOTON_MAPPING) {
//		photonMap->AllocatePhotons(TOTAL_PHOTONS);
//
//		threads = std::vector<std::thread>(TOTAL_THREADS);
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i) = std::thread(initPhotonRays, i, photonMap, TOTAL_PHOTONS, PHOTON_SCALE, MapType::GLOBAL_ILLUMINATION, "Photon Map");
//		}
//
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i).join();
//			threads.at(i).~thread();
//		}
//		std::cout << "Balancing Refractive Photon Map" << std::endl;
//		photonMap->PrepareForIrradianceEstimation();
//	}
//
//	if (USE_CAUSTIC_REFLECTIONS && USE_CACHED_REFLECTIONS) {
//		readPhotonMap("reflmap.dat", reflectionMap);
//		std::cout << "Reflection Map: 100 percent" << std::endl;
//	}
//	else if (USE_CAUSTIC_REFLECTIONS) {
//		reflectionMap->AllocatePhotons(TOTAL_PHOTONS);
//
//		threads = std::vector<std::thread>(TOTAL_THREADS);
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i) = std::thread(initPhotonRays, i, reflectionMap, TOTAL_REFLECTION_PHOTONS, CAUSTIC_REFLECTION_SCALE, MapType::CAUSTIC_REFLECTIONS, "Reflection Map");
//		}
//
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i).join();
//			threads.at(i).~thread();
//		}
//		std::cout << "Balancing Refractive Photon Map" << std::endl;
//		reflectionMap->PrepareForIrradianceEstimation();
//	}
//
//	if (USE_CAUSTIC_REFRACTIONS && USE_CACHED_REFRACTIONS) {
//		readPhotonMap("refrmap.dat", refractionMap);
//		std::cout << "Refraction Map: 100 percent" << std::endl;
//	}
//	else if (USE_CAUSTIC_REFRACTIONS) {
//		refractionMap->AllocatePhotons(TOTAL_PHOTONS);
//
//		threads = std::vector<std::thread>(TOTAL_THREADS);
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i) = std::thread(initPhotonRays, i, refractionMap, TOTAL_REFRACTION_PHOTONS, CAUSTIC_REFRACTION_SCALE, MapType::CAUSTIC_REFRACTIONS, "Refraction Map");
//		}
//
//		for (int i = 0; i < TOTAL_THREADS; ++i) {
//			threads.at(i).join();
//			threads.at(i).~thread();
//		}
//		std::cout << "Balancing Refractive Photon Map" << std::endl;
//		refractionMap->PrepareForIrradianceEstimation();
//	}
//#endif
//
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

	rendering = true;

	/* Initialize the image as black */
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	std::fill(renderImage.GetPixels(), renderImage.GetPixels() + (width * height & sizeof(Color24)), Color24::Black());

	sharedStatistics.resize(width*height);
	for (int i = 0; i < TOTAL_THREADS; ++i) haltonIDX[i] = HaltonIDX();

	threads = std::vector<std::thread>(TOTAL_THREADS);
	for (int i = 0; i < TOTAL_THREADS; ++i) {
		threads.at(i) = std::thread(render_internal, i, true);
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

	delete irradianceMap;
	delete photonMap;
	delete refractionMap;
	delete reflectionMap;
}

/* GUI */
void BeginRender() {
	/* Render asynchronously */
	rendering = true;

#if USE_SPPM
	renderThread = new std::thread(render_sppm);
#else
	renderThread = new std::thread(render_pt);
#endif
}
void StopRender() {
	rendering = false;
	int widthPow2 = nextPow2(camera.imgWidth);
	int heightPow2 = nextPow2(camera.imgHeight);

	pixelsDone = widthPow2 * heightPow2;

	renderImage.ComputeZBufferImage();
	renderImage.ComputeSampleCountImage();
	renderImage.SaveImage("Image.png");
	renderImage.SaveZImage("zImage.png");
	renderImage.SaveSampleCountImage("SImage.png");
}

