#define CATCH_CONFIG_RUNNER
#include "Catch/catch.hpp"
#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"
#include "render.h"

using namespace std;

int LoadScene(const char *filename);
void ShowViewport();

Camera camera;
Node rootNode;
RenderImage renderImage;
Sphere theSphere;

int main(int argc, char* argv[])
{	
	int result = Catch::Session().run(argc, argv);
	
	LoadScene("test_scene1.xml");
	ShowViewport();

	return (result < 0xff ? result : 0xff);
}

SCENARIO("Camera rays can be calculated correctly", "[render]") {
	GIVEN("The scene and common ray info") {
		LoadScene("scene.xml");
		CommonRayInfo cri = getCommonCameraRayInfo();
		WHEN("Some camera rays are computed") {
			Ray r1 = getCameraRay(0, 0, cri);
			Ray r2 = getCameraRay(799, 0, cri);
			Ray r3 = getCameraRay(799, 599, cri);
			Ray r4 = getCameraRay(0, 599, cri);

			THEN("The results are valid") {
				REQUIRE( compareRay(r1, Ray({ 0.0f, 0.0f, 0.0f }, { -0.484687001f, 1.f, -0.363363594f })) == true);
				REQUIRE( compareRay(r2, Ray({ 0.0f, 0.0f, 0.0f }, { 0.484686971f,  1.f, -0.363363594f })) == true);
				REQUIRE( compareRay(r3, Ray({ 0.0f, 0.0f, 0.0f }, { 0.484686971f, 1.f, 0.363363594f })) == true);
				REQUIRE( compareRay(r4, Ray({ 0.0f, 0.0f, 0.0f }, { -0.484687001f, 1.f, 0.363363564f })) == true);
			}
		}
	}
}

SCENARIO("Camera rays intersect spheres correctly", "[intersection]") {
	GIVEN("The sceen and common ray info") {
		LoadScene("test_scene1.xml");
		CommonRayInfo cri = getCommonCameraRayInfo();

		WHEN("A ray aimed at a sphere is traced through the scene") {
			Ray r = getCameraRay(400, 300, cri);
			HitInfo info;
			bool hit = Trace(r, rootNode, info);
			REQUIRE(hit == true);
		}
		WHEN("A ray aimed away from a sphere is traced through the scene") {
			Ray r = getCameraRay(799, 599, cri);
			HitInfo info;
			bool hit = Trace(r, rootNode, info);
			REQUIRE(hit == false);
		}
	}
}