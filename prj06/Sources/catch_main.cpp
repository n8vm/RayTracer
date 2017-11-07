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
LightList lights;
MaterialList materials;

int main(int argc, char* argv[])
{	
	//LoadScene(SCENE_PATH "prj2.xml");
	//ShowViewport();
	int result = Catch::Session().run(argc, argv);
	return (result < 0xff ? result : 0xff);
}

SCENARIO("Camera rays can be calculated correctly", "[prj1]") {
	GIVEN("The scene and common ray info") {
		LoadScene(SCENE_PATH "ray_test.xml");
		CommonRayInfo cri = getCommonCameraRayInfo();
		WHEN("Some camera rays are computed") {
			Ray r1 = getCameraRay(0, 0, cri);
			Ray r2 = getCameraRay(799, 0, cri);
			Ray r3 = getCameraRay(799, 599, cri);
			Ray r4 = getCameraRay(0, 599, cri);

			THEN("The results are valid") {
				REQUIRE( compareRay(r1, Ray({ 0.0f, 0.0f, 0.0f }, { -0.414557070f, .310788095f, -0.855308831f })) == true);
				REQUIRE( compareRay(r2, Ray({ 0.0f, 0.0f, 0.0f }, { 0.414557070f,  .310788095f, -0.855308831f })) == true);
				REQUIRE( compareRay(r3, Ray({ 0.0f, 0.0f, 0.0f }, { 0.414557070f, -.310788065f, 0.855308831f })) == true);
				REQUIRE( compareRay(r4, Ray({ 0.0f, 0.0f, 0.0f }, { -0.414557070f, -.310788065f, 0.855308831f })) == true);
			}
		}
	}
}

SCENARIO("Camera rays intersect spheres correctly", "[prj1]") {
	GIVEN("The scene and common ray info") {
		LoadScene(SCENE_PATH "ray_intersects_sphere.xml");
		CommonRayInfo cri = getCommonCameraRayInfo();

		WHEN("A ray aimed at a sphere is traced through the scene") {
			Ray r = getCameraRay(500, 300, cri);
			HitInfo info;
			Trace(r, rootNode, info, false);
			REQUIRE(info.node != nullptr);
			REQUIRE(info.z - 8.88425922 < .00001);
			REQUIRE(info.front == true);
		}
		WHEN("A ray aimed away from a sphere is traced through the scene") {
			Ray r = getCameraRay(799, 599, cri);
			HitInfo info;
			Trace(r, rootNode, info, false);
			REQUIRE(info.node == nullptr);
		}
	}
}

SCENARIO("Surface normals are calculated correctly", "[prj2]") {
	GIVEN("The scene and common ray info") {
		LoadScene(SCENE_PATH "prj2.xml");
		CommonRayInfo cri = getCommonCameraRayInfo();
		
		WHEN("A ray aimed at a sphere is traced through the scene") {

				Ray r = getCameraRay(500, 66, cri);
				HitInfo info;
				Trace(r, rootNode, info, false);

				THEN("The first surface normal should be calculated correctly") {
					REQUIRE(comparePoint(info.N, cyPoint3f(0.587693572f, -0.762885153f, 0.269485682f)) == true);
				}

				r = getCameraRay(517, 441, cri);
				HitInfo info2;
				Trace(r, rootNode, info2, false);

				THEN("The second surface normal should be calculated correctly") {
					REQUIRE(comparePoint(info2.N, cyPoint3f(0.128222868f, -0.826791644f, 0.547699273f)) == true);
				}

		}
	}


}
