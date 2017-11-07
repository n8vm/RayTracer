#include "lights.h"
#include "render.h"

//extern Camera camera;
//extern RenderImage renderImage;
extern Node rootNode;
//extern MaterialList materials;
//extern LightList lights;

static bool disableShadows = false;

float GenLight::Shadow(Ray ray, float t_max) {
	HitInfo hInfo;
	Trace(ray, rootNode, hInfo, true);
	if ((hInfo.node != nullptr) && hInfo.z < t_max && !disableShadows) {
		return 0;
	}
	return 1;
}