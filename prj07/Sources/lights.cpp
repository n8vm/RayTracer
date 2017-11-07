#include "lights.h"
#include "render.h"

extern Node rootNode;

static bool disableShadows = false;

float GenLight::Shadow(Rays rays, float t_max) {
	if (disableShadows) return 1;
	HitInfo hInfo;
	Trace(rays, rootNode, hInfo, HIT_FRONT_AND_BACK);
	if ((hInfo.node != nullptr) && hInfo.z <= t_max) {
		return 0.;
	}
	return 1;
}
