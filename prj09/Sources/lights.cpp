#include "lights.h"
#include "render.h"
#include "options.h"

extern Node rootNode;


float GenLight::Shadow(Rays rays, float t_max) {
	if (DISABLE_SHADOWS) return 1;
	HitInfo hInfo;
	Trace(rays, rootNode, hInfo, HIT_FRONT_AND_BACK);
	if ((hInfo.node != nullptr) && hInfo.z <= t_max) {
		return 0.;
	}
	return 1;
}
