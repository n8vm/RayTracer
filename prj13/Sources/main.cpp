using namespace std;

#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"
#include "options.h"
#include <time.h>
#include "render.h"
#include "options.h"

Camera camera;
Node rootNode;
RenderImage renderImage;
Sphere theSphere;
Plane thePlane;
LightList lights;
MaterialList materials;
ItemFileList<Object> objList;
TexturedColor background;
TexturedColor environment;
ItemFileList<Texture> textureList;

int LoadScene(const char *filename);
void ShowViewport();

int main() {
	srand(time(NULL));

	LoadScene(SCENE_PATH SCENE);

#if SHOW_VIEWPORT == true
	ShowViewport();
#else
	render_sppm();
#endif 

	return 0;
}