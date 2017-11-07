using namespace std;

#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"
#include "options.h"
#include <time.h>

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
	ShowViewport();
	return 0;
}