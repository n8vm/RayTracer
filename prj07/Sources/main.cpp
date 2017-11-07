using namespace std;

#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"

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
	LoadScene(SCENE_PATH "texture_test.xml");
	ShowViewport();
	return 0;
}