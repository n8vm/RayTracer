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

int LoadScene(const char *filename);
void ShowViewport();

int main() {
	LoadScene(SCENE_PATH "coke.xml");
	ShowViewport();
	return 0;
}