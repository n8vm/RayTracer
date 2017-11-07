using namespace std;

#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"

Camera camera;
Node rootNode;
RenderImage renderImage;
Sphere theSphere;
LightList lights;
MaterialList materials;

int LoadScene(const char *filename);
void ShowViewport();

int main() {
	LoadScene(SCENE_PATH "ReflectRefractTest.xml");
	ShowViewport();
	return 0;
}