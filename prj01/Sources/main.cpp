using namespace std;

#include <iostream>
#include "scene.h"
#include "xmlload.h"
#include "objects.h"

Camera camera;
Node rootNode;
RenderImage renderImage;
Sphere theSphere;

int LoadScene(const char *filename);
void ShowViewport();

int main() {
	LoadScene("scene.xml");
	ShowViewport();
	return 0;
}