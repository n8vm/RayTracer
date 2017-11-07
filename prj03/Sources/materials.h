
//-------------------------------------------------------------------------------
///
/// \file       materials.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    2.0
/// \date       September 3, 2015
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------
 
#ifndef _MATERIALS_H_INCLUDED_
#define _MATERIALS_H_INCLUDED_
 
#include "scene.h"
 
//-------------------------------------------------------------------------------
 
class MtlBlinn : public Material
{
public:
    MtlBlinn() : diffuse(0.5f,0.5f,0.5f), specular(0.7f,0.7f,0.7f), glossiness(20.0f) {}
		Color Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights) const {
			Color c;
			c.SetBlack();
			/* For each light in the scene */
			for (int i = 0; i < lights.size(); i++) {
				Light *l = lights.at(i);
				Color li = l->Illuminate(hInfo.p, hInfo.N);
				if (l->IsAmbient()) {
					c += diffuse * li;
				}
				else {
					cyPoint3f ldir = -l->Direction(hInfo.p);
					float dot = (hInfo.N.GetNormalized() % ldir);
					if (dot >= 0) {
						cyPoint3f H = ((ldir + -ray.dir) / (ldir + -ray.dir).Length()).GetNormalized();
						c += li * dot * (diffuse + (specular * pow(hInfo.N % H, glossiness)));
						//cyPoint3f V = 2 * (ldir % hInfo.N) * hInfo.N - ldir;
						//c += li * dot * (diffuse + (white * pow(-ray.dir % V, glossiness)));
					}
				}
			}

			//c.r = c.g = c.b = 255;
			return c; //TODO IMPLEMENT THIS
		}
 
    void SetDiffuse(Color dif) { diffuse = dif; }
    void SetSpecular(Color spec) { specular = spec; }
    void SetGlossiness(float gloss) { glossiness = gloss; }
 
    virtual void SetViewportMaterial(int subMtlID=0) const; // used for OpenGL display
 
private:
    Color diffuse, specular;
    float glossiness;
};
 
//-------------------------------------------------------------------------------
 
#endif