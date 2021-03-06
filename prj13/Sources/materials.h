//-------------------------------------------------------------------------------
///
/// \file       materials.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    13.0
/// \date       November 20, 2015
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
	MtlBlinn() : diffuse(0.5f, 0.5f, 0.5f), specular(0.7f, 0.7f, 0.7f), glossiness(20.0f), emission(0,0,0),
		reflection(0, 0, 0), refraction(0, 0, 0), absorption(0, 0, 0), ior(1),
		reflectionGlossiness(0), refractionGlossiness(0) {}
	
	virtual bool isTransparent(const HitInfo &hInfo) const;
	bool ContainsDiffuse() const { 
		bool containsDiffuse = false;
		containsDiffuse |= (Color(diffuse.GetColor()) != Color::Black()); 
		containsDiffuse |= (Color(specular.GetColor()) != Color::Black());
		return containsDiffuse;
	}
	bool ContainsReflection() const { return (Color(reflection.GetColor()) != Color::Black()); }
	bool ContainsRefraction() const { return (Color(refraction.GetColor()) != Color::Black()); }
	virtual Color Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType = PATH_TRACING, IlluminationType IndirectType = PATH_TRACING) const;
	virtual Color ShadeReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &khr, Color absorpted, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const;
	virtual Color ShadeRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &khr, Color absorpted, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const;
	virtual Color ShadeDirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const;
	virtual Color ShadeIndirectPathTracing(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const;
	virtual Color ShadeCausticReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const;
	virtual Color ShadeCausticRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const;
	virtual Color ShadeIndirectPhotonMapping(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const;
	virtual Color ShadePoint(HitInfo &info, PixelStatistics &stats) const;

	void SetDiffuse		(Color dif)		{ diffuse.SetColor(dif); }
	void SetSpecular	(Color spec)	{ specular.SetColor(spec); }
	void SetGlossiness  (float gloss)	{ glossiness = gloss; }
	void SetEmission	(Color e)		{ emission.SetColor(e); }

	void SetReflection	(Color reflect)	{ reflection.SetColor(reflect); }
	void SetRefraction	(Color refract)	{ refraction.SetColor(refract); }
	void SetAbsorption	(Color absorp )	{ absorption = absorp; }
	void SetRefractionIndex(float _ior) { ior = _ior; }
	float GetIndexOfRefraction() const { return ior; };

	void SetDiffuseTexture	 (TextureMap *map)	{ diffuse.SetTexture(map); }
	void SetSpecularTexture	 (TextureMap *map)	{ specular.SetTexture(map); }
	void SetEmissionTexture	 (TextureMap *map)	{ emission.SetTexture(map); }
	void SetReflectionTexture(TextureMap *map)	{ reflection.SetTexture(map); }
	void SetRefractionTexture(TextureMap *map)	{ refraction.SetTexture(map); }
	void SetReflectionGlossiness(float gloss)	{ reflectionGlossiness=gloss; }
	void SetRefractionGlossiness(float gloss)	{ refractionGlossiness=gloss; }

	virtual void SetViewportMaterial(int subMtlID=0) const;	// used for OpenGL display

	// Photon Extensions
	virtual bool IsPhotonSurface(int subMtlID=0) const { return diffuse.GetColor().Gray() > 0; }	// if this method returns true, the photon will be stored
	virtual bool BouncePhoton(BounceInfo &bInfo, const HitInfo &hInfo) const;	// if this method returns true, a new photon with the given direction and color will be traced
	virtual bool containsNonspecular() const;
	virtual void GetDiffuseHits(std::vector<HitInfo> &hits, Ray ray, HitInfo hInfo, int bounceCount) const;

private:
	TexturedColor diffuse, specular, reflection, refraction, emission;
	float glossiness;
	Color absorption;
	float ior;	// index of refraction
	float reflectionGlossiness, refractionGlossiness;
};

//-------------------------------------------------------------------------------

class MultiMtl : public Material
{
public:
	virtual ~MultiMtl() { for ( unsigned int i=0; i<mtls.size(); i++ ) delete mtls[i]; }

	virtual bool isTransparent(const HitInfo &hInfo) const {
		return hInfo.mtlID<(int)mtls.size() ? mtls[hInfo.mtlID]->isTransparent(hInfo) : false;
	}

	virtual Color Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType,IlluminationType indirectType)
		const { return hInfo.mtlID<(int)mtls.size() ? mtls[hInfo.mtlID]->Shade(rays, hInfo, lights, bounceCount, statistics, directType, indirectType) : Color(1, 1, 1); }

	virtual Color ShadePoint(HitInfo &info, PixelStatistics &stats) const { return Color::White(); }

	virtual void SetViewportMaterial(int subMtlID=0) const { if ( subMtlID<(int)mtls.size() ) mtls[subMtlID]->SetViewportMaterial(); }

	void AppendMaterial(Material *m) { mtls.push_back(m); }

	// Photon Extensions
	virtual bool IsPhotonSurface(int subMtlID=0) const { return mtls[subMtlID]->IsPhotonSurface(); }
	virtual bool BouncePhoton(BounceInfo &bInfo, const HitInfo &hInfo) const { return hInfo.mtlID<(int)mtls.size() ? mtls[hInfo.mtlID]->BouncePhoton(bInfo, hInfo) : false; }

private:
	std::vector<Material*> mtls;
};

//-------------------------------------------------------------------------------

#endif