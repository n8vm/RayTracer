//-------------------------------------------------------------------------------
///
/// \file       objects.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    6.0
/// \date       September 30, 2015
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _OBJECTS_H_INCLUDED_
#define _OBJECTS_H_INCLUDED_

#include "scene.h"
#include "cyCodeBase/cyTriMesh.h"
#include "cyCodeBase/cyBVH.h"

//-------------------------------------------------------------------------------

class Sphere : public Object
{
public:
	virtual bool IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide = HIT_FRONT) const;
	virtual Box GetBoundBox() const { return Box(-1, -1, -1, 1, 1, 1); }
	virtual void ViewportDisplay(const Material *mtl) const;
};

extern Sphere theSphere;

//-------------------------------------------------------------------------------

class Plane : public Object
{
public:
	virtual bool IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide = HIT_FRONT) const;
	virtual Box GetBoundBox() const { return Box(-1, -1, 0, 1, 1, 0); }
	virtual void ViewportDisplay(const Material *mtl) const;
};

extern Plane thePlane;

//-------------------------------------------------------------------------------

class TriObj : public Object, public cyTriMesh
{
public:
	virtual bool IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide = HIT_FRONT) const;
	virtual Box GetBoundBox() const { return Box(GetBoundMin(), GetBoundMax()); }
	virtual void ViewportDisplay(const Material *mtl) const;

	bool Load(const char *filename)
	{
		std::string modelPath = MODEL_PATH;
		modelPath += filename;
		bvh.Clear();
		if (!LoadFromFileObj(modelPath.c_str())) return false;
		if (!HasNormals()) ComputeNormals();
		ComputeBoundingBox();
		bvh.SetMesh(this, 8);
		return true;
	}

private:
	cyBVHTriMesh bvh;
	bool IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const;
	bool TraceBVHNode(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID) const;
};

//-------------------------------------------------------------------------------

#endif