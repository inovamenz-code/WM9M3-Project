#pragma once

#include "Core.h"
#include "Sampling.h"
#define EPSILON 0.001f
class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float denom = Dot(n, r.dir);
		if (fabsf(denom) < EPSILON)
		{
			return false;
		}
		t = (d - Dot(n, r.o)) / denom;
		return t >= EPSILON;
	}
};



class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 mtE1 = vertices[1].p - vertices[0].p;
		Vec3 mtE2 = vertices[2].p - vertices[0].p;
		Vec3 p = Cross(r.dir, mtE2);
		float det = Dot(mtE1, p);
		if (fabsf(det) < 1e-6f)
		{
			return false;
		}
		float invDet = 1.0f / det;
		Vec3 s = r.o - vertices[0].p;
		float beta = Dot(s, p) * invDet;
		if (beta < 0.0f || beta > 1.0f)
		{
			return false;
		}
		Vec3 q = Cross(s, mtE1);
		float gamma = Dot(r.dir, q) * invDet;
		if (gamma < 0.0f || (beta + gamma) > 1.0f)
		{
			return false;
		}
		t = Dot(mtE2, q) * invDet;
		if (t < EPSILON)
		{
			return false;
		}
		u = 1.0f - beta - gamma;
		v = beta;
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		float sr1 = sqrtf(r1);
		float alpha = 1.0f - sr1;
		float beta = r2 * sr1;
		float gamma = 1.0f - alpha - beta;
		pdf = 1.0f / area;
		return (vertices[0].p * alpha) + (vertices[1].p * beta) + (vertices[2].p * gamma);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 t0 = (min - r.o) * r.invDir;
		Vec3 t1 = (max - r.o) * r.invDir;
		Vec3 tMin = Min(t0, t1);
		Vec3 tMax = Max(t0, t1);
		float tEntry = std::max(tMin.x, std::max(tMin.y, tMin.z));
		float tExit = std::min(tMax.x, std::min(tMax.y, tMax.z));
		t = tEntry;
		return (tExit >= tEntry) && (tExit >= 0.0f);
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 oc = r.o - centre;
		float a = Dot(r.dir, r.dir);
		float b = 2.0f * Dot(oc, r.dir);
		float c = Dot(oc, oc) - (radius * radius);
		float discriminant = (b * b) - (4.0f * a * c);
		if (discriminant < 0.0f)
		{
			return false;
		}
		float sqrtDiscriminant = sqrtf(discriminant);
		float t0 = (-b - sqrtDiscriminant) / (2.0f * a);
		float t1 = (-b + sqrtDiscriminant) / (2.0f * a);
		if (t0 >= EPSILON)
		{
			t = t0;
			return true;
		}
		if (t1 >= EPSILON)
		{
			t = t1;
			return true;
		}
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned int num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}
	~BVHNode()
	{
		if (l != NULL)
		{
			delete l;
		}
		if (r != NULL)
		{
			delete r;
		}
	}
	bool isLeaf() const
	{
		return l == NULL && r == NULL;
	}
	void updateBounds(std::vector<Triangle>& inputTriangles)
	{
		bounds.reset();
		for (unsigned int i = offset; i < offset + num; i++)
		{
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}
	}
	int longestAxis()
	{
		Vec3 size = bounds.max - bounds.min;
		if (size.x > size.y && size.x > size.z)
		{
			return 0;
		}
		if (size.y > size.z)
		{
			return 1;
		}
		return 2;
	}
	bool findSAHSplit(std::vector<Triangle>& triangles, int& axis, float& splitPos)
	{
		float parentArea = bounds.area();
		if (parentArea <= 0.0f)
		{
			return false;
		}
		float bestCost = num * TRIANGLE_COST;
		axis = -1;
		// SpatialDataStructures p46-47: test equally spaced candidate planes.
		for (int a = 0; a < 3; a++)
		{
			float step = (bounds.max.coords[a] - bounds.min.coords[a]) / BUILD_BINS;
			if (step <= 0.0f)
			{
				continue;
			}
			for (int bin = 1; bin < BUILD_BINS; bin++)
			{
				float position = bounds.min.coords[a] + step * bin;
				AABB leftBounds, rightBounds;
				unsigned int leftCount = 0, rightCount = 0;
				for (unsigned int i = offset; i < offset + num; i++)
				{
					const Triangle& triangle = triangles[i];
					bool goesLeft = triangle.centre().coords[a] < position;
					AABB& childBounds = goesLeft ? leftBounds : rightBounds;
					for (int v = 0; v < 3; v++)
					{
						childBounds.extend(triangle.vertices[v].p);
					}
					if (goesLeft) leftCount++;
					else rightCount++;
				}
				if (leftCount == 0 || rightCount == 0)
				{
					continue;
				}
				float cost = TRAVERSE_COST + TRIANGLE_COST *
					(leftBounds.area() * leftCount + rightBounds.area() * rightCount) / parentArea;
				if (cost < bestCost)
				{
					bestCost = cost;
					axis = a;
					splitPos = position;
				}
			}
		}
		return axis != -1;
	}
	void splitNode(std::vector<Triangle>& inputTriangles)
	{
		updateBounds(inputTriangles);
		if (num <= MAXNODE_TRIANGLES)
		{
			return;
		}
		int axis;
		float splitPos;
		if (!findSAHSplit(inputTriangles, axis, splitPos))
		{
			return;
		}
		// Keep the offset/count representation from the lecture (p73).
		auto middle = std::partition(inputTriangles.begin() + offset,
			inputTriangles.begin() + offset + num, [axis, splitPos](const Triangle& triangle)
		{
			return triangle.centre().coords[axis] < splitPos;
		});
		unsigned int mid = (unsigned int)(middle - inputTriangles.begin());
		if (mid == offset || mid == offset + num)
		{
			return;
		}
		l = new BVHNode();
		r = new BVHNode();
		l->offset = offset;
		l->num = mid - offset;
		r->offset = mid;
		r->num = (offset + num) - mid;
		l->splitNode(inputTriangles);
		r->splitNode(inputTriangles);
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		offset = 0;
		num = (unsigned int)inputTriangles.size();
		splitNode(inputTriangles);
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		float boundsT;
		if (!bounds.rayAABB(ray, boundsT))
		{
			return;
		}
		if (isLeaf())
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float t;
				float u;
				float v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t < intersection.t)
				{
					intersection.t = t;
					intersection.ID = i;
					intersection.alpha = u;
					intersection.beta = v;
					intersection.gamma = 1.0f - (u + v);
				}
			}
			return;
		}
		float leftT = FLT_MAX;
		float rightT = FLT_MAX;
		bool hitLeft = (l != NULL) ? l->bounds.rayAABB(ray, leftT) : false;
		bool hitRight = (r != NULL) ? r->bounds.rayAABB(ray, rightT) : false;
		if (hitLeft && hitRight)
		{
			if (leftT < rightT)
			{
				l->traverse(ray, triangles, intersection);
				if (rightT < intersection.t)
				{
					r->traverse(ray, triangles, intersection);
				}
			} else
			{
				r->traverse(ray, triangles, intersection);
				if (leftT < intersection.t)
				{
					l->traverse(ray, triangles, intersection);
				}
			}
		} else if (hitLeft)
		{
			l->traverse(ray, triangles, intersection);
		} else if (hitRight)
		{
			r->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		float boundsT;
		if (!bounds.rayAABB(ray, boundsT) || boundsT > maxT)
		{
			return true;
		}
		if (isLeaf())
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float t;
				float u;
				float v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t < maxT)
				{
					return false;
				}
			}
			return true;
		}
		if (l != NULL && !l->traverseVisible(ray, triangles, maxT))
		{
			return false;
		}
		if (r != NULL && !r->traverseVisible(ray, triangles, maxT))
		{
			return false;
		}
		return true;
	}
};
