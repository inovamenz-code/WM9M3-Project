#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

class HaltonSampler : public Sampler
{
public:
	unsigned int index;
	unsigned int dimension = 0;
	MTRandom tail;
	HaltonSampler(unsigned int sampleIndex) : index(sampleIndex), tail(sampleIndex) {}
	float next()
	{
		// MonteCarlo p137-140: radical inverse with a prime base per dimension.
		static const unsigned int bases[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,
			59,61,67,71,73,79,83,89,97,101,103,107,109,113,127,131};
		// Rare long paths use the existing RNG rather than repeating correlated bases.
		if (dimension >= 32) return tail.next();
		unsigned int base = bases[dimension++];
		unsigned int n = index;
		float value = 0.0f;
		float factor = 1.0f / base;
		while (n > 0)
		{
			value += (n % base) * factor;
			n /= base;
			factor /= base;
		}
		return std::min(value, std::nextafter(1.0f, 0.0f));
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float phi = 2.0f * M_PI * r1;
		float cosTheta = r2;
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - (cosTheta * cosTheta)));
		return Vec3(cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		if (wi.z < 0.0f)
		{
			return 0.0f;
		}
		return 1.0f / (2.0f * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float r = sqrtf(r1);
		float phi = 2.0f * M_PI * r2;
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		float z = sqrtf(std::max(0.0f, 1.0f - (x * x) - (y * y)));
		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		if (wi.z < 0.0f)
		{
			return 0.0f;
		}
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float phi = 2.0f * M_PI * r1;
		float cosTheta = 1.0f - (2.0f * r2);
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - (cosTheta * cosTheta)));
		return Vec3(cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1.0f / (4.0f * M_PI);
	}
};
