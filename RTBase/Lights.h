#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		// RenderingAlgorithms p10: cosine emission cancels the first geometry cosine.
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<double> rowCDF;
	std::vector<double> columnCDF;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		buildDistribution();
	}
	void buildDistribution()
	{
		// MonteCarlo p72-74: marginal rows, then conditional columns in each row.
		rowCDF.assign(env->height + 1, 0.0);
		columnCDF.assign(env->height * (env->width + 1), 0.0);
		for (int y = 0; y < env->height; y++)
		{
			double* row = &columnCDF[y * (env->width + 1)];
			int nextY = std::min(y + 1, env->height - 1);
			float sinTheta = sinf(M_PI * (y + 0.5f) / env->height);
			for (int x = 0; x < env->width; x++)
			{
				int nextX = (x + 1) % env->width;
				// Match bilinear lookup: a black texel's cell can contain a bright neighbour.
				float luminance = 0.25f * (env->texels[y * env->width + x].Lum() +
					env->texels[y * env->width + nextX].Lum() + env->texels[nextY * env->width + x].Lum() +
					env->texels[nextY * env->width + nextX].Lum());
				row[x + 1] = row[x] + std::max(0.0f, luminance) * sinTheta;
			}
			rowCDF[y + 1] = rowCDF[y] + row[env->width];
		}
	}
	static float sampleCDF(const double* cdf, int count, float random, int& index)
	{
		// Unnormalised CDFs avoid an extra table. upper_bound skips zero-weight bins.
		double target = std::min(double(random) * cdf[count], std::nextafter(cdf[count], 0.0));
		index = int(std::upper_bound(cdf, cdf + count + 1, target) - cdf) - 1;
		double fraction = (target - cdf[index]) / (cdf[index + 1] - cdf[index]);
		return std::min(float((index + fraction) / count), std::nextafter(1.0f, 0.0f));
	}
	void directionUV(const Vec3& wi, float& u, float& v)
	{
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.0f) phi += 2.0f * M_PI;
		u = phi / (2.0f * M_PI);
		if (u >= 1.0f) u = 0.0f;
		// Equivalent to acos(y) for unit directions, but stable near both poles.
		v = atan2f(sqrtf(wi.x * wi.x + wi.z * wi.z), wi.y) / M_PI;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		if (rowCDF.back() == 0.0)
		{
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			reflectedColour = evaluate(wi);
			return wi;
		}
		int x, y;
		float v = sampleCDF(rowCDF.data(), env->height, sampler->next(), y);
		float u = sampleCDF(&columnCDF[y * (env->width + 1)], env->width, sampler->next(), x);
		// Keep finite-precision samples off the zero-measure poles.
		v = std::max(1e-7f, std::min(v, 1.0f - 1e-7f));
		float theta = M_PI * v;
		float phi = 2.0f * M_PI * u;
		Vec3 wi(cosf(phi) * sinf(theta), cosf(theta), sinf(phi) * sinf(theta));
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(wi);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u, v;
		directionUV(wi, u, v);
		// Longitude repeats, latitude must not interpolate from south back to north.
		return env->sample(u, std::min(v, float(env->height - 1) / env->height));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		if (rowCDF.back() == 0.0) return SamplingDistributions::uniformSpherePDF(wi);
		float u, v;
		directionUV(wi, u, v);
		float sinTheta = sqrtf(wi.x * wi.x + wi.z * wi.z);
		if (sinTheta == 0.0f) return 0.0f;
		int x = std::min(int(u * env->width), env->width - 1);
		int y = std::min(int(v * env->height), env->height - 1);
		const double* row = &columnCDF[y * (env->width + 1)];
		double pdfUV = (row[x + 1] - row[x]) * env->width * env->height / rowCDF.back();
		// LightTransport 1 p69-71: UV density -> solid angle, including sin(theta).
		return float(pdfUV / (2.0 * M_PI * M_PI * sinTheta));
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		// Integrate each cell's average radiance over its spherical area.
		return float(rowCDF.back() * (4.0 * M_PI / env->width) * sin(M_PI / (2.0 * env->height)));
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};
