#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		if (iorInt == iorExt) return 0.0f;
		float eta = cosTheta >= 0.0f ? iorExt / iorInt : iorInt / iorExt;
		float cosI = std::min(fabsf(cosTheta), 1.0f);
		float sinTSq = eta * eta * (1.0f - cosI * cosI);
		if (sinTSq >= 1.0f) return 1.0f; // Total internal reflection (p33).
		float cosT = sqrtf(1.0f - sinTSq);
		// Materials p36: average the squared polarization amplitudes.
		float parallel = (cosI - eta * cosT) / (cosI + eta * cosT);
		float perpendicular = (eta * cosI - cosT) / (eta * cosI + cosT);
		return 0.5f * (parallel * parallel + perpendicular * perpendicular);
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		float c = std::min(fabsf(cosTheta), 1.0f);
		Colour one(1, 1, 1);
		Colour cosSq(c * c, c * c, c * c);
		Colour etaSqK = ior * ior + k * k;
		Colour twoEtaCos = ior * (2.0f * c);
		// Materials p41 的导体近似：平行项最后应为 1，而不是课件中的 sin^2。
		// In the p41 conductor approximation, the parallel term ends in 1, not sin^2.
		Colour parallel = (etaSqK * cosSq - twoEtaCos + one) / (etaSqK * cosSq + twoEtaCos + one);
		Colour perpendicular = (etaSqK - twoEtaCos + cosSq) / (etaSqK + twoEtaCos + cosSq);
		return (parallel + perpendicular) * 0.5f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		if (wi.z == 0.0f) return FLT_MAX;
		float tanSq = (wi.x * wi.x + wi.y * wi.y) / (wi.z * wi.z);
		return 0.5f * (sqrtf(1.0f + alpha * alpha * tanSq) - 1.0f);
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		if (wi.z <= 0.0f || wo.z <= 0.0f) return 0.0f;
		// Materials p83/p88：假设遮蔽与阴影独立，G = G1(wi) * G1(wo)。
		// Assume independent masking and shadowing: G = G1(wi) * G1(wo).
		return (1.0f / (1.0f + lambdaGGX(wi, alpha))) * (1.0f / (1.0f + lambdaGGX(wo, alpha)));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		if (h.z <= 0.0f || alpha <= 0.0f) return 0.0f;
		float alphaSq = alpha * alpha;
		// p80 换个等价写法，避免粗糙度很小时相减丢精度。
		// Equivalent p80 form, avoiding cancellation in 1 + (alpha^2 - 1) at low roughness.
		float denominator = h.x * h.x + h.y * h.y + alphaSq * h.z * h.z;
		return alphaSq / (M_PI * denominator * denominator);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Vec3 sampleImportance(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		return sample(shadingData, sampler, reflectedColour, pdf);
	}
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	// 给 OIDN 的首次交点颜色，不带光照；玻璃等介电表面先用白色。
	// First-hit colour for OIDN, without lighting; use white for glass and other dielectrics.
	virtual Colour getAlbedo(const ShadingData& shadingData)
	{
		return Colour(1, 1, 1);
	}
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		return albedo->sample(shadingData.tu, shadingData.tv);
	}
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		wi = shadingData.frame.toWorld(wi);
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		return albedo->sample(shadingData.tu, shadingData.tv);
	}
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
		pdf = 1.0f;
		if (wiLocal.z <= 0.0f)
		{
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return shadingData.frame.toWorld(wiLocal);
		}
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wiLocal.z;
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		// 金属用正入射反射率乘纹理，不使用方向相关的 BRDF 值。
		// For metals, use normal-incidence reflectance times the texture, not the directional BRDF.
		return albedo->sample(shadingData.tu, shadingData.tv) * ShadingHelper::fresnelConductor(1.0f, eta, k);
	}
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(std::max(0.0f, roughness));
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		pdf = 0.0f;
		reflectedColour = Colour(0, 0, 0);
		if (wo.z <= 0.0f) return Vec3(0, 0, 0);
		if (isPureSpecular())
		{
			// p103：接近光滑时作为带导体 Fresnel 的镜面事件处理。
			// Near-smooth metals use a mirror event with conductor Fresnel.
			Vec3 wi(-wo.x, -wo.y, wo.z);
			pdf = 1.0f;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * ShadingHelper::fresnelConductor(wo.z, eta, k) / wi.z;
			return shadingData.frame.toWorld(wi).normalize();
		}
		// p94：按 D(h) * cos(theta_h) 采样微表面法线，而不是余弦采样 wi。
		// Sample the microfacet normal using D(h) * cos(theta_h), not a cosine-sampled wi.
		float r1 = sampler->next();
		float r2 = sampler->next();
		float denominator = (1.0f - r1) + alpha * alpha * r1;
		float cosTheta = sqrtf((1.0f - r1) / denominator);
		float sinTheta = sqrtf(alpha * alpha * r1 / denominator);
		float phi = 2.0f * M_PI * r2;
		Vec3 h(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		float woH = wo.dot(h);
		Vec3 wiLocal = h * (2.0f * woH) - wo;
		// 无效反射直接算零，不能重抽，否则 PDF 就不匹配了。
		// Invalid reflections contribute zero; resampling would no longer match this PDF.
		if (woH <= 0.0f || wiLocal.z <= 0.0f) return Vec3(0, 0, 0);
		Vec3 wi = shadingData.frame.toWorld(wiLocal).normalize();
		pdf = PDF(shadingData, wi);
		reflectedColour = evaluate(shadingData, wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 incoming = shadingData.frame.toLocal(wi);
		if (isPureSpecular() || wo.z <= 0.0f || incoming.z <= 0.0f) return Colour(0, 0, 0);
		Vec3 h = (wo + incoming).normalize();
		Colour fresnel = ShadingHelper::fresnelConductor(wo.dot(h), eta, k);
		// Materials p100：f = D * G * F / (4 cos(theta_i) cos(theta_o))。
		// Combine the distribution, geometry and Fresnel terms using the p100 BRDF.
		float scale = ShadingHelper::Dggx(h, alpha) * ShadingHelper::Gggx(incoming, wo, alpha) / (4.0f * incoming.z * wo.z);
		return albedo->sample(shadingData.tu, shadingData.tv) * fresnel * scale;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		if (isPureSpecular() || wiLocal.z <= 0.0f || wo.z <= 0.0f) return 0.0f;
		Vec3 h = (wo + wiLocal).normalize();
		// p92：微表面法线密度转为反射方向密度的 Jacobian。
		// Convert the microfacet-normal PDF to the reflected-direction PDF with the p92 Jacobian.
		return ShadingHelper::Dggx(h, alpha) * h.z / (4.0f * wo.dot(h));
	}
	bool isPureSpecular()
	{
		return alpha < 0.001f;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		pdf = 0.0f;
		reflectedColour = Colour(0, 0, 0);
		if (wo.z == 0.0f) return Vec3(0, 0, 0);
		if (intIOR == extIOR)
		{
			// No interface: avoid cancellation in sqrt(1 - (1 - cos^2)) at grazing angles.
			pdf = 1.0f;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / fabsf(wo.z);
			return -shadingData.wo;
		}
		float eta = wo.z > 0.0f ? extIOR / intIOR : intIOR / extIOR;
		float fresnel = ShadingHelper::fresnelDielectric(wo.z, intIOR, extIOR);
		Vec3 wi;
		if (sampler->next() < fresnel || fresnel == 1.0f)
		{
			wi = Vec3(-wo.x, -wo.y, wo.z);
			pdf = fresnel;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (fresnel / fabsf(wi.z));
		} else
		{
			float cosI = std::min(fabsf(wo.z), 1.0f);
			float cosT = sqrtf(std::max(0.0f, 1.0f - eta * eta * (1.0f - cosI * cosI)));
			wi = Vec3(-eta * wo.x, -eta * wo.y, wo.z > 0.0f ? -cosT : cosT);
			pdf = 1.0f - fresnel;
			// Camera paths transport radiance: incident/transmitted IOR squared (p48).
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (pdf * eta * eta / cosT);
		}
		return shadingData.frame.toWorld(wi).normalize();
	}
	Vec3 sampleImportance(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) override
	{
		Vec3 wi = sample(shadingData, sampler, reflectedColour, pdf);
		float cosO = Dot(shadingData.wo, shadingData.sNormal);
		if (pdf > 0.0f && cosO * Dot(wi, shadingData.sNormal) < 0.0f)
		{
			// RenderingAlgorithms p22：光源路径传 importance，透射不带 radiance 的 eta^2。
			// Light paths carry importance, so transmission omits the radiance eta^2 factor.
			float eta = cosO > 0.0f ? extIOR / intIOR : intIOR / extIOR;
			reflectedColour = reflectedColour / (eta * eta);
		}
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0, 0, 0);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		return 0.0f; // Delta distribution; sample() returns the event probability.
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		return albedo->sample(shadingData.tu, shadingData.tv);
	}
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		return albedo->sample(shadingData.tu, shadingData.tv);
	}
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour getAlbedo(const ShadingData& shadingData) override
	{
		// 渲染和辅助色都用 base。 Use base for both rendering and albedo.
		return base->getAlbedo(shadingData);
	}
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};
