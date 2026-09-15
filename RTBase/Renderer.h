#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <atomic>
#include <fstream>
#include <OpenImageDenoise/oidn.hpp>

// RenderingAlgorithms p60：留下交点信息，Le 存加权后的路径贡献。
// Keep the hit data and store the weighted path contribution in Le.
class VPL
{
public:
	ShadingData shadingData;
	Colour Le;
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	Film* albedoFilm = NULL;
	Film* normalFilm = NULL;
	std::vector<Colour> colorBuffer, albedoBuffer, normalBuffer;
	std::vector<Colour> outputBuffer;
	oidn::DeviceRef denoiseDevice;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	unsigned int renderSeed = 1;
	bool environmentMIS = true;
	bool useInstantRadiosity = false;
	unsigned int vplPaths = 64;
	unsigned int vplLightPaths = 0;
	std::vector<VPL> vpls;
	static float powerHeuristic(float sampledPDF, float otherPDF)
	{
		// MonteCarlo p105, beta=2. Rescale first to avoid squaring a huge density.
		float scale = std::max(sampledPDF, otherPDF);
		if (scale <= 0.0f) return 0.0f;
		float a = sampledPDF / scale;
		float b = otherPDF / scale;
		return a * a / (a * a + b * b);
	}
	float environmentPDF(const Vec3& wi)
	{
		// NEE first selects one light; its PMF is part of the competing density.
		for (Light* light : scene->lights)
		{
			if (light == scene->background)
			{
				ShadingData unused = {};
				return light->PDF(unused, wi) / float(scene->lights.size());
			}
		}
		return 0.0f;
	}
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas, bool withAOVs = false)
	{
		scene = _scene;
		canvas = _canvas;
		vpls.clear();
		vplLightPaths = 0;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		if (withAOVs)
		{
			albedoFilm = new Film();
			normalFilm = new Film();
			albedoFilm->init(film->width, film->height, film->filter);
			normalFilm->init(film->width, film->height, film->filter);
		}
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
		if (albedoFilm != NULL)
		{
			albedoFilm->clear();
			normalFilm->clear();
		}
		colorBuffer.clear();
		albedoBuffer.clear();
		normalBuffer.clear();
		outputBuffer.clear();
	}
	// 直接采一个光源，不等随机反弹碰到它。
	// Next event estimation: sample a light directly.
	Colour computeDirect(ShadingData shadingData, Sampler* sampler, bool useEnvironmentMIS = false)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		float pmf = 0.0f;
		Light* light = scene->sampleLight(sampler, pmf);
		if (light == NULL || pmf <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Colour emitted;
		float pdf = 0.0f;
		Vec3 sampled = light->sample(shadingData, sampler, emitted, pdf);
		if (pdf <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// 面光源返回点和面积 PDF，环境光返回方向和立体角 PDF。
		// Area lights return a point and area PDF; environment lights return a direction and solid-angle PDF.
		if (light->isArea() == true)
		{
			Vec3 toLight = sampled - shadingData.x;
			float distanceSq = toLight.lengthSq();
			Vec3 wi = toLight.normalize();
			float cosSurface = Dot(wi, shadingData.sNormal);
			// A light only emits on the side its geometric normal points to, so the
			// cosine at the light uses -wi. 光源只朝几何法线那一侧发光，故用 -wi。
			float cosLight = -Dot(wi, light->normal(shadingData, wi));
			if (cosSurface <= 0.0f || cosLight <= 0.0f)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			if (scene->visible(shadingData.x, sampled) == false)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			// 把面积 PDF 换成立体角 PDF：p_w = p_A * d^2 / cos(theta_light)。
			// Convert the area PDF to solid angle before using it in the lighting estimate.
			float pdfSolidAngle = pdf * distanceSq / cosLight;
			if (pdfSolidAngle <= 0.0f)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			return f * emitted * (cosSurface / (pdfSolidAngle * pmf));
		}
		// Infinite light. The sampled value is already a direction, so only the
		// shadow ray needs a distance. 无限远光源：采样值已是方向，只差一条阴影线的长度。
		Vec3 wi = sampled;
		float cosSurface = Dot(wi, shadingData.sNormal);
		if (cosSurface <= 0.0f)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Reach past the scene bounds so nothing beyond the geometry can occlude.
		// 追到场景包围球之外，保证不会漏掉遮挡。
		float reach = 2.0f * use<SceneBounds>().sceneRadius;
		if (reach <= 0.0f)
		{
			reach = 10000.0f;
		}
		if (scene->visible(shadingData.x, shadingData.x + (wi * reach)) == false)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		float weight = useEnvironmentMIS ? powerHeuristic(pdf * pmf, shadingData.bsdf->PDF(shadingData, wi)) : 1.0f;
		return f * emitted * (weight * cosSurface / (pdf * pmf));
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool previousSpecular = false, float previousPDF = 0.0f)
	{
		// Add pathtracer code here
		IntersectionData intersection = scene->traverse(r);
		bool includeEmission = depth == 0 || previousSpecular;
		if (intersection.t == FLT_MAX)
		{
			// LightTransport 1 p105: NEE already covers non-specular light hits.
			// p106: with MIS, keep the BSDF-sampled environment hit and weight it.
			float weight = 1.0f;
			if (!includeEmission)
			{
				weight = environmentMIS ? powerHeuristic(previousPDF, environmentPDF(r.dir)) : 0.0f;
			}
			return pathThroughput * scene->background->evaluate(r.dir) * weight;
		}
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.bsdf->isLight())
		{
			// Area lights use NEE without MIS; only camera or specular hits add emission here.
			return includeEmission ? pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo) : Colour(0, 0, 0);
		}
		Colour radiance = pathThroughput * computeDirect(shadingData, sampler, environmentMIS);
		Colour f;
		float pdf;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, f, pdf);
		if (!(pdf > 0.0f))
		{
			return radiance;
		}
		// sample() returns f; multiply by cosine/pdf exactly once (p111, p116).
		Colour nextThroughput = pathThroughput * f * (fabsf(Dot(wi, shadingData.sNormal)) / pdf);
		if (!(nextThroughput.Lum() > 0.0f))
		{
			return radiance;
		}
		if (depth >= 3)
		{
			// Surviving paths must divide by their survival probability (p99-100).
			float survival = std::min(nextThroughput.Lum(), 0.95f);
			if (sampler->next() >= survival)
			{
				return radiance;
			}
			nextThroughput = nextThroughput / survival;
		}
		Ray nextRay(shadingData.x + wi * EPSILON, wi);
		return radiance + pathTrace(nextRay, nextThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular(), pdf);
	}
	// One camera ray worth of direct lighting. 一条相机光线的直接光照。
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			// A camera ray landing on an emitter sees the emitter itself.
			// 相机光线直接打到光源时，看到的就是光源本身。
			if (shadingData.bsdf->isLight() == true)
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	// 保留旧 BRDF 调试入口；它不是用于降噪的 albedo。
	// Keep the old BRDF preview; this is not the denoiser's albedo input.
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void sampleAOVs(Ray& ray, Colour& surfaceAlbedo, Colour& surfaceNormal)
	{
		// SpeedingItUp p83/p103：首个交点的辅助数据，不消耗路径采样器。
		// Read AOVs at the first hit without advancing the path sampler.
		surfaceAlbedo = Colour(1, 1, 1);
		surfaceNormal = Colour(0, 0, 0);
		IntersectionData hit = scene->traverse(ray);
		if (hit.t == FLT_MAX) return;
		ShadingData shading = scene->calculateShadingData(hit, ray);
		if (!shading.bsdf->isLight()) surfaceAlbedo = shading.bsdf->getAlbedo(shading);
		surfaceAlbedo.r = std::max(0.0f, std::min(1.0f, surfaceAlbedo.r));
		surfaceAlbedo.g = std::max(0.0f, std::min(1.0f, surfaceAlbedo.g));
		surfaceAlbedo.b = std::max(0.0f, std::min(1.0f, surfaceAlbedo.b));
		Vec3 n = shading.sNormal;
		if (n.lengthSq() > 0.0f) n = n.normalize();
		// 世界空间法线，负分量也要留下。 Keep signed world-space normals.
		surfaceNormal = Colour(n.x, n.y, n.z);
	}
	void renderTile(int startX, int startY, int endX, int endY, Sampler* sampler)
	{
		for (int y = startY; y < endY; y++)
		{
			for (int x = startX; x < endX; x++)
			{
				float px = x + sampler->next();
				float py = y + sampler->next();
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, sampler);
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				Colour col = useInstantRadiosity
					? traceIR(ray, sampler)
					: pathTrace(ray, pathThroughput, 0, sampler);
				Colour surfaceAlbedo, surfaceNormal;
				if (albedoFilm != NULL) sampleAOVs(ray, surfaceAlbedo, surfaceNormal);
				if (film->filter->size() == 0)
				{
					// BoxFilter 样本属于当前像素，避免 px 浮点舍入到 x+1 后跨块写入。
					// Keep BoxFilter writes in this pixel, even if px rounds up to x+1.
					px = x + 0.5f;
					py = y + 0.5f;
				}
				film->splat(px, py, col);
				if (albedoFilm != NULL)
				{
					albedoFilm->splat(px, py, surfaceAlbedo);
					normalFilm->splat(px, py, surfaceNormal);
				}
				/* 旧版直接显示单次采样，没有做 gamma，留在这里对照。
				   Old single-sample display, without gamma, kept for comparison.
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				*/
			}
		}
	}
	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		// RenderingAlgorithms p25-26：几何项乘相机重要性，再投到胶片。
		// Multiply the geometry term by camera importance, then splat onto the film.
		Vec3 toCamera = scene->camera.origin - p;
		float distanceSq = toCamera.lengthSq();
		if (distanceSq <= EPSILON * EPSILON) return;
		Vec3 wi = toCamera / sqrtf(distanceSq);
		float cosCamera = Dot(scene->camera.viewDirection, -wi);
		float cosSurface = Dot(n, wi);
		if (cosCamera <= 0.0f || cosSurface <= 0.0f) return;
		float x, y;
		if (!scene->camera.projectOntoCamera(p, x, y)) return;
		if (x < 0 || x >= film->width || y < 0 || y >= film->height) return;
		if (!scene->visible(p, scene->camera.origin)) return;
		float cosSquared = cosCamera * cosCamera;
		float We = 1.0f / (scene->camera.Afilm * cosSquared * cosSquared);
		float G = cosSurface * cosCamera / distanceSq;
		if (film->filter->size() == 0)
		{
			x = floorf(x) + 0.5f;
			y = floorf(y) + 0.5f;
		}
		film->splat(x, y, col * (We * G));
	}
	float importanceNormalCorrection(const ShadingData& shadingData, const Vec3& wi)
	{
		float woNs = Dot(shadingData.wo, shadingData.sNormal);
		float woNg = Dot(shadingData.wo, shadingData.gNormal);
		float wiNs = Dot(wi, shadingData.sNormal);
		float wiNg = Dot(wi, shadingData.gNormal);
		if (woNs * woNg <= 0 || wiNs * wiNg <= 0) return 0.0f;
		// RenderingAlgorithms p22：绝对值也适用于进入/离开玻璃的方向。
		// The absolute value also handles paths entering or leaving glass.
		return fabsf((woNs * wiNg) / (woNg * wiNs));
	}
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int maxDepth = -1, bool storeVPLs = false)
	{
		// RenderingAlgorithms p29：用循环延伸同一条路径，避免 RR 长路径占用递归栈。
		// Extend the path in a loop so long RR paths do not grow the call stack.
		for (int depth = 0; maxDepth < 0 || depth < maxDepth; depth++)
		{
			IntersectionData hit = scene->traverse(r);
			if (hit.t == FLT_MAX) return;
			ShadingData shadingData = scene->calculateShadingData(hit, r);
			if (shadingData.bsdf->isLight()) return;
			// Delta 表面不能任意连接相机，但光线路径仍要从它继续反射/折射。
			// Skip camera connections at delta surfaces, but continue reflection or refraction.
			if (!shadingData.bsdf->isPureSpecular())
			{
				if (storeVPLs)
				{
					vpls.push_back({shadingData, pathThroughput * Le});
				} else
				{
					Vec3 wi = scene->camera.origin - shadingData.x;
					if (wi.lengthSq() > EPSILON * EPSILON)
					{
						wi = wi.normalize();
						float correction = importanceNormalCorrection(shadingData, wi);
						Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;
						connectToCamera(shadingData.x, shadingData.sNormal, col * correction);
					}
				}
			}
			if (maxDepth > 0 && depth + 1 >= maxDepth) return;
			Colour f;
			float pdf;
			Vec3 wi = shadingData.bsdf->sampleImportance(shadingData, sampler, f, pdf);
			if (!(pdf > 0.0f)) return;
			float correction = importanceNormalCorrection(shadingData, wi);
			pathThroughput = pathThroughput * f * (fabsf(Dot(wi, shadingData.sNormal)) * correction / pdf);
			if (!(pathThroughput.Lum() > 0.0f)) return;
			if (depth >= 3)
			{
				// p23：沿用 pathTrace 的 RR，留下的路径要除以存活概率。
				// Reuse pathTrace's RR and divide surviving paths by their survival probability.
				float survival = std::min(pathThroughput.Lum(), 0.95f);
				if (sampler->next() >= survival) return;
				pathThroughput = pathThroughput / survival;
			}
			r.init(shadingData.x + wi * EPSILON, wi);
		}
	}
	void lightTrace(Sampler* sampler, int maxDepth = 0, bool storeVPLs = false)
	{
		// RenderingAlgorithms p27：先把光源上的点直接连到相机。
		// First connect the sampled emitter point to the camera.
		float pmf, pdfPosition;
		Light* light = scene->sampleLight(sampler, pmf);
		if (light == NULL || !light->isArea() || pmf <= 0.0f) return;
		Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
		if (pdfPosition <= 0.0f) return;
		ShadingData unused = {};
		Vec3 n = light->normal(unused, Vec3(0, 0, 0));
		// 面光源的 Le 与出射角无关；连接函数检查是否朝向相机。
		// Area-light Le is angle-independent; connectToCamera checks the facing direction.
		// IR 的真实光源由 computeDirect 处理，不再生成光源 VPL，避免重复计光。
		// IR uses computeDirect for real lights, so do not also store emitter VPLs.
		if (!storeVPLs)
			connectToCamera(p, n, light->evaluate(-n) / (pmf * pdfPosition));
		if (maxDepth == 0) return;
		// p10/p28：发射方向 PDF 只除一次，余弦采样使 cos/pdf 等于 pi。
		// Divide by the emission PDF once; cosine sampling gives cos/pdf = pi.
		float pdfDirection;
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
		float cosLight = Dot(n, wi);
		if (pdfDirection <= 0.0f || cosLight <= 0.0f) return;
		Colour pathThroughput = Colour(1, 1, 1) * (cosLight / (pmf * pdfPosition * pdfDirection));
		Ray ray(p + wi * EPSILON, wi);
		lightTracePath(ray, pathThroughput, light->evaluate(-wi), sampler, maxDepth, storeVPLs);
	}
	void generateVPLs(unsigned int pathCount)
	{
		// RenderingAlgorithms p55：少量分布较均匀的光源路径，复用 LT 反弹与 RR。
		// Spread a small set of light paths with Halton samples; reuse LT bounces and RR.
		vpls.clear();
		vplLightPaths = pathCount;
		for (unsigned int i = 0; i < pathCount; i++)
		{
			HaltonSampler sampler(i + 1 + renderSeed);
			lightTrace(&sampler, -1, true);
		}
	}
	Colour computeVPLLighting(const ShadingData& shadingData)
	{
		Colour result(0, 0, 0);
		if (vplLightPaths == 0) return result;
		for (const VPL& vpl : vpls)
		{
			Vec3 delta = vpl.shadingData.x - shadingData.x;
			float distanceSq = delta.lengthSq();
			if (distanceSq <= EPSILON * EPSILON) continue;
			Vec3 wi = delta / sqrtf(distanceSq);
			float cosSurface = Dot(shadingData.sNormal, wi);
			float cosVPL = Dot(vpl.shadingData.sNormal, -wi);
			if (cosSurface <= 0 || cosVPL <= 0 || Dot(shadingData.gNormal, wi) <= 0) continue;
			float correction = importanceNormalCorrection(vpl.shadingData, -wi);
			if (correction <= 0 || !scene->visible(shadingData.x, vpl.shadingData.x)) continue;
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			Colour fVPL = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi);
			// p61-62：这里没有截断几何项，VPL 靠得太近时仍会有亮斑。
			// The geometry term is not clamped, so nearby VPLs can still cause bright spots.
			result = result + f * fVPL * vpl.Le * (cosSurface * cosVPL * correction / distanceSq);
		}
		// 除以发射路径数，而不是随机生成的 VPL 个数。
		// Divide by emitted path count, not the number of VPLs generated.
		return result / float(vplLightPaths);
	}
	Colour traceIR(Ray ray, Sampler* sampler)
	{
		Colour throughput(1, 1, 1);
		// RenderingAlgorithms p57-58：相机先穿过 delta 链，到首个非镜面点再求和。
		// Follow delta bounces from the camera; sum VPL lighting at the first non-specular hit.
		for (int depth = 0; ; depth++)
		{
			IntersectionData hit = scene->traverse(ray);
			if (hit.t == FLT_MAX) return throughput * scene->background->evaluate(ray.dir);
			ShadingData shading = scene->calculateShadingData(hit, ray);
			if (shading.bsdf->isLight()) return throughput * shading.bsdf->emit(shading, shading.wo);
			if (!shading.bsdf->isPureSpecular())
				return throughput * (computeDirect(shading, sampler) + computeVPLLighting(shading));
			Colour f;
			float pdf;
			// 相机链传 radiance，不能使用光源路径的 sampleImportance。
			// Camera paths carry radiance, so use sample rather than sampleImportance.
			Vec3 wi = shading.bsdf->sample(shading, sampler, f, pdf);
			if (!(pdf > 0.0f)) return Colour(0, 0, 0);
			throughput = throughput * f * (fabsf(Dot(wi, shading.sNormal)) / pdf);
			if (!(throughput.Lum() > 0.0f)) return Colour(0, 0, 0);
			if (depth >= 3)
			{
				float survival = std::min(throughput.Lum(), 0.95f);
				if (sampler->next() >= survival) return Colour(0, 0, 0);
				throughput = throughput / survival;
			}
			ray.init(shading.x + wi * EPSILON, wi);
		}
	}
	bool saveVPLs(const std::string& filename)
	{
		std::ofstream file(filename);
		file << "x,y,z,nx,ny,nz,wo_x,wo_y,wo_z,Le_r,Le_g,Le_b\n";
		for (const VPL& vpl : vpls)
		{
			const ShadingData& s = vpl.shadingData;
			file << s.x.x << ',' << s.x.y << ',' << s.x.z << ','
				<< s.sNormal.x << ',' << s.sNormal.y << ',' << s.sNormal.z << ','
				<< s.wo.x << ',' << s.wo.y << ',' << s.wo.z << ','
				<< vpl.Le.r << ',' << vpl.Le.g << ',' << vpl.Le.b << '\n';
		}
		return bool(file);
	}
	void renderLightTrace(int maxDepth = 0)
	{
		outputBuffer.clear();
		film->incrementSPP();
		samplers[0].generator.seed(renderSeed + unsigned(film->SPP - 1));
		// 一轮发射 width*height 条路径，抵消像素面积因子；Film 仍按轮数平均。
		// Emit width*height paths per pass to cancel the pixel-area factor; Film averages passes.
		for (unsigned int i = 0; i < film->width * film->height; i++)
			lightTrace(&samplers[0], maxDepth);
		if (canvas == NULL) return;
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void render(int requestedThreads = 0)
	{
		if (useInstantRadiosity && vplLightPaths == 0)
		{
			GamesEngineeringBase::Timer timer;
			timer.reset();
			generateVPLs(vplPaths);
			std::cout << "vpl_generation_seconds=" << timer.dt() << '\n';
			std::cout << "vpl_paths=" << vplLightPaths << " vpl_count=" << vpls.size() << '\n';
		}
		// 新一轮采样后，旧降噪图就不对应当前 Film 了。
		// Discard the old denoised image before adding more samples.
		outputBuffer.clear();
		film->incrementSPP();
		if (albedoFilm != NULL)
		{
			albedoFilm->filter = normalFilm->filter = film->filter;
			albedoFilm->incrementSPP();
			normalFilm->incrementSPP();
		}
		const int tileSize = 32;
		int tilesX = (int(film->width) + tileSize - 1) / tileSize;
		int tilesY = (int(film->height) + tileSize - 1) / tileSize;
		int tileCount = tilesX * tilesY;
		int workerCount = requestedThreads > 0 ? std::min(requestedThreads, numProcs) : numProcs;
		workerCount = std::min(workerCount, tileCount);
		// 宽滤波核串行写入，避免线程写到同一像素。
		// Use serial splats for wide filters to avoid overlapping writes.
		if (film->filter->size() > 0) workerCount = 1;
		std::atomic<int> nextTile(0);
		auto work = [&](int threadID)
		{
			while (true)
			{
				// RayTracing p87-88：用原子编号代替单独建立一个任务队列。
				// Claim tiles with an atomic counter instead of a separate task queue.
				int tile = nextTile.fetch_add(1);
				if (tile >= tileCount) break;
				int x = (tile % tilesX) * tileSize;
				int y = (tile / tilesX) * tileSize;
				// MonteCarlo p127：每线程独立 RNG；按 tile/SPP 播种使调度不改变图像。
				// Each worker has its own RNG; tile/SPP seeds keep results independent of scheduling.
				unsigned int seed = renderSeed + unsigned(film->SPP - 1) * unsigned(tileCount) + unsigned(tile);
				samplers[threadID].generator.seed(seed);
				renderTile(x, y, std::min(x + tileSize, int(film->width)),
					std::min(y + tileSize, int(film->height)), &samplers[threadID]);
			}
		};
		if (workerCount == 1)
		{
			work(0);
		} else
		{
			for (int i = 0; i < workerCount; i++)
				threads[i] = new std::thread(work, i);
			for (int i = 0; i < workerCount; i++)
			{
				threads[i]->join();
				delete threads[i];
			}
		}
		// 工作线程只写 Film；全部完成后由主线程显示，离屏测试可不创建窗口。
		// Workers only write Film; display it after they finish. Headless tests can skip the window.
		if (canvas == NULL) return;
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				// 窗口和 HDR 都用累积胶片。 Display and HDR use the same accumulated film.
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	bool prepareAOVBuffers()
	{
		// 这里只支持 BoxFilter，宽核 splat 的相关噪声不适合 OIDN RT。
		// Only BoxFilter is supported here; wide splats introduce correlated noise for OIDN RT.
		if (albedoFilm == NULL || film->SPP == 0 || film->filter->size() != 0) return false;
		// SpeedingItUp p103-104：线性 Float3，不对输入做 tonemap/gamma。
		// Pass linear Float3 inputs, without tonemapping or gamma.
		int count = film->width * film->height;
		colorBuffer.resize(count);
		albedoBuffer.resize(count);
		normalBuffer.resize(count);
		for (int i = 0; i < count; i++)
		{
			colorBuffer[i] = film->film[i] / float(film->SPP);
			albedoBuffer[i] = albedoFilm->film[i] / float(film->SPP);
			normalBuffer[i] = normalFilm->film[i] / float(film->SPP);
			// 抗锯齿后的法线平均值不要重新归一化。
			// Do not renormalize the antialiased normal average.
		}
		return true;
	}
	bool saveFloatImage(const std::string& filename, const std::vector<Colour>& pixels)
	{
		// PFM 保留有符号浮点数；Radiance HDR 不能保存负法线。
		// PFM keeps signed floats; Radiance HDR cannot store negative normal components.
		static_assert(sizeof(Colour) == 3 * sizeof(float), "Float3 must be packed");
		std::ofstream file(filename, std::ios::binary);
		file << "PF\n" << film->width << ' ' << film->height << "\n-1.0\n";
		for (int y = int(film->height) - 1; y >= 0; y--)
			file.write(reinterpret_cast<const char*>(&pixels[y * film->width]), film->width * sizeof(Colour));
		return bool(file);
	}
	bool saveAOVs(const std::string& prefix)
	{
		if (!prepareAOVBuffers()) return false;
		bool ok = saveFloatImage(prefix + "_color.pfm", colorBuffer);
		ok = saveFloatImage(prefix + "_albedo.pfm", albedoBuffer) && ok;
		ok = saveFloatImage(prefix + "_normal.pfm", normalBuffer) && ok;
		std::vector<unsigned char> albedoPixels(albedoBuffer.size() * 3),
			normalPixels(normalBuffer.size() * 3);
		for (size_t i = 0; i < albedoBuffer.size(); i++)
		{
			float a[3] = {albedoBuffer[i].r, albedoBuffer[i].g, albedoBuffer[i].b};
			float n[3] = {normalBuffer[i].r, normalBuffer[i].g, normalBuffer[i].b};
			for (int c = 0; c < 3; c++)
			{
				// 仅 PNG 预览：albedo 加显示 gamma，normal 从 [-1,1] 映射到 [0,1]。
				// PNG previews only: apply gamma to albedo and map normals from [-1,1] to [0,1].
				albedoPixels[i * 3 + c] = (unsigned char)(255.0f *
					powf(std::max(0.0f, std::min(1.0f, a[c])), 1.0f / 2.2f));
				normalPixels[i * 3 + c] = (unsigned char)(255.0f *
					std::max(0.0f, std::min(1.0f, 0.5f * n[c] + 0.5f)));
			}
		}
		ok = stbi_write_png((prefix + "_albedo.png").c_str(), film->width, film->height, 3,
			albedoPixels.data(), film->width * 3) && ok;
		ok = stbi_write_png((prefix + "_normal.png").c_str(), film->width, film->height, 3,
			normalPixels.data(), film->width * 3) && ok;
		return ok;
	}
	bool checkDenoiseError()
	{
		const char* message = nullptr;
		if (denoiseDevice.getError(message) == oidn::Error::None) return true;
		std::cerr << "OIDN: " << (message ? message : "unknown error") << std::endl;
		outputBuffer.clear();
		return false;
	}
	bool denoise(bool useAOVs = true)
	{
		outputBuffer.clear();
		if (film->SPP == 0 || film->filter->size() != 0) return false;
		if (useAOVs)
		{
			if (!prepareAOVBuffers()) return false;
		} else
		{
			colorBuffer.resize(film->width * film->height);
			for (size_t i = 0; i < colorBuffer.size(); i++)
				colorBuffer[i] = film->film[i] / float(film->SPP);
		}
		// SpeedingItUp p104：复用课程的 device/filter 流程，CPU 可直接读取现有 vector。
		// Follow the lecture's device/filter steps; the CPU device can read our vectors directly.
		if (!denoiseDevice)
		{
			denoiseDevice = oidn::newDevice(oidn::DeviceType::CPU);
			if (!denoiseDevice)
			{
				checkDenoiseError();
				return false;
			}
			denoiseDevice.commit();
			if (!checkDenoiseError())
			{
				denoiseDevice.release();
				return false;
			}
		}
		oidn::FilterRef filter = denoiseDevice.newFilter("RT");
		if (!filter)
		{
			checkDenoiseError();
			return false;
		}
		outputBuffer.resize(colorBuffer.size());
		// OIDN 2.5 将课件的 setSharedImage 改名为 setImage；指针重载仍共享 CPU 内存。
		// Use OIDN 2.5's setImage pointer overload to share CPU memory as in the lecture.
		filter.setImage("color", colorBuffer.data(), oidn::Format::Float3, film->width, film->height);
		if (useAOVs)
		{
			filter.setImage("albedo", albedoBuffer.data(), oidn::Format::Float3, film->width, film->height);
			filter.setImage("normal", normalBuffer.data(), oidn::Format::Float3, film->width, film->height);
		}
		filter.setImage("output", outputBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		// 首次交点 AOV 的边缘仍有采样噪声。 First-hit AOV edges still have sampling noise.
		filter.set("cleanAux", false);
		filter.commit();
		if (!checkDenoiseError()) return false;
		filter.execute();
		return checkDenoiseError();
	}
	bool saveDenoised(const std::string& prefix)
	{
		if (outputBuffer.empty()) return false;
		// SpeedingItUp p105：单独保存线性结果，显示时才 tonemap，不覆盖累积 Film。
		// Save the linear result separately; tonemap for display without overwriting Film.
		bool ok = saveFloatImage(prefix + ".pfm", outputBuffer);
		ok = stbi_write_hdr((prefix + ".hdr").c_str(), film->width, film->height, 3,
			reinterpret_cast<const float*>(outputBuffer.data())) && ok;
		std::vector<unsigned char> pixels(outputBuffer.size() * 3);
		for (size_t i = 0; i < outputBuffer.size(); i++)
		{
			pixels[i * 3] = (unsigned char)(tonemapChannel(outputBuffer[i].r) * 255.0f);
			pixels[i * 3 + 1] = (unsigned char)(tonemapChannel(outputBuffer[i].g) * 255.0f);
			pixels[i * 3 + 2] = (unsigned char)(tonemapChannel(outputBuffer[i].b) * 255.0f);
		}
		return stbi_write_png((prefix + ".png").c_str(), film->width, film->height, 3,
			pixels.data(), film->width * 3) && ok;
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};
