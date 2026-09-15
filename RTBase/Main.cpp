

#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

void runTests()
{
	// Add test code here
}

int main(int argc, char *argv[])
{
	// Add call to tests if required
	// runTests()
	
	// Initialize default parameters
	std::string sceneName = "cornell-box";
	//std::string sceneName = "MaterialsScene";
	std::string filename = "GI.hdr";
	std::string aovPrefix;
	std::string denoiseMode = "off";
	std::string integrator = "path";
	int vplPaths = 64;
	unsigned int SPP = 8192;
	int renderThreads = 0; // 0 uses the available logical processors; 1 is the serial comparison.

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
			if (pair.first == "-threads")
			{
				renderThreads = std::max(0, stoi(pair.second));
			}
			if (pair.first == "-aovPrefix") aovPrefix = pair.second;
			if (pair.first == "-denoise") denoiseMode = pair.second;
			if (pair.first == "-integrator") integrator = pair.second;
			if (pair.first == "-vplPaths") vplPaths = stoi(pair.second);
		}
	}
	if (denoiseMode != "off" && denoiseMode != "color" &&
		denoiseMode != "guided" && denoiseMode != "both")
	{
		std::cerr << "-denoise must be off, color, guided or both" << std::endl;
		return 1;
	}
	if (integrator != "path" && integrator != "light-direct" &&
		integrator != "light-one-bounce" && integrator != "light" && integrator != "ir")
	{
		std::cerr << "-integrator must be path, light-direct, light-one-bounce, light or ir" << std::endl;
		return 1;
	}
	bool isIR = integrator == "ir";
	bool isLightTrace = integrator != "path" && !isIR;
	if (isIR && vplPaths <= 0)
	{
		std::cerr << "-vplPaths must be positive" << std::endl;
		return 1;
	}
	// -1 用 RR 结束，0 只直连，1 反弹一次。
	// -1 uses RR; 0 connects directly; 1 allows one bounce.
	int lightDepth = -1;
	if (integrator == "light-direct") lightDepth = 0;
	if (integrator == "light-one-bounce") lightDepth = 1;
	if ((isLightTrace || isIR) && (!aovPrefix.empty() || denoiseMode != "off"))
	{
		std::cerr << "LT/IR currently run without AOVs or denoising" << std::endl;
		return 1;
	}
	Scene* scene = loadScene(sceneName);
	if (isLightTrace || isIR)
	{
		if (scene->lights.empty()) return 1;
		for (Light* light : scene->lights)
		{
			if (!light->isArea())
			{
				std::cerr << "LT/IR currently require area lights; environment emission is not implemented" << std::endl;
				return 1;
			}
		}
	}
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", false);
	RayTracer rt;
	rt.init(scene, &canvas, !aovPrefix.empty() || denoiseMode == "guided" || denoiseMode == "both");
	rt.useInstantRadiosity = isIR;
	rt.vplPaths = vplPaths;
	bool running = true;
	GamesEngineeringBase::Timer timer;
	while (running)
	{
		canvas.checkInput();
		canvas.clear();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
		}
		// Time how long a render call takes
		timer.reset();
		if (isLightTrace) rt.renderLightTrace(lightDepth);
		else rt.render(renderThreads);
		float t = timer.dt();
		// Write
		std::cout << t << std::endl;
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			if (isLightTrace || isIR)
				rt.savePNG(filename.substr(0, filename.find_last_of('.')) + ".png");
			if (isIR && !rt.saveVPLs(filename.substr(0, filename.find_last_of('.')) + "_vpls.csv"))
			{
				std::cerr << "Could not save VPL data" << std::endl;
				return 1;
			}
			if (!aovPrefix.empty() && !rt.saveAOVs(aovPrefix))
			{
				std::cerr << "Could not save AOVs to " << aovPrefix << std::endl;
				return 1;
			}
			if (denoiseMode != "off")
			{
				std::string base = filename.substr(0, filename.find_last_of('.'));
				// 留下同一 SPP 的原图，方便比较降噪效果。
				// Keep the raw image at the same SPP for the denoising comparison.
				rt.savePNG(base + ".png");
				for (const std::string mode : {std::string("color"), std::string("guided")})
				{
					if (denoiseMode != "both" && denoiseMode != mode) continue;
					timer.reset();
					if (!rt.denoise(mode == "guided"))
					{
						std::cerr << "Denoising failed; raw HDR was preserved" << std::endl;
						return 1;
					}
					std::cout << "denoise_" << mode << "_seconds=" << timer.dt() << std::endl;
					if (!rt.saveDenoised(base + "_denoised_" + mode))
					{
						std::cerr << "Could not save denoised images" << std::endl;
						return 1;
					}
				}
			}
			break;
		}
		canvas.present();
	}
	return 0;
}
