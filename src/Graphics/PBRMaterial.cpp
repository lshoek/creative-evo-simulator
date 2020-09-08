#include "PBRMaterial.h"

PBRMaterial::PBRMaterial() {}
PBRMaterial::PBRMaterial(ofFloatColor albedo, float metallic, float roughness, float ao) :
	_albedo(albedo), _metallic(metallic), _roughness(roughness), _ao(ao) {}

void PBRMaterial::setup(std::string albedoPath, std::string normalPath, std::string metallicPath, std::string roughnessPath, std::string aoPath)
{
	ofImageLoadSettings settings;
	settings.grayscale = false;

	ofPixels pixelBuffer;
	pixelBuffer.allocate(1, 1, 1);
	pixelBuffer[0] = 255;
	_dummy.allocate(pixelBuffer);

	_albedoMap = std::make_unique<ofTexture>();
	_normalMap = std::make_unique<ofTexture>();
	_metallicMap = std::make_unique<ofTexture>();
	_roughnessMap = std::make_unique<ofTexture>();
	_aoMap = std::make_unique<ofTexture>();

	loadMap(_albedoMap, albedoPath, settings);
	loadMap(_normalMap, normalPath, settings);

	settings.grayscale = true;
	loadMap(_roughnessMap, roughnessPath, settings);
	loadMap(_metallicMap, metallicPath, settings);
	loadMap(_aoMap, aoPath, settings);

	_bUseMaps = true;
}

void PBRMaterial::loadMap(std::unique_ptr<ofTexture>& mapPtr, const std::string& path, const ofImageLoadSettings& settings)
{
	if (path != "") {
		ofLoadImage(*mapPtr, path, settings);
	}
	else {
		mapPtr = std::make_unique<ofTexture>(_dummy);
	}
	mapPtr->generateMipmap();
	mapPtr->setTextureMinMagFilter(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
	mapPtr->setTextureWrap(GL_REPEAT, GL_REPEAT);
}

void PBRMaterial::setAlbedo(ofFloatColor albedo)
{
	_albedo = albedo;
}

void PBRMaterial::setMetallic(float metallic)
{
	_metallic = metallic;
}

void PBRMaterial::setRoughness(float roughness)
{
	_roughness = roughness;
}

void PBRMaterial::setAmbientOcclusion(float ao)
{
	_ao = ao;
}

void PBRMaterial::setNormalMapMult(float mult)
{
	_normalMapMult = mult;
}

void PBRMaterial::setShaderUniforms(const std::shared_ptr<ofShader>& shader)
{
	if (!_bUseMaps) {
		shader->setUniform4f("mtl.albedo", _albedo);
		shader->setUniform1f("mtl.metallic", _metallic);
		shader->setUniform1f("mtl.roughness", _roughness);
		shader->setUniform1f("mtl.ao", _ao);
	}
	else {
		shader->setUniformTexture("albedoMap", *_albedoMap, 2);
		shader->setUniformTexture("normalMap", *_normalMap, 3);
		shader->setUniformTexture("metallicMap", *_metallicMap, 4);
		shader->setUniformTexture("roughnessMap", *_roughnessMap, 5);
		shader->setUniformTexture("aoMap", *_aoMap, 6);

		shader->setUniform1f("normalMapMult", _normalMapMult);
	}
}

PBRMaterial::~PBRMaterial() {}
