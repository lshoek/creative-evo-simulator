#pragma once
#include "MaterialBase.h"
#include "ofVectorMath.h"
#include "ofColor.h"
#include "ofShader.h"
#include "ofTexture.h"
#include "ofImage.h"

class PBRMaterial : public MaterialBase
{
public:
	PBRMaterial();
	PBRMaterial(ofFloatColor albedo, float metallic, float roughness, float ao);
	~PBRMaterial();

	void setup(std::string albedoPath, std::string normalPath, std::string metallicPath, std::string roughnessPath, std::string aoPath);

	void setAlbedo(ofFloatColor albedo);
	void setMetallic(float metallic);
	void setRoughness(float roughness);
	void setAmbientOcclusion(float ao);

	void setNormalMapMult(float mult);

	virtual void setShaderUniforms(const std::shared_ptr<ofShader>& shader) override;

private:
	void loadMap(std::unique_ptr<ofTexture>& mapPtr, const std::string& path, const ofImageLoadSettings& settings);

	ofFloatColor _albedo = ofFloatColor(1.0f);
	float _metallic = 1.0f;
	float _roughness = 1.0f;
	float _ao = 1.0f;
	float _normalMapMult = 1.0f;

	std::unique_ptr<ofTexture> _albedoMap;
	std::unique_ptr<ofTexture> _normalMap;
	std::unique_ptr<ofTexture> _metallicMap;
	std::unique_ptr<ofTexture> _roughnessMap;
	std::unique_ptr<ofTexture> _aoMap;

	ofTexture _dummy;

	bool _bUseMaps = false;
};
