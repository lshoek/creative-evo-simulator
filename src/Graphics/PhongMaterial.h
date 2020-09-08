#pragma once
#include "MaterialBase.h"
#include "ofVectorMath.h"
#include "ofColor.h"
#include "ofShader.h"

class PhongMaterial : public MaterialBase
{
public:
	PhongMaterial(ofFloatColor ambient, ofFloatColor diffuse, ofFloatColor specular, ofFloatColor emission, float shininess);
	~PhongMaterial();

	void setAmbient(ofFloatColor ambient);
	void setDiffuse(ofFloatColor diffuse);
	void setSpecular(ofFloatColor specular);
	void setEmission(ofFloatColor emission);
	void setShininess(float shininess);

	virtual void setShaderUniforms(const std::shared_ptr<ofShader>& shader) override;

private:
	ofFloatColor _ambient;
	ofFloatColor _diffuse;
	ofFloatColor _specular;
	ofFloatColor _emission;
	float _shininess;
};
