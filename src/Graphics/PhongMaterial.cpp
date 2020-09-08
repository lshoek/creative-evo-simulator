#include "PhongMaterial.h"

PhongMaterial::PhongMaterial(ofFloatColor ambient, ofFloatColor diffuse, ofFloatColor specular, ofFloatColor emission, float shininess) :
	_ambient(ambient), _diffuse(diffuse), _specular(specular), _emission(emission), _shininess(shininess) {}

void PhongMaterial::setAmbient(ofFloatColor ambient)
{
	_ambient = ambient;
}

void PhongMaterial::setDiffuse(ofFloatColor diffuse)
{
	_diffuse = diffuse;
}

void PhongMaterial::setSpecular(ofFloatColor specular)
{
	_specular = specular;
}

void PhongMaterial::setEmission(ofFloatColor emission)
{
	_emission = emission;
}

void PhongMaterial::setShininess(float shininess)
{
	_shininess = shininess;
}

void PhongMaterial::setShaderUniforms(const std::shared_ptr<ofShader>& shader)
{
	shader->setUniform4f("mtl.ambient", _ambient);
	shader->setUniform4f("mtl.diffuse", _diffuse);
	shader->setUniform4f("mtl.specular", _specular);
	shader->setUniform4f("mtl.emission", _emission);
	shader->setUniform1f("mtl.shininess", _shininess);
}

PhongMaterial::~PhongMaterial() {}