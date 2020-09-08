#pragma once
#include "ofShader.h"

class MaterialBase
{
public:
	virtual void MaterialBase::setShaderUniforms(const std::shared_ptr<ofShader>& shader) = 0;
};
