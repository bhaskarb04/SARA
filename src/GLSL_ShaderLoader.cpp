/*
 *  GLSL_ShaderLoader.cpp
 *  GLSL_Shader_Test
 *
 *  Created by Dr.-Ing. Rafael Radkowski on 31.07.10.
 *  Copyright 2010 Heinz Nixdorf Institute. All rights reserved.
 *
 */

#include "GLSL_ShaderLoader.h"




/*!
	 Constructor
	 @param shaderStateSet - the OSG stateset that should be affected by this shader
	 @param shaderName - a string that contains the name of that shader
	 @param vertexProgram - the vertx shader program file
	 @param fragProgram - the fragment shader program
*/
GLSL_ShaderLoader::GLSL_ShaderLoader( osg::StateSet* shaderStateSet, std::string shaderName, std::string vertexProgram, std::string fragProgram):
	_shaderStateSet(shaderStateSet), _name(shaderName), _vertObjFile(vertexProgram), _fragObjFile(fragProgram)
{
	
	// Creates a shader program object
	_shaderProgram = new osg::Program();

	// Pass the name to that object
	_shaderProgram->setName( shaderName );

	// Creates a vertex and a fragment shader operation
	_vertObj = new osg::Shader( osg::Shader::VERTEX );
	_fragObj = new osg::Shader( osg::Shader::FRAGMENT );

	// Add this shader to the program
	_shaderProgram->addShader( _fragObj );
	_shaderProgram->addShader( _vertObj );
	
	// Load the shader from a shource file. 
	LoadShaderSource( _vertObj, _vertObjFile );
    LoadShaderSource( _fragObj, _fragObjFile );
	
	// Apply the shader program on a stateset
	_shaderStateSet->setAttributeAndModes(_shaderProgram, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	
}

// Destructor
GLSL_ShaderLoader::~GLSL_ShaderLoader()
{
	
}



/**
	 * Load the shader code from file
	 @param shader - the osg::Shader object that refers to that shader
	 @param fileName - the name and path of the shader code on hard disk. 
*/
void GLSL_ShaderLoader::LoadShaderSource( osg::Shader* shader, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() != 0 )
    {
        shader->loadShaderSourceFromFile( fqFileName.c_str() );
    }
    else
    {
        osg::notify(osg::WARN) << "File \"" << fileName << "\" not found." << std::endl;
    }
}