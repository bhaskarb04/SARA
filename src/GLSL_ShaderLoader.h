/*
 *  GLSL_ShaderLoader.h
 *  GLSL_Shader_Test
 *
 *  This class reads a shader program from file and pass it to a
 *  stateset object. 
 *
 *  Created by Dr.-Ing. Rafael Radkowski on 31.07.10.
 *  Copyright 2010 Heinz Nixdorf Institute. All rights reserved.
 *
 */
#pragma once
#include <string>
#include <osg/Node>
#include <osg/Referenced>
#include <osg/ref_ptr>
#include <osg/Program>
#include <osg/Shader>
#include <osgDB/FileUtils>

class GLSL_ShaderLoader
{
private:
	
	//----------------------------------------
	// The shader programm 
	osg::Program* 	_shaderProgram;
	
	//----------------------------------------
	// The vertex shader program
	osg::Shader*  	_vertObj;
	
	//----------------------------------------
	// The fragment shader program
	osg::Shader*  	_fragObj;
	
	//----------------------------------------
	// The stateset, that should be affected by this shader
	osg::StateSet*	_shaderStateSet;
	
	//----------------------------------------
	// The name of this shader and the filename 
	// of the shader programs
	std::string		_name;
	std::string		_vertObjFile;
	std::string		_fragObjFile;
	
	//////////////////////////////////
	// Methods
	
	/**
	 * Load the shader code from file
	 @param shader - the osg::Shader object that refers to that shader
	 @param fileName - the name and path of the shader code on hard disk. 
	 */
	void LoadShaderSource(  osg::Shader* shader, const std::string& fileName );
	
public:
	/*!
	 Constructor
	 @param shaderStateSet - the OSG stateset that should be affected by this shader
	 @param shaderName - a string that contains the name of that shader
	 @param vertexProgram - the vertx shader program file
	 @param fragProgram - the fragment shader program
	 */
	GLSL_ShaderLoader(osg::StateSet* shaderStateSet, std::string shaderName, std::string vertexProgram, std::string fragProgram);
	
	/*!
	Destruktor
	*/
	~GLSL_ShaderLoader();
	
	
};


	
		
		



		
	
	
		
		
		
	
		
	
	