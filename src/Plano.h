#pragma once

//#include <windows.h>
#include <GL/glew.h>
#include <iostream>
//#include <thread>

//#include <opencv2\cv.hpp>
#include <opencv2/opencv.hpp>
#include "Shaders.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace cv;

class Plano
{
public:
	Plano();
	~Plano();
	void initialize(int);
	void close();
	bool createTexture(Mat);
	void createShader();
	void updateTexture(Mat);
	void drawTexture();
	void initializeUniforms();
	void applyTransformations();

private:
	GLuint vbo, vao, tex;
	CShader shaderVertex, shaderFragment;
	CShaderProgram shaderProgram;
	glm::vec2 lensCenter, screenCenter;
	GLint uniLensCenter, uniScreenCenter;
};


