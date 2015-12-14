#include "Plano.h"

Plano::Plano()
{
}

void Plano::initialize(int numV)
{
	float vertices1[]
	{
		//  Position      Color             Texcoords
		-1.0f,  1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, // Top-left
		-1.0f,  -1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, // Top-right
		0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, // Bottom-right
		0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f  // Bottom-left
	};
	float vertices2[] =
	{
		//  Position      Color             Texcoords
		0.0f,  1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, // Top-left
		0.0f,  -1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f, // Top-right
		1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, // Bottom-right
		1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f  // Bottom-left
	};

	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	if(numV==1)
	{
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices1), vertices1, GL_STATIC_DRAW);
		lensCenter=glm::vec2(0.55, 0.5);
		screenCenter=glm::vec2(0.25, 0.5);

	}
	else if(numV==2)
	{
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices2), vertices2, GL_STATIC_DRAW);
		lensCenter=glm::vec2(0.45, 0.5);
		screenCenter=glm::vec2(0.75, 0.5);
	}
	glBindVertexArray(vao);

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
}

bool Plano::createTexture(Mat frame)
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, tex);

	GLenum minFilter = GL_NEAREST;
	GLenum magFilter = GL_NEAREST;
	GLenum wrapFilter = GL_CLAMP;

	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
		magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		//cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

	GLenum inputColourFormat = GL_BGR;
	if (frame.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}

	glTexImage2D(GL_TEXTURE_2D,
		0,
		GL_RGB,
		frame.cols,
		frame.rows,
		0,
		inputColourFormat,
		GL_UNSIGNED_BYTE,
		frame.ptr());

	if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
		minFilter == GL_LINEAR_MIPMAP_NEAREST ||
		minFilter == GL_NEAREST_MIPMAP_LINEAR ||
		minFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		glGenerateMipmap(GL_TEXTURE_2D);
	}

	return true;
}

void Plano::createShader()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, tex);

	shaderVertex.LoadShader("/home/alvarovito/catkin_ws/src/augmentedVision/src/data/shaders/material1.vert", GL_VERTEX_SHADER);
	shaderFragment.LoadShader("/home/alvarovito/catkin_ws/src/augmentedVision/src/data/shaders/material1.frag", GL_FRAGMENT_SHADER);

	shaderProgram.CreateProgram();
	shaderProgram.AddShaderToProgram(&shaderVertex);
	shaderProgram.AddShaderToProgram(&shaderFragment);

	shaderProgram.LinkProgram();
	shaderProgram.UseProgram();

	shaderProgram.AsignAttribute("position", 2, GL_FLOAT, 7 * sizeof(float), 0);
	shaderProgram.AsignAttribute("color", 3, GL_FLOAT, 7 * sizeof(float), (void*)(2 * sizeof(float)));
    shaderProgram.AsignAttribute("texcoord", 2, GL_FLOAT, 7 * sizeof(float), (void*)(5 * sizeof(float)));
}

void Plano::updateTexture(Mat texture)
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, tex);

	glTexImage2D(GL_TEXTURE_2D,
		0,
		GL_RGB,
		texture.cols,
		texture.rows,
		0,
		GL_BGR,
		GL_UNSIGNED_BYTE,
		texture.ptr());
}

void Plano::initializeUniforms()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, tex);

	uniLensCenter = glGetUniformLocation(shaderProgram.GetProgramID(), "LensCenter");
	glUniform2fv(uniLensCenter, 1, glm::value_ptr(lensCenter));
	
	uniScreenCenter = glGetUniformLocation(shaderProgram.GetProgramID(), "ScreenCenter");
	glUniform2fv(uniScreenCenter, 1, glm::value_ptr(screenCenter));
}

void Plano::applyTransformations()
{
	glUniform2fv(uniLensCenter, 1, glm::value_ptr(lensCenter));
}

void Plano::drawTexture()
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, tex);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void Plano::close()
{
	glDeleteTextures(1, &tex);

	shaderProgram.DeleteProgram();
	shaderFragment.DeleteShader();
	shaderVertex.DeleteShader();

	glDeleteBuffers(1, &vbo);
	glDeleteVertexArrays(1, &vao);
}

Plano::~Plano()
{
}

