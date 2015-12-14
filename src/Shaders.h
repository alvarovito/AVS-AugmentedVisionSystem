#pragma once

//#include <windows.h>
#include <GL/glew.h>
#include <cstring>
#include <sstream>
#include <vector>


using namespace std;

class CShader
{
public:
	CShader();

	bool LoadShader(string, int);
	void DeleteShader();

	bool IsLoaded();
	GLuint GetShaderID();

private:
	GLuint uiShader; // ID of shader
	int iType; // GL_VERTEX_SHADER, GL_FRAGMENT_SHADER...
	bool bLoaded; // Whether shader was loaded and compiled
};

class CShaderProgram
{
public:
	CShaderProgram();

	void CreateProgram();
	void DeleteProgram();

	bool AddShaderToProgram(CShader* shShader);
	bool LinkProgram();
	void AsignAttribute(const GLchar *name, int size, GLenum tipo, int stride, void* pointer);

	void UseProgram();

	GLuint GetProgramID();

private:
	GLuint uiProgram; // ID of program
	bool bLinked; // Whether program was linked and is ready to use
};


