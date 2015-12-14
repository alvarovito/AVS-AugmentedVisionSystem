#include "Shaders.h"
#include <stdio.h>

CShader::CShader()
{
	bLoaded = false;
}

bool CShader::LoadShader(string nameFile, int a_iType)
{
	FILE* fp = fopen(nameFile.c_str(), "r");
	if (!fp)return false;

	// Get all lines from a file

	vector<string> sLines;
	char sLine[255];
	while (fgets(sLine, 255, fp))
		sLines.push_back(sLine);
	fclose(fp);

	const char** sProgram = new const char*[(int)sLines.size()];

	for(int i=0; i<(int)sLines.size(); i++)
		sProgram[i] = sLines[i].c_str();

	uiShader = glCreateShader(a_iType);

	glShaderSource(uiShader, (int)sLines.size(), sProgram, NULL);
	glCompileShader(uiShader);

	delete[] sProgram;

	int iCompilationStatus;
	glGetShaderiv(uiShader, GL_COMPILE_STATUS, &iCompilationStatus);

	if (iCompilationStatus == GL_FALSE)return false;
	iType = a_iType;
	bLoaded = true;

	return 1;
}

bool CShader::IsLoaded()
{
	return bLoaded;
}

GLuint CShader::GetShaderID()
{
	return uiShader;
}

void CShader::DeleteShader()
{
	if (!IsLoaded())return;
	bLoaded = false;
	glDeleteShader(uiShader);
}

//////////////////////////////////////////////////////////////

CShaderProgram::CShaderProgram()
{
	bLinked = false;
}

void CShaderProgram::CreateProgram()
{
	uiProgram = glCreateProgram();
}

bool CShaderProgram::AddShaderToProgram(CShader* shShader)
{
	if (!shShader->IsLoaded())return false;

	glAttachShader(uiProgram, shShader->GetShaderID());

	return true;
}

bool CShaderProgram::LinkProgram()
{
	glLinkProgram(uiProgram);
	int iLinkStatus;
	glGetProgramiv(uiProgram, GL_LINK_STATUS, &iLinkStatus);
	bLinked = iLinkStatus == GL_TRUE;
	return bLinked;
}

void CShaderProgram::AsignAttribute(const GLchar *name, int size, GLenum tipo, int stride, void* pointer)
{
	GLint attrib = glGetAttribLocation(uiProgram, name);
	glVertexAttribPointer(attrib, size, tipo, GL_FALSE, stride, pointer);
	glEnableVertexAttribArray(attrib);
}

void CShaderProgram::DeleteProgram()
{
	if (!bLinked)return;
	bLinked = false;
	glDeleteProgram(uiProgram);
}

void CShaderProgram::UseProgram()
{
	if (bLinked)glUseProgram(uiProgram);
}

GLuint CShaderProgram::GetProgramID()
{
	return uiProgram;
}
