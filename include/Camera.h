#pragma once
#include "GL_Util.h"
#include "quatvec.h"

class Camera
{
public:
	enum class Type
	{
		ORTHO,
		PERSP,
		NONE
	};
	Camera();
	void InitOrtho( float xMin, float xMax, float yMin, float yMax );
	void InitPersp( float fovy, float aspect, float near, float far );

	void ResetRot();
	void ResetPos();
	void ResetTransform();
	void ResetProj();
	void Reset();

	vec3 GetView() const;
	vec3 GetPos() const;
	fquat GetRot() const;
	quatvec GetTransform() const;
	mat4 GetProjMat() const;
	mat4 GetTransformMat() const;
	mat4 GetCameraMat() const;

	void Translate( vec3 t );
	void Translate( vec2 t );
	void Rotate( fquat q );

	static void SetCamMatHandle( GLint h );
	static GLint GetCamMatHandle();

private:
	Type m_eType;
	quatvec m_qvTransform;
	mat4 m_m4Proj;

	static GLint s_CamMatHandle;
};