#include "Camera.h"

// Declare static shader var
/*static*/ GLint Camera::s_CamMatHandle;

// This comes up enough
using glm::normalize;

void Camera::InitOrtho( float xMin, float xMax, float yMin, float yMax )
{
	Reset();
	m_eType = Type::ORTHO;
	m_m4Proj = glm::ortho( xMin, xMax, yMin, yMax );
}

void Camera::InitPersp( float fovy, float aspect, float near, float far )
{
	Reset();
	m_eType = Type::PERSP;
	m_m4Proj = glm::perspective( fovy, aspect, near, far );
}

// See how this would affect a vector pointing out in z
vec3 Camera::GetView() const
{
	return vec3( m_m4Proj * vec4( 0, 0, 1, 1 ) );
}

Camera::Camera()
{
	Reset();
}

void Camera::Reset()
{
	m_eType = Type::NONE;
	ResetTransform();
	ResetProj();
}

void Camera::ResetPos()
{
	m_qvTransform.vec = vec3( 0 );
}

void Camera::ResetRot()
{
	m_qvTransform.quat = fquat( 1, 0, 0, 0 );
}

void Camera::ResetTransform()
{
	m_qvTransform = quatvec( quatvec::Type::RT );
}

void Camera::ResetProj()
{
	m_m4Proj = mat4( 1 );
}

// Get at the quatvec
vec3 Camera::GetPos() const
{
	return m_qvTransform.vec;
}
fquat Camera::GetRot() const
{
	return m_qvTransform.quat;
}
quatvec Camera::GetTransform() const
{
	return m_qvTransform;
}
mat4 Camera::GetTransformMat() const
{
	return m_qvTransform.ToMat4();
}

// return proj as is
mat4 Camera::GetProjMat() const
{
	return m_m4Proj;
}

mat4 Camera::GetCameraMat() const
{
	return GetProjMat() * GetTransformMat();
}

// These may be wrong, but I have to figure out why
void Camera::Translate( vec3 t )
{
	m_qvTransform.vec += t;
}

void Camera::Translate( vec2 t )
{	
	m_qvTransform.vec += vec3( t, 0 );
}

void Camera::Rotate( fquat q )
{
	m_qvTransform.quat *= q;
}

/*static*/ void Camera::SetCamMatHandle( GLint h )
{
	s_CamMatHandle = h;
}

/*static*/ GLint Camera::GetCamMatHandle()
{
	return s_CamMatHandle;
}