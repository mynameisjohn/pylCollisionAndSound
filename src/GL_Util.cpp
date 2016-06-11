#include "GL_Util.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Implement GLM print functions
std::ostream& operator<<( std::ostream& os, const vec2& vec )
{
	os << "< " << vec.x << ", " << vec.y << " >";
	return os;
}
std::ostream& operator<<( std::ostream& os, const vec3& vec )
{
	os << "< " << vec.x << ", " << vec.y << ", " << vec.z << " >";
	return os;
}
std::ostream& operator<<( std::ostream& os, const vec4& vec )
{
	os << "< " << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << " >";
	return os;
}
std::ostream& operator<<( std::ostream& os, const mat4& mat )
{
	// These are stored in a transposed form, so transpose locally
	glm::mat4 t = glm::transpose( mat );
	os << t[0] << "\n" << t[1] << "\n" << t[2] << "\n" << t[3];
	return os;
}
std::ostream& operator<<( std::ostream& os, const fquat& quat )
{
	os << "< " << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << " >";
	return os;
}
