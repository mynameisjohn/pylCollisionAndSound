#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform.hpp>

struct quatvec
{
	enum class Type
	{
		TR,	// Transform = T * R
		RT,	// Transform = R * T
		TRT	// Transform = T * R * Inv(T)
	};

	Type eType{ Type::TR };
	glm::vec3 vec{ 0, 0, 0 };
	glm::fquat quat{ 1, 0, 0, 0 };

	quatvec( Type t = Type::TR ) :
		eType( Type::TR ),
		vec( 0, 0, 0 ),
		quat( 1, 0, 0, 0 )
	{}

	quatvec( glm::vec3 v, glm::fquat q, Type t = Type::TR ):
		eType( t ),
		vec(v),
		quat(q)
	{}

	quatvec( glm::fquat q, glm::vec3 v, Type t = Type::RT ) :
		eType( t ),
		vec( v ),
		quat( q )
	{}

	inline glm::mat4 ToMat4() const
	{
		switch ( eType )
		{
			case Type::TR:
				
			case Type::RT:
				return glm::mat4_cast( quat ) * glm::translate( vec );
			case Type::TRT:
				return glm::translate( vec ) * glm::mat4_cast( quat ) * glm::translate( -vec );
		}

		return glm::mat4( 1 );
	}
};