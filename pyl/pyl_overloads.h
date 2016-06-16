#pragma once

#include "pyliason.h"
#include <glm/fwd.hpp>
#include "../include/SoundManager.h"
#include "../include/RigidBody2D.h"
#include "../include/Contact.h"
#include "../include/quatvec.h"

namespace pyl
{
	bool convert( PyObject *, glm::vec2& );
	bool convert( PyObject *, glm::vec3& );
	bool convert( PyObject *, glm::vec4& );
	bool convert( PyObject *, glm::fquat& );
	bool convert( PyObject *, quatvec& );
	bool convert( PyObject *, RigidBody2D::EType& );
	bool convert( PyObject *, SoundManager::ECommandID& );

	PyObject * alloc_pyobject( const SoundManager::ECommandID );
	PyObject * alloc_pyobject( const RigidBody2D::EType );
	PyObject * alloc_pyobject( const quatvec& );
}