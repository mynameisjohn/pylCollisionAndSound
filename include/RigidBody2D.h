#pragma once

#include "Contact.h"
#include "quatvec.h"

#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>

// Let's see if I can make this a POD union
struct RigidBody2D
{
	// All rigid bodies are the same type,
	// but this enum dictates what they really are
	enum EType
	{
		Circle,
		AABB,
		OBB
	};

	EType eType;		// Primitive type
	float fMass;		// Mass
	float fElast;		// Elasticity
	float fTheta;		// Rotation angle
	float rOmega;		// Angular velocity
	glm::vec2 v2Vel;	// Velocity
	glm::vec2 v2Center;	// Center position

	// The big union
	union
	{
		struct { glm::vec2 v2HalfDim; } boxData;
		struct { float fRadius; } circData;
	};

	// Default constructor
	RigidBody2D();

	// Various gets
	glm::vec2 GetMomentum() const;
	float GetKineticEnergy() const;
	quatvec GetQuatVec() const;
	glm::mat2 GetRotMat() const;

	// Until we get rid of this dumb list pattern. This is handled internally via a switch
	static std::list<Contact> GetSpeculativeContacts( const RigidBody2D * pA, const RigidBody2D * pB );

	// Interesting constructor is protected, called
	// by class static methods from child classes (?)
protected:
	RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float th = 0.f );
};

// I'm not really sure about this...
// But as long as these subclasses declare
// no internal members I think we're good
struct Circle : public RigidBody2D
{
	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius, float th = 0.f );

protected:
	Circle( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius, float th = 0.f );
};

struct AABB
{
	// useful things
	float Width() const;
	float Height() const;
	float Left() const;
	float Right() const;
	float Top() const;
	float Bottom() const;
	glm::vec2 Clamp( glm::vec2 p ) const;
	glm::vec2 GetFaceNormalFromPoint( glm::vec2 p ) const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h );

protected:
	AABB( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R );
	AABB( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h );
};

struct OBB
{
	glm::vec2 WorldSpaceClamp() const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R, float th = 0.f );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h, float th = 0.f );

protected:
	OBB( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R, float th = 0.f );
	OBB( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h, float th = 0.f );
};