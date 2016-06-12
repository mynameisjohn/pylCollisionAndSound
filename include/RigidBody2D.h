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
	enum class EType : int
	{
		None,
		Circle,
		AABB,
		OBB
	};

	EType eType;		// Primitive type
	float fMass;		// Mass
	float fElast;		// Elasticity
	float fTheta;		// Rotation angle
	float fOmega;		// Angular velocity
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
	float GetInertia() const;

	// Until we get rid of this dumb list pattern. This is handled internally via a switch
	static std::list<Contact> GetSpeculativeContacts( const RigidBody2D * pA, const RigidBody2D * pB );

	// Interesting constructor is protected, called
	// by class static methods from child classes (?)
protected:
	RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float th = 0.f );
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float th = 0.f );
};

// I'm not really sure about this...
// But as long as these subclasses declare
// no internal members I think we're good
struct Circle : public RigidBody2D
{
	Circle() = delete;
	float Radius() const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius );
};

struct AABB : public RigidBody2D
{
	AABB() = delete;

	// useful things
	float Width() const;
	float Height() const;
	float Left() const;
	float Right() const;
	float Top() const;
	float Bottom() const;
	glm::vec2 Clamp( const glm::vec2 p ) const;
	glm::vec2 GetFaceNormalFromPoint( const glm::vec2 p ) const;
	glm::vec2 HalfDim() const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h );
};

struct OBB : public AABB
{
	OBB() = delete;

	glm::vec2 WorldSpaceClamp( const glm::vec2 p ) const;

	// Static creation function, returns a RigidBody2D
	static RigidBody2D Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R, float th = 0.f );
	static RigidBody2D Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h, float th = 0.f );
};

struct FeaturePair
{
	enum class EType : int
	{
		F_V,
		V_F,
		N
	};
	float dist;
	float c_dist;
	int fIdx;
	int vIdx;
	EType T;
	FeaturePair( float d, float cd = -1, int f = -1, int v = -1, EType t = EType::N );
};