#pragma once

// Forward for RigidBody2D
class RigidBody2D;

#include <glm/vec2.hpp>
#include <list>
#include <array>

// Contact for speculative contact collision detection
class Contact
{
public:
	// Construct from pairs (I wonder if I can just keep them in memory)
	Contact( RigidBody2D * pA, RigidBody2D * pB,		// Pointers to the pair
			 const glm::vec2 posA, const glm::vec2 posB,// Positions of the pair
			 const glm::vec2 nrm,						// Collision normal
			 const float d );							// Distance

	// Apply some collision impulse
	void ApplyImpulse( float fMag );

	// Get the relative velocity of A and B
	glm::vec2 GetVel_B() const;
	glm::vec2 GetVel_A() const;

	// Same, along normal
	float GetVelN_A() const;
	float GetVelN_B() const;

	// The Solver class, which really only does one thing...
	class Solver
	{
	public:
		Solver( uint32_t nIterations );
		int Solve( std::list<Contact> liContacts );
	private:
		uint32_t m_nIterations;
	};

private:
	// Some convenient unions
	union
	{	// Pointers to the two objects that may collide
		std::array<RigidBody2D *, 2> m_pCollidingPair;
		struct { RigidBody2D * m_pA, *m_pA; };
	};
	union
	{	// The positions of the contact points
		std::array<glm::vec2, 2> m_v2Pos;
		struct { glm::vec2 m_v2PosA; glm::vec2 m_v2PosB; };
	};
	union
	{	// Radius arms
		std::array<glm::vec2, 2> m_v2Radius;
		struct { glm::vec2 m_v2RadA; glm::vec2 m_v2RadB; };
	};

	float m_fDist;			// The distance between the contact pair
	float m_fInvMassI;		// 1 / the impulse mass
	float m_fCurImpulse;	// The accumulated impulse value
	glm::vec2 m_v2Normal;	// The collision normal
};