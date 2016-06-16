#pragma once

// Forward for RigidBody2D
class RigidBody2D;

// Forward for debugging
class Drawable;

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

	float GetDistance() const;
	float GetCoefRest() const;
	float GetInertialDenom() const;
	float GetCurImpulse() const;

	const RigidBody2D * GetBodyA() const;
	const RigidBody2D * GetBodyB() const;


	// The Solver class, which really only does one thing...
	class Solver
	{
	public:
		Solver();
		Solver( uint32_t nIterations );
		uint32_t Solve( std::list<Contact>& liContacts );
	private:
		uint32_t m_nIterations;
	};

	// Init a drawable, for debugging purposes
	void InitDrawable( std::array<Drawable *, 2> drPtrArr ) const;

	bool IsColliding() const;
	class Solver;
protected:
	void setIsColliding( bool bCollidng );

private:
	// Some convenient unions
	union
	{	// Pointers to the two objects that may collide
		std::array<RigidBody2D *, 2> m_pCollidingPair;
		struct { RigidBody2D * m_pA, *m_pB; };
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

	bool m_bIsColliding;
	float m_fDist;			// The distance between the contact pair
	float m_fInvMassI;		// 1 / the impulse mass
	float m_fCurImpulse;	// The accumulated impulse value
	glm::vec2 m_v2Normal;	// The collision normal

	// internal contact vel function
	glm::vec2 getContactVel( int i ) const;
};