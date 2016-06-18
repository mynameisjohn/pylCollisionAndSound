#include "Contact.h"
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include "Drawable.h"
#include <glm/vec4.hpp>
#include <iostream>

Contact::Contact( RigidBody2D * pA, RigidBody2D * pB, const vec2 posA, const vec2 posB, const vec2 nrm, const float d ) :
	m_bIsColliding( false ),
	m_pCollidingPair{ pA, pB },
	m_v2Pos{ posA, posB },
	m_v2Normal( nrm ),
	m_fDist( d ),
	m_fCurImpulse( 0 )
{
	// How should I handle this?
	if ( pA == pB )
		throw std::runtime_error( "Error: Contact created from one object!" );

	// Find the inverse denom
	float fDenom( 0.f );
	for ( size_t i = 0; i < 2; i++ )
	{
		// The radius arm is the vector from the object's center to the contact
		m_v2Radius[i] = perp( m_v2Pos[i] - m_pCollidingPair[i]->v2Center );

		// The inverse mass denominator is a coeffecient used to calculate impulses
		// and depends on the object's inertia intertia, and it has a translation/rotation component
		fDenom += 1.f / m_pCollidingPair[i]->fMass;
		float rN = glm::dot( m_v2Radius[i], m_v2Normal );
		fDenom += powf( rN, 2 ) / m_pCollidingPair[i]->GetInertia();
	}
	
	// Is this necessary?
	if ( fDenom < kEPS )
		throw std::runtime_error( "Error: Invalid inertial denominator calculated!" );

	// Set the inverse mass denom
	m_fInvMassI = 1.f / fDenom;
}

void Contact::ApplyImpulse( float fMag )
{
	// Find the direction along our collision normal
	vec2 v2Impulse = fMag * m_v2Normal;

	// Apply to both objects in opposing directions
	m_pA->v2Vel -= v2Impulse / m_pA->fMass;
	m_pB->v2Vel += v2Impulse / m_pB->fMass;

	// Add impulse to local var
	m_fCurImpulse += fMag;
}

vec2 Contact::getContactVel( int i ) const
{
	return m_pCollidingPair[i]->v2Vel + m_v2Radius[i] * m_pCollidingPair[i]->fOmega;
}

vec2 Contact::GetVel_A() const
{
	return getContactVel( 0 );
}

vec2 Contact::GetVel_B() const
{
	return getContactVel( 1 );
}

float Contact::GetVelN_A() const
{
	return glm::dot( GetVel_A(), m_v2Normal );
}

float Contact::GetVelN_B() const
{
	return glm::dot( GetVel_B(), m_v2Normal );
}

float Contact::GetDistance() const
{
	return m_fDist;
}

float Contact::GetCoefRest() const
{
	return m_pA->fElast + m_pB->fElast;
}

float Contact::GetInertialDenom() const
{
	return m_fInvMassI;
}

float Contact::GetCurImpulse() const
{
	return m_fCurImpulse;
}

const RigidBody2D * Contact::GetBodyA() const
{
	return m_pA;
}

const RigidBody2D * Contact::GetBodyB() const
{
	return m_pB;
}

bool Contact::IsColliding() const
{
	return m_bIsColliding;
}

void Contact::setIsColliding( bool bColliding )
{
	m_bIsColliding = bColliding;
}

// Contact Solver
Contact::Solver::Solver():
	m_nIterations(0)
{}

Contact::Solver::Solver( uint32_t nIterations ) :
	m_nIterations( nIterations )
{}

uint32_t Contact::Solver::Solve( std::list<Contact>& liContacts )
{
	// Return the # of collisions
	uint32_t uNumCollisions( 0 );

	// Iterate and solve contacts
	for ( int nIt = 0; nIt < m_nIterations; nIt++ )
	{
		// The # of collisions this iteration
		uint32_t uColCount = 0;

		// Walk the contacts
		for ( Contact& c : liContacts )
		{
			// Coeffcicient of restitution, plus 1
			const float fCr_1 = 1.f + c.GetCoefRest();

			// Get the relative contact velocity - Vb - Va
			float vA_N = c.GetVelN_A();
			float vB_N = c.GetVelN_B();
			float relNv = vB_N - vA_N;

			// Find out how much you'd remove to have them just touch
			float remove = relNv + c.GetDistance() / g_fTimeStep;
			
			// Detect collision
			bool colliding = (remove < -kEPS);
			if ( colliding )
			{
				uColCount++;
				c.setIsColliding( true );
			}

			// Get the magnitude of the collision (negative or zero)
			float impulseMag = colliding ? (fCr_1 * relNv * c.GetInertialDenom()) : 0.f;

			// Calculate the new impulse and apply the change
			const float fCurImpulse = c.GetCurImpulse();
			float n_Impulse = impulseMag + fCurImpulse;
			float delImpulse = n_Impulse - fCurImpulse;
			c.ApplyImpulse( delImpulse );
		}

		// Maybe break if no contacts are colliding
		if ( uColCount == 0 )
			break;
		// Otherwise keep going
		else
			uNumCollisions += uColCount;
	}

	return uNumCollisions;
}

void Contact::InitDrawable( std::array<Drawable *, 2> drPtrArr ) const
{
	for ( Drawable * pDr : drPtrArr )
	{
		if ( pDr )
		{
			for ( glm::vec2 pos : m_v2Pos )
			{
				// NYI
			}
		}
	}
}