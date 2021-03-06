#include "Contact.h"
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include "Drawable.h"
#include <glm/vec4.hpp>
#include <glm/gtc/random.hpp>

#include <iostream>

Contact::Contact():
	m_bIsColliding( false ),
	m_pCollidingPair{ nullptr, nullptr },
	m_v2Pos{ vec2(), vec2() },
	m_v2Radius{ vec2(), vec2() },
	m_v2Normal( vec2() ),
	m_fDist( 0 ),
	m_fCurImpulse( 0 ),
	m_fInvMassI(0)
{}

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
		// Skip anything with a negative mass
		if ( m_pCollidingPair[i]->fMass < 0 )
			continue;

		// The radius arm is the vector from the object's center to the contact
		m_v2Radius[i] = perp( m_v2Pos[i] - m_pCollidingPair[i]->v2Center );

		// The inverse mass denominator is a coeffecient used to calculate impulses
		// and depends on the object's inertia intertia, and it has a translation/rotation component
		fDenom += 1.f / m_pCollidingPair[i]->fMass;

		// Right now OBB is the only primitive that rotates
		//if ( m_pCollidingPair[i]->eType == RigidBody2D::EType::OBB )
		{
			float rN = glm::dot( m_v2Radius[i], m_v2Normal );
			fDenom += powf( rN, 2 ) / m_pCollidingPair[i]->GetInertia();
		}
	}
	
	// Is this necessary?
	if ( fDenom < kEPS )
		throw std::runtime_error( "Error: Invalid inertial denominator calculated!" );

	// Set the inverse mass denom
	m_fInvMassI = 1.f / fDenom;
}

void Contact::ApplyImpulse( float fMag )
{
	// Calculate the new impulse and apply the change
	float newImpulse = std::min( 0.f, fMag + m_fCurImpulse );
	float delImpulse = newImpulse - m_fCurImpulse;

	// Find the direction along our collision normal
	vec2 v2Impulse = delImpulse * m_v2Normal;

	float k0 = m_pA->GetKineticEnergy() + m_pB->GetKineticEnergy();

	for ( int i = 0; i < 2; i++ )
	{
		// Skip anything with a negative mass
		if ( m_pCollidingPair[i]->fMass < 0 )
			continue;

		const float sgn = i == 0 ? 1.f : -1.f;
		m_pCollidingPair[i]->v2Vel += sgn * v2Impulse / m_pCollidingPair[i]->fMass;
		m_pCollidingPair[i]->fOmega += sgn * glm::dot(v2Impulse, m_v2Radius[i]) / m_pCollidingPair[i]->GetInertia();
	}

	float k1 = m_pA->GetKineticEnergy() + m_pB->GetKineticEnergy();

	// Add impulse to local var
	m_fCurImpulse = newImpulse;
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

float Contact::GetAvgCoefRest() const
{
	return .5f * (m_pA->fElast + m_pB->fElast);
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
			const float fCr_1 = 1.f + c.GetAvgCoefRest();

			// Get the relative velocity of each body along the contact normal
			const float vA_N = c.GetVelN_A();
			const float vB_N = c.GetVelN_B();

			// Find the relative velocity of the system
			float fRelVN = vB_N - vA_N;

			// Determine how much velocity we'd need to remove such that
			// in the next iteration the two objects will be touching
			float fVelNeeded = c.GetDistance() * g_fInvTimeStep;
			float fVelToRemove = fRelVN + fVelNeeded;

			// If this is very low
			if ( fVelToRemove < kEPS )
			{
				// apply a collison along the normal
				float impulseMag = fCr_1 * fRelVN * c.GetInertialDenom();
				if ( c.GetDistance() < 0 && fRelVN > 0 && c.GetBodyA()->fMass > 0 && c.GetBodyB()->fMass > 0 )
					impulseMag = -(.05f * c.GetInertialDenom());
				c.ApplyImpulse( impulseMag );

				// Increase collision counter, flag contact as colliding
				uColCount++;
				c.setIsColliding( true );
			}
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
	const float drScale = 0.2f;
	vec4 v4Color = glm::linearRand( vec4( vec3( 0.3 ), 1 ), vec4( 1 ) );
	for ( int i = 0; i < 2; i++ )
	{
		if ( drPtrArr[i] )
		{
			quatvec qv; 
			qv.vec = vec3( m_v2Pos[i], 1.f );
			drPtrArr[i]->Init( "../models/quad.iqm", v4Color, qv, vec2( drScale ) );
		}
	}
}