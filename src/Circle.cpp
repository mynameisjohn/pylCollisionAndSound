
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"

#include <glm/gtx/norm.hpp>

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D Circle::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.circData.fRadius = radius;
	ret.eType = RigidBody2D::EType::Circle;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

float Circle::Radius() const
{
	return circData.fRadius;
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( Circle * pA, Circle * pB )
{
	// find and normalize distance
	vec2 d = pB->v2Center - pA->v2Center;
	vec2 n = glm::normalize( d );

	// contact points along circumference
	vec2 a_pos = pA->v2Center + n * pA->Radius();
	vec2 b_pos = pB->v2Center - n * pB->Radius();

	// distance between circumferences
	float dist = glm::length( a_pos - b_pos );

	// Construct and return
	return{ Contact( pA, pB, a_pos, b_pos, n, dist ) };
}

////////////////////////////////////////////////////////////////////////////

// Simlar to the AABB case, but we only care about the center of the circle
std::list<Contact> GetSpecContacts( Circle * pCirc, AABB *  pAABB)
{
	// Determine which feature region we're on
	vec2 n;
	int vIdx( -1 );

	// top/bottom face region
	if ( (pAABB->Right() < pCirc->v2Center.x || pAABB->Left() > pCirc->v2Center.x) == false )
	{
		// Circle is below box
		if ( pCirc->v2Center.y < pAABB->Bottom() )
		{
			vIdx = 1;
			n = vec2( 0, 1 );
		}
		else
		{
			vIdx = 3;
			n = vec2( 0, -1 );
		}
	}
	// left/right face region
	else if ( (pAABB->Top() < pCirc->v2Center.y || pAABB->Bottom() > pCirc->v2Center.y) == false )
	{
		// Circle is to the left of the box
		if ( pCirc->v2Center.x < pAABB->Left() )
		{
			vIdx = 2;
			n = vec2( 1, 0 );
		}
		else
		{
			vIdx = 0;
			n = vec2( -1, 0 );
		}
	}
	// Vertex region
	else
	{
		// Determine which vertex
		bool bAIsLeft = (pCirc->v2Center.x < pAABB->Left());
		bool bAIsBelow = (pCirc->v2Center.y < pAABB->Bottom());
		if ( bAIsLeft )
		{
			if ( bAIsBelow )
				vIdx = 2;
			else
				vIdx = 3;
		}
		else
		{
			if ( bAIsBelow )
				vIdx = 1;
			else
				vIdx = 0;
		}

		// We don't need to average contact positions for the corner case
		vec2 posB = GetVert( pAABB, vIdx );
		n = glm::normalize( posB - pCirc->v2Center );
		vec2 posA = pCirc->v2Center + pCirc->circData.fRadius * n;
		float fDist = glm::distance( posA, posB );
		return{ Contact( pCirc, pAABB, posA, posB, n, fDist ) };
	}
	// For a face region collision, we want to make sure the contact knows it's
	// working with one of the box's face normals (meaning distance is along that normal)
	vec2 posB = 0.5f*(GetVert( pAABB, vIdx ) + GetVert( pAABB, vIdx + 1 ));
	//n = glm::normalize( posB - pCirc->v2Center );
	vec2 posA = pCirc->v2Center + pCirc->circData.fRadius * n;
	float fDist = glm::dot( posB - posA, n );
	return{ Contact( pCirc, pAABB, posA, posB, n, fDist ) };
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( Circle * pCirc, OBB * pOBB )
{
	vec2 b_pos = pOBB->WorldSpaceClamp( pCirc->v2Center );
	vec2 n = glm::normalize( b_pos - pCirc->v2Center );
	vec2 a_pos = pCirc->v2Center + n * pCirc->circData.fRadius;
	float fDist = glm::distance( a_pos, b_pos );
	
	// Construct and return
	return{ Contact( pCirc, pOBB, a_pos, b_pos, n, fDist ) };
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pA, Circle * pB )
{
	float dist = glm::length( pA->v2Center - pB->v2Center );
	float totalRadius = pA->Radius() + pB->Radius();

	return (dist < totalRadius);
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pCirc, AABB * pAABB )
{
	float r = pCirc->Radius();
	glm::vec2& C = pCirc->v2Center;
	bool bX = (pAABB->Left() > C.x + r) || (pAABB->Right() < C.y - r) == false;
	bool bY = (pAABB->Bottom() > C.y + r) || (pAABB->Top() < C.y - r);
	return bX && bY;
}

////////////////////////////////////////////////////////////////////////////

// Not tested, and there may be a better way...
bool IsOverlapping( Circle * pCirc, OBB * pOBB )
{
	for ( int i = 0; i < 4; i++ )
		if ( IsPointInside( GetVert( pOBB, i ), pCirc ) )
			return true;
	return IsPointInside( pCirc->v2Center, pOBB );
}

////////////////////////////////////////////////////////////////////////////

bool IsPointInside( vec2 p, Circle * pCirc )
{
	return glm::length2( pCirc->v2Center - p ) < powf( pCirc->circData.fRadius, 2 );
}