
#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"

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
	vec2 d = pA->v2Center - pB->v2Center;
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

std::list<Contact> GetSpecContacts( Circle * pCirc, AABB *  pAABB)
{
	vec2 b_pos = pAABB->Clamp( pCirc->v2Center );
	vec2 n = glm::normalize( b_pos - pCirc->v2Center );

	vec2 a_pos = pCirc->v2Center + pCirc->Radius() * n;
	float dist = glm::length( a_pos - b_pos );

	// Construct and return
	return{ Contact( pCirc, pAABB, a_pos, b_pos, n, dist ) };
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( Circle * pCirc, OBB * pOBB )
{
	vec2 a_pos = pOBB->WorldSpaceClamp( pCirc->v2Center );
	vec2 n = -glm::normalize( a_pos - pCirc->v2Center );
	vec2 b_pos = pCirc->v2Center - n*pCirc->circData.fRadius;
	float dist = glm::length( a_pos - b_pos );

	// Construct and return
	return{ Contact( pCirc, pOBB, a_pos, b_pos, n, dist ) };
}

////////////////////////////////////////////////////////////////////////////

// Between Circle and OBB (see OBB impl)
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
	bool x = (pAABB->Left() > C.x + r) || (pAABB->Right() > C.y - r);
	bool y = (pAABB->Bottom() > C.x + r) || (pAABB->Top() > C.y - r);
	return x && y;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( Circle * pCirc, OBB * pOBB )
{
	// NYI
	return false;
}