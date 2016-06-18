#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"


////////////////////////////////////////////////////////////////////////////

float AABB::Width() const
{
	return 2.f * boxData.v2HalfDim.x;
}

float AABB::Height() const
{
	return 2.f * boxData.v2HalfDim.y;
}

float AABB::Left() const
{
	return v2Center.x - boxData.v2HalfDim.x;
}

float AABB::Right() const
{
	return v2Center.x + boxData.v2HalfDim.x;
}

float AABB::Top() const
{
	return v2Center.y + boxData.v2HalfDim.y;
}

float AABB::Bottom() const
{
	return v2Center.y - boxData.v2HalfDim.y;
}

glm::vec2 AABB::HalfDim() const
{
	return boxData.v2HalfDim;
}

glm::vec2 AABB::Clamp( const glm::vec2 p ) const
{
	return glm::clamp( p, v2Center - boxData.v2HalfDim, v2Center + boxData.v2HalfDim );
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 AABB::GetFaceNormalFromPoint( const glm::vec2 p ) const
{
	vec2 n( 0 );

	if ( p.x < Right() && p.x > Left() )
	{
		if ( p.y < Bottom() )
			n = vec2( 0, -1 );
		else
			n = vec2( 0, 1 );
	}
	else
	{
		if ( p.x < Left() )
			n = vec2( -1, 0 );
		else
			n = vec2( 1, 0 );
	}

	return n;
}

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.boxData.v2HalfDim = v2R;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h )
{
	RigidBody2D ret = RigidBody2D::Create( vel, vec2( x, y ), mass, elasticity );
	ret.boxData.v2HalfDim = vec2( w, h ) / 2.f;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( AABB * pA, AABB * pB )
{
	// Find the distance from A's center to B's
	vec2 d = pB->v2Center - pA->v2Center;

	// Clamp that vector to the box bounds, negating for b
	vec2 a_pos = pA->Clamp( d );
	vec2 b_pos = pB->Clamp( -d );

	// Find the distance between the two contact points
	float dist = glm::length( a_pos - b_pos );

	// Construct and return
	return{ Contact( pA, pB, a_pos, b_pos, glm::normalize( d ), dist ) };
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( AABB * pAABB, OBB * pOBB )
{
	std::list<Contact> ret;

	// Find the vector from our center to theirs
	vec2 centerVecN = pAABB->v2Center - pOBB->v2Center;
	vec2 faceN( 0 );

	// Make it a unit vector in the direction of largest magnitude
	faceN = glm::normalize( maxComp( centerVecN ) );

	// Get supporting vertex / vertices along that direction
	std::array<int, 2> sV;
	int nSupportVerts = GetSupportIndices( pOBB, faceN, sV );

	// one support vert means a vertex-face collision
	if ( nSupportVerts == 1 )
	{
		// Two contact points; the support vertex, and it's best neighbor
		std::array<glm::vec2, 2> pA_arr = GetSupportNeighbor( pOBB, faceN, sV[0] );
		for ( auto& pA : pA_arr )
		{
			vec2 pB = pAABB->Clamp( pA );
			float d = glm::distance( pA, pB );
			ret.emplace_back( pAABB, pOBB, pA, pB, faceN, d );
		}
	}
	// face-face collsion, average two support verts into one contact point
	else
	{
		vec2 pA = 0.5f * (GetVert( pOBB, sV[0] ) + GetVert( pOBB, sV[1] ));
		vec2 pB = pAABB->Clamp( pA );
		float d = glm::distance( pA, pB );
		ret.emplace_back( pAABB, pOBB, pA, pB, faceN, d );
	}

	return ret;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( AABB* pA, AABB* pB )
{
	return
		(pA->Left() > pB->Right() || pA->Right() < pB->Left()) &&
		(pA->Bottom() > pB->Top() || pA->Top() < pB->Bottom());
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( AABB * pAABB, OBB * pOBB )
{
	// NYI
	return false;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( AABB * pAABB, int idx )
{
	vec2 ret( 0 );
	vec2 R = pAABB->v2Center;
	switch ( idx % 4 )
	{
		case 0:
			return pAABB->v2Center + R;
		case 1:
			return pAABB->v2Center + vec2( R.x, -R.y );
		case 2:
			return pAABB->v2Center - R;
		case 3:
		default:
			return pAABB->v2Center + vec2( -R.x, R.y );
	}
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetNormal( AABB * pAABB, int uIdx )
{
	switch ( uIdx % 4 )
	{
		case 0:
			return vec2( 1, 0 );
		case 1:
			return vec2( 0, -1 );
		case 2:
			return vec2( -1, 0 );
		case 3:
		default:
			return vec2( 0, 1 );
	}
}