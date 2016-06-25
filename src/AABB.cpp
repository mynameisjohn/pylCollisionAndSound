#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"
#include <glm/gtx/norm.hpp>

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

// This is rather verbose, but it gets the job done
std::list<Contact> GetSpecContacts( AABB * pA, AABB * pB )
{
	// Contact normal and indices from each box
	vec2 n;
	int vIdxA, vIdxB;
	// Vertical collision
	if ( IsOverlappingX( pA, pB ) )
	{
		// If We're below
		if ( pA->Top() < pB->Bottom() )
		{
			// top of A, bottpm of B
			n = vec2( 0, 1 );
			vIdxA = 3;
			vIdxB = 1;
		}
		else
		{
			// bottom of A, top of B
			n = vec2( 0, -1 );
			vIdxA = 1;
			vIdxB = 3;
		}
	}
	// Horizontal collision
	else if ( IsOverlappingY( pA, pB ) )
	{
		if ( pA->Right() < pB->Left() )
		{
			// right of A, left of B
			n = vec2( 1, 0 );
			vIdxA = 0;
			vIdxB = 2;
		}
		else
		{
			// left of A, right of B
			n = vec2( -1, 0 );
			vIdxA = 2;
			vIdxB = 0;
		}
	}
	// Corner collision
	else
	{
		// If we aren't overlapping in either direction
		// handle a potential corner collision
		bool bAIsLeft = (pA->Right() < pB->Left());
		bool bAIsBelow = (pA->Top() < pB->Bottom());
		if ( bAIsLeft )
		{
			if ( bAIsBelow )
			{
				// Top right of A, bottom left of B
				vIdxA = 0;
				vIdxB = 2;
			}
			else
			{
				// Bottom right of A, top left of B
				vIdxA = 1;
				vIdxB = 3;
			}
		}
		else
		{
			if ( bAIsBelow )
			{
				// Top left of A, bottom right of B
				vIdxA = 3;
				vIdxB = 1;
			}
			else
			{
				// Bottom left of A, top right of B
				vIdxA = 2;
				vIdxB = 0;
			}
		}

		// We don't need to average contact positions for the corner case
		vec2 posA = GetVert( pA, vIdxA );
		vec2 posB = GetVert( pB, vIdxB );
		n = glm::normalize( posB - posA );
		float fDist = glm::distance( posA, posB );
		return{ Contact( pA, pB, posA, posB, n, fDist ) };
	}

	// For the face case, we get the two vertices from each colliding face
	// and average them per face, so that the collision is not diminished
	// by the radius arm of the contact. Distance is along direction of normal
	vec2 posA = 0.5f * (GetVert( pA, vIdxA ) + GetVert( pA, vIdxA + 1 ));
	vec2 posB = 0.5f * (GetVert( pB, vIdxB ) + GetVert( pB, vIdxB + 1 ));
	float fDist = glm::dot( n, posB - posA );
	return{ Contact( pA, pB, posA, posB, n, fDist ) };
}

////////////////////////////////////////////////////////////////////////////

std::list<Contact> GetSpecContacts( AABB * pAABB, OBB * pOBB )
{
	// I wonder if, as an early out, I can treat an OBB with
	// an angle such that cos(fTheta) or sin(fTheta) is near 0
	// as an AABB, since it's practically on axis
	// cos(fTheta) = 0 case, though, i'd have to flip the dims
	//if ( feq( sinf( pOBB->fTheta ), 0 ) )
	//{
	//	GetSpecContacts( pAABB, (AABB *) pOBB );
	//}
	//else if ( feq( cosf( pOBB->fTheta ), 0 ) )
	//{
	//	// Temporarily flip the half-dims
	//	std::swap( pOBB->boxData.v2HalfDim.x, pOBB->boxData.v2HalfDim.y );
	//	auto ret = GetSpecContacts( pAABB, (AABB *) pOBB );
	//	std::swap( pOBB->boxData.v2HalfDim.x, pOBB->boxData.v2HalfDim.y );
	//	ret;
	//}
	
	// Find the best face-vertex pair between the two
	FaceVertexPair fp( (OBB *) pAABB, pOBB );

	// Project the two vertices of the face feature onto
	// the edge formed by two vertices of the vertex feature

	// Get the contact normal
	vec2 cN = GetNormal( fp.pBestFace, fp.ixBestFace );

	// The face edge
	vec2 v2F_e0 = GetVert( fp.pBestFace, fp.ixBestFace );
	vec2 v2F_e1 = GetVert( fp.pBestFace, fp.ixBestFace +1 );

	// The vertex edge (reversed because we want this CCW)
	vec2 v2V_e0 = GetVert( fp.pBestVert, fp.ixBestVert);
	vec2 v2V_e1 = GetVert( fp.pBestVert, fp.ixBestVert +1);

	// The vertex contact positions are the projection of
	// the two face vertices along the edge formed by V_p0,p1
	vec2 v2V_p1 = projectOnEdge( v2F_e0, v2V_e0, v2V_e1 );
	vec2 v2V_p0 = projectOnEdge( v2F_e1, v2V_e0, v2V_e1 );

	// We have to special case the two face contact positions
	vec2 v2F_p0, v2F_p1;
	if ( fp.pBestFace == pAABB )
	{
		// If the AABB is the face feature object,
		// the two contact points are the midpoint
		// of the face (so the collision doesn't lose
		// any energy to a rotation that can't occur)
		v2F_p0 = v2F_p1 = 0.5f * (v2F_e0 + v2F_e1);
	}
	else
	{
		// If the AABB is the vertex feature object,
		// The face contact positions are the projections
		// of the two vertex edge points on the face edge
		v2F_p0 = projectOnEdge( v2V_e0, v2F_e0, v2F_e1 );
		v2F_p1 = projectOnEdge( v2V_e1, v2F_e0, v2F_e1 );
	}

	// The contact distances are the projections of the
	// distance between contact points along face normal
	float fDist0 = glm::dot( cN, v2V_p0 - v2F_p0 );
	float fDist1 = glm::dot( cN, v2V_p1 - v2F_p1 );

	if ( glm::distance2( v2V_p0, v2V_p1 ) < kEPS )
		return{ Contact( fp.pBestFace, fp.pBestVert, v2F_p0,  v2V_p0, cN, fDist0 ) };

	//std::cout << glm::vec2( fDist0, fDist1 ) << std::endl;

	// Construct the contact and get out
	return{
		Contact( fp.pBestFace, fp.pBestVert, v2F_p0,  v2V_p0, cN, fDist0 ),
		Contact( fp.pBestFace, fp.pBestVert, v2F_p1,  v2V_p1, cN, fDist1 ),
	};
}

////////////////////////////////////////////////////////////////////////////

bool IsPointInside( vec2 p, AABB * pAABB )
{
	bool bX = fabs( p.x - pAABB->v2Center.x ) < pAABB->boxData.v2HalfDim.x;
	bool bY = fabs( p.y - pAABB->v2Center.y ) < pAABB->boxData.v2HalfDim.y;
	return bX && bY;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlappingX( AABB* pA, AABB* pB )
{
	return (pA->Right() < pB->Left() || pA->Left() > pB->Right()) == false;
}

bool IsOverlappingY( AABB* pA, AABB* pB )
{
	return (pA->Top() < pB->Bottom() || pA->Bottom() > pB->Top()) == false;
}

bool IsOverlapping( AABB* pA, AABB* pB )
{
	return IsOverlappingX( pA, pB ) && IsOverlappingY( pA, pB );
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( AABB * pAABB, OBB * pOBB )
{
	// if any OBB verts are in AABB, return true
	for ( int i = 0; i < 4; i++ )
	{
		vec2 p = GetVert( pOBB, i );
		if ( IsPointInside( p, pAABB ) )
			return true;
	}
	// And vice versa
	for ( int i = 0; i < 4; i++ )
	{
		vec2 p = GetVert( pAABB, i );
		if ( IsPointInside( p, pOBB ) )
			return true;
	}
	// Otherwise return false
	return false;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( AABB * pAABB, int idx )
{
	vec2 ret( 0 );
	vec2 R = pAABB->boxData.v2HalfDim;	// 3---0
	switch ( (idx + 4) % 4 )			// |   |
	{									// 2---1
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

glm::vec2 GetNormal( AABB * pAABB, int idx )
{										// --0--
	switch ( (idx + 4) % 4 )			// 3   1
	{									// --2--
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
