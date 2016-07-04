#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"

#include <glm/gtx/norm.hpp>

// Used to handle collisions between OBBs
struct FeaturePair
{
	// The type of anticipated collision 
	enum class EType
	{
		None = 0,
		FaceAVertexB,
		FaceBVertexA
	};

	float fDist2;	// Distance from vertex to face, squared
	float fCenDist2;// Distance vertex to face center, squared
	int ixVert;		// index of closest vert
	int ixFace;		// index of closest face
	EType eType;	// type of anticipated colllision

	FeaturePair() :
		fDist2( FLT_MAX ),
		fCenDist2( FLT_MAX ),
		ixVert( INT_MIN ),
		ixFace( INT_MIN ),
		eType( EType::None )
	{}
	FeaturePair( float fD2, float fCenD2, float iV, int iF, EType e ) :
		fDist2( fD2 ),
		fCenDist2( fCenD2 ),
		ixVert( iV ),
		ixFace( iF ),
		eType( e )
	{}
};

// A convenience struct that contains a vertex
// from an OBB and the vertex's index on that OBB
struct SupportVertex
{
	vec2 v;
	int idx;
};

// Given an OBB and a direction, return either 1 or 2
// vertices that are most in line with that direction
int GetSupportVerts( OBB * pOBB, vec2 N, std::array<SupportVertex, 2> * aSV )
{
	// Temporarily move the box back to the origin
	vec2 v2Center = pOBB->v2Center;
	pOBB->v2Center = vec2();

	int ixClosest( -1 ), ixSecondClosest( -1 );
	float fClosestDist( -FLT_MAX ), fSecondClosestDist( -FLT_MAX );

	// Find the index whose direction from the origin
	// most closely aligns with the normal
	for ( int i = 0; i < 4; i++ )
	{
		float fDist = glm::dot( N, GetVert( pOBB, i ) );
		if ( fDist > fClosestDist )
		{
			fClosestDist = fDist;
			ixClosest = i;
		}
	}

	// See if there's a very close runner up (happens quite often...)
	for ( int i = 0; i < 4; i++ )
	{
		if ( i == ixClosest )
			continue;

		float fDist = glm::dot( N, GetVert( pOBB, i ) );
		if ( feq( fDist, fClosestDist ) )
		{
			fSecondClosestDist = fDist;
			ixSecondClosest = i;
		}
	}

	// Reassign center
	pOBB->v2Center = v2Center;

	// Fill array, return count
	aSV->at( 0 ) = { GetVert( pOBB, ixClosest ), ixClosest };

	if ( ixSecondClosest != -1 )
	{
		aSV->at( 1 ) = { GetVert( pOBB, ixSecondClosest ), ixSecondClosest };
		return 2;
	}

	return 1;
}

// Given an OBB, and index on that OBB, and a normal, find the neighbor of the
// vertex at that index who helps form an edge most in line with the normal
std::array<glm::vec2, 2> GetSupportNeighbor( OBB * pOBB, vec2 n, int ixSupportVert )
{
	vec2 vA = GetVert( pOBB, ixSupportVert - 1 );
	vec2 vB = GetVert( pOBB, ixSupportVert );
	vec2 vC = GetVert( pOBB, ixSupportVert + 1 );

	vec2 nAB = glm::normalize( perp( vB - vA ) );
	vec2 nBC = glm::normalize( perp( vB - vA ) );

	if ( glm::dot( n, nAB ) < glm::dot( n, nBC ) )
		return{ vA, vB };
	return{ vB, vC };
}

// Algorithm used to find feature pairs in OBBs by treating one as the face object and one as the verex object
// This is the algorithm outlined by Paul Firth at http://www.wildbunny.co.uk/blog/2011/04/20/collision-detection-for-dummies/
// It's rather expensive and could possibly be done without walking so many vertices and faces, but for now I'd 
// prefer to be clear and explicit
void JudgeFeatures( OBB * pFace, OBB * pVertex, FeaturePair * pMostSep, FeaturePair * pMostPen, FeaturePair::EType e )
{
	// For every face in pFace
	for ( int i = 0; i < 4; i++ )
	{
		// Get the face normal and the edge of that face
		const vec2 wsN = GetNormal( pFace, i );
		const vec2 wsV0 = GetVert( pFace, i );
		const vec2 wsV1 = GetVert( pFace, i + 1 );

		// Get the support vertices along the negative of that normal (closest to the face)
		std::array<SupportVertex, 2> aSupportVerts;
		for ( int j = 0; j < GetSupportVerts( pVertex, -wsN, &aSupportVerts ); j++ )
		{
			// Form the minkowski face of {wsV0, wsV1} - {aSupportVerts[j].v}
			const vec2 mfp0 = aSupportVerts[j].v - wsV0;
			const vec2 mfp1 = aSupportVerts[j].v - wsV1;

			// The distance of the vertex from the normal is the projection of
			// the vector from the first edge vertex to the support vertex
			// along the direction of the face normal
			float fDist = glm::dot( wsN, mfp0 );

			// Compute the squared distance and squared center distance
			float fDist2 = fDist * fDist;
			float fCenterDist2 = glm::distance2( aSupportVerts[j].v, pFace->v2Center );

			// If the projection has a positive value, we are separated
			if ( fDist > 0 )
			{
				// The real distance between the vertex and face is the projection
				// of the origin onto the minkowski face (like GJK, I think)
				const vec2 projPoint = projectOnEdge( vec2(), mfp0, mfp1 );
				fDist2 = glm::length2( projPoint );

				// If this is a better candidate, reassign
				if ( fDist2 < pMostSep->fDist2 )
				{
					*pMostSep = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
				}
				// If they're very close
				else if ( feq( fDist2, pMostSep->fDist2 ) && pMostSep->eType == e )
				{
					// Pick the vertex closest to the face object's center
					if ( fCenterDist2 < pMostSep->fCenDist2 )
					{
						*pMostSep = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
					}
					// If theose two are very close as well...
					else if ( feq( fCenterDist2, pMostSep->fCenDist2 ) )
					{
						// Some wishful thinking - if we're this desparate, pick the feature pair
						// with a normal that is most in line with the distance between the two centers
						vec2 d = pVertex->v2Center - pFace->v2Center;
						vec2 oldN = GetNormal( pFace, pMostSep->ixFace );

						// If the distance vector more closely aligns with this face normal, reassign
						if ( glm::dot( wsN, d ) > glm::dot( oldN, d ) )
							*pMostSep = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
					}
				}
			}
			// If the projected value was negative, we are penetrating
			else if ( IsPointInside( aSupportVerts[j].v, pFace ) )
			{
				// Negate distance squared, it opposes the normal
				fDist2 = -fDist2;

				// We want the greatest penetration distance
				if ( fDist2 > pMostPen->fDist2 )
				{
					// reassign mostPenetrating
					*pMostPen = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
				}
				// A lot of this ambiguity case logic is copied from above
				else if ( feq( fDist2, pMostPen->fDist2 ) && pMostPen->eType == e )
				{
					if ( fCenterDist2 < pMostPen->fCenDist2 )
					{
						*pMostPen = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
					}
					else if ( feq( fCenterDist2, pMostSep->fCenDist2 ) )
					{
						vec2 d = pVertex->v2Center - pFace->v2Center;
						vec2 oldN = GetNormal( pFace, pMostSep->ixFace );

						if ( glm::dot( wsN, d ) > glm::dot( oldN, d ) )
							*pMostPen = FeaturePair( fDist2, fCenterDist2, aSupportVerts[j].idx, i, e );
					}
				}
			}
		}
	}

}

// Functions for getting speculative contacts
std::list<Contact> GetSpecContacts( OBB * pA, OBB * pB )
{
	// We're going to want the closest face-vertex feature pair
	FeaturePair fpMostSeparated, fpMostPenetrating;
	fpMostPenetrating.fDist2 = -FLT_MAX;
	FeaturePair::EType e;

	// Find the feature pair that is the best candidate for collision detection
	JudgeFeatures( pA, pB, &fpMostSeparated, &fpMostPenetrating, FeaturePair::EType::FaceAVertexB );
	JudgeFeatures( pB, pA, &fpMostSeparated, &fpMostPenetrating, FeaturePair::EType::FaceBVertexA );

	// Determine who is the face and who is the vertex
	FeaturePair * pFeaturePair = nullptr;
	OBB * pFace = nullptr, *pVertex = nullptr;
	if ( fpMostPenetrating.fDist2 <= 0 && fpMostPenetrating.eType != FeaturePair::EType::None )
	{
		pFeaturePair = &fpMostPenetrating;
	}
	else if ( fpMostSeparated.fDist2 > 0 && fpMostSeparated.eType != FeaturePair::EType::None )
	{
		pFeaturePair = &fpMostSeparated;
	}
	else
	{
		throw std::runtime_error( "Something went wrong during an OBB-OBB in GetSpecContacts(OBB *, OBB *)!" );
		return{};
	}

	// All this feels rather clunky...
	if ( pFeaturePair->eType == FeaturePair::EType::FaceAVertexB )
	{
		pFace = pA;
		pVertex = pB;
	}
	else //if ( pFeaturePair->eType == FeaturePair::EType::FaceBVertexA )
	{
		pVertex = pA;
		pFace = pB;
	}

	// Get the face normal and edge on pFace, in world space
	vec2 faceN = GetNormal( pFace, pFeaturePair->ixFace );
	vec2 faceEdge0 = GetVert( pFace, pFeaturePair->ixFace );
	vec2 faceEdge1 = GetVert( pFace, pFeaturePair->ixFace + 1 );

	// Get the support neighbor of the vertex object, given the face normal (forming an edge on pVertex)
	std::array<vec2, 2> aVertices = GetSupportNeighbor( pVertex, faceN, pFeaturePair->ixVert );

	// Project the two edges onto each other to form the contact positions
	vec2 ptFace0 = projectOnEdge( aVertices[1], faceEdge0, faceEdge1 );
	vec2 ptFace1 = projectOnEdge( aVertices[0], faceEdge0, faceEdge1 );
	vec2 ptVert0 = projectOnEdge( faceEdge0, aVertices[0], aVertices[1] );
	vec2 ptVert1 = projectOnEdge( faceEdge1, aVertices[0], aVertices[1] );

	// And their distances
	float fDist0 = glm::dot( faceN, ptVert0 - ptFace0 );
	float fDist1 = glm::dot( faceN, ptVert1 - ptFace1 );

	// If they're very close, collapse into one contact in the middle of the edge
	if ( feq( glm::distance2( ptVert0, ptFace0 ), glm::distance2( ptVert1, ptFace1 ) ) )
	{
		vec2 ptFace = 0.5f * (ptFace0 + ptFace1);
		vec2 ptVert = 0.5f * (ptVert0 + ptVert1);
		return{ Contact( pFace, pVertex, ptFace, ptVert, faceN, glm::dot( faceN, ptVert - ptFace ) ) };
	}
	
	// Otherwise return both
	return{ 
		Contact( pFace, pVertex, ptFace0, ptVert0, faceN, fDist0 ),
		Contact( pFace, pVertex, ptFace1, ptVert1, faceN, fDist1 ) 
	};
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 OBB::WorldSpaceClamp( const glm::vec2 p ) const
{
	// Invoke base if needed
	if ( eType == EType::AABB )
		return Clamp( p );

	// u and v are x and y in the OBB's local space
	vec2 u( cos( fTheta ), sin( fTheta ) );
	vec2 v = perp( u );

	// Transform the vector from the center into OBB local space, clamp to half dim
	vec2 d = p - v2Center;
	vec2 localPoint( glm::dot( d, u ), glm::dot( d, v ) );
	localPoint = glm::clamp( localPoint, -boxData.v2HalfDim, boxData.v2HalfDim );

	// Transform back into world space
	return v2Center + localPoint.x * u + localPoint.y * v;
}

/*static*/ RigidBody2D OBB::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R, float th /*= 0.f*/ )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.boxData.v2HalfDim = v2R / 2.f;
	ret.eType = RigidBody2D::EType::OBB;
	ret.fTheta = th;
	return ret;
}

/*static*/ RigidBody2D OBB::Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h, float th /*= 0.f*/ )
{
	RigidBody2D ret = RigidBody2D::Create( vel, vec2( x, y ), mass, elasticity );
	ret.boxData.v2HalfDim = vec2( w, h ) / 2.f;
	ret.eType = RigidBody2D::EType::OBB;
	return ret;
}


bool IsPointInside( vec2 p, OBB * pOBB )
{
	if ( pOBB->eType == RigidBody2D::EType::AABB )
		return IsPointInside( p, (AABB *) pOBB );

	glm::vec2 d = p - pOBB->v2Center;
	glm::vec2 xHat = GetNormal( pOBB, 0 );
	glm::vec2 yHat = perp( xHat );
	bool bX = fabs( glm::dot( d, xHat ) ) < pOBB->boxData.v2HalfDim.x;
	bool bY = fabs( glm::dot( d, yHat ) ) < pOBB->boxData.v2HalfDim.y;
	return bX && bY;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( OBB * pA, OBB * pB )
{
	// There may be a more efficient way of doing this
	for ( int i = 0; i < 4; i++ )
	{
		vec2 p = GetVert( pA, i );
		if ( IsPointInside( p, pB ) )
			return true;
	}
	// And vice versa
	for ( int i = 0; i < 4; i++ )
	{
		vec2 p = GetVert( pB, i );
		if ( IsPointInside( p, pA ) )
			return true;
	}
	// Otherwise return false
	return false;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( OBB * pOBB, int idx )
{
	if ( pOBB->eType == RigidBody2D::EType::AABB )
		return GetVert( (AABB *) pOBB, idx );

	vec2 v2Rad = pOBB->HalfDim();
	glm::mat2 M = pOBB->GetRotMat();

	switch ( (idx + 4) % 4 )
	{
		case 0:
			return pOBB->v2Center + M *  v2Rad;
		case 1:
			return pOBB->v2Center + M * vec2( v2Rad.x, -v2Rad.y );
		case 2:
			return pOBB->v2Center - M * v2Rad;
		case 3:
			return pOBB->v2Center + M * vec2( -v2Rad.x, v2Rad.y );
	}
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetNormal( OBB * pOBB, int idx )
{
	if ( pOBB->eType == RigidBody2D::EType::AABB )
		return GetNormal( (AABB *) pOBB, idx );

	float c = cosf( pOBB->fTheta );
	float s = sinf( pOBB->fTheta );

	switch ( idx % 4 )
	{
		case 0:
			return glm::vec2( c, s );
		case 1:
			return glm::vec2( s, -c );
		case 2:
			return glm::vec2( -c, -s );
		case 3:
			return glm::vec2( -s, c );
	}
}

////////////////////////////////////////////////////////////////////////////

int GetSupportVerts( OBB * pOBB, glm::vec2 n, std::array<glm::vec2, 2>& sV )
{
	int foundIdx( -1 );

	// Find the furthest vertex
	float dMin = -FLT_MAX;
	for ( int i = 0; i < 4; i++ )
	{
		vec2 v = GetVert( pOBB, i );
		float d = glm::dot( n, v );
		if ( d > dMin )
		{
			dMin = d;
			sV[0] = v;
			foundIdx = i;
		}
	}

	int num( 1 );

	// If there's a different vertex
	for ( int i = 0; i < 4; i++ )
	{
		if ( i == foundIdx )
			continue;
		vec2 v = GetVert( pOBB, i );
		float d = glm::dot( n, v );
		// That's pretty close...
		if ( feq( d, dMin, 100.f * kEPS ) )
		{
			// Take it too
			dMin = d;
			sV[num++] = v;
		}
	}

	return num;
}

////////////////////////////////////////////////////////////////////////////

int GetSupportIndices( OBB * pOBB, glm::vec2 n, std::array<int, 2>& sV )
{
	// Find the furthest vertex
	float dMin = -FLT_MAX;
	for ( int i = 0; i < 4; i++ )
	{
		vec2 v = GetVert( pOBB, i );
		float d = glm::dot( n, v );
		if ( d > dMin )
		{
			dMin = d;
			sV[0] = i;
		}
	}

	int num( 1 );

	// If there's a different vertex
	for ( int i = 0; i < 4; i++ )
	{
		if ( i == sV[0] )
			continue;
		vec2 v = GetVert( pOBB, i );
		float d = glm::dot( n, v );
		// That's pretty close...
		if ( feq( d, dMin, 100.f * kEPS ) )
		{
			// Take it too
			dMin = d;
			sV[num++] = i;
		}
	}

	return num;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetSupportNormal( OBB * pOBB, glm::vec2 N )
{
	uint32_t iMin( 0 );
	float dMin( -FLT_MAX );

	for ( uint32_t i = 0; i<4; i++ )
	{
		float d = glm::dot( N, GetNormal( pOBB, i ) );
		if ( d > dMin )
		{
			dMin = d;
			iMin = i;
		}
	}

	return GetNormal( pOBB, iMin );
}