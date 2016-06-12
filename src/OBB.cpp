#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"

// Static function to project a point p along the edge
// between points e0 and e1
static vec2 projectOnEdge( vec2 p, vec2 e0, vec2 e1 )
{
	// u and v form a little triangle
	vec2 u = p - e0;
	vec2 v = e1 - e0;

	// find the projection of u onto v, parameterize it
	float t = glm::dot( u, v ) / glm::dot( v, v );

	//std::cout << t << ", " << glm::dot(u, v) << ", " << glm::dot(v,v) << std::endl;

	// Clamp between two edge points
	return e0 + v * clamp( t, 0.f, 1.f );
}

// Pick the best feature pair (penetrating or separating); A is the "face" object, B is the "vertex" object
// Penetrating is a bit of a grey area atm
static void featurePairJudgement( FeaturePair& mS, FeaturePair& mP, OBB * A, OBB * B, FeaturePair::EType type )
{
	// For all A's normals
	for ( int fIdx = 0; fIdx < 4; fIdx++ )
	{
		vec2 n = GetNormal( A, fIdx );
		vec2 p1 = GetVert( A, fIdx );
		vec2 p2 = GetVert( A, (fIdx + 1) % 4 );

		// For B's support verts relative to the normal
		std::array<int, 2> supportVerts = { { -1, -1 } };
		int nVerts = GetSupportIndices( B, -n, supportVerts );
		for ( int s = 0; s < nVerts; s++ )
		{
			int sIdx = supportVerts[s];
			vec2 sV = GetVert( B, sIdx );

			// minkowski face points
			vec2 mfp0 = sV - p1;
			vec2 mfp1 = sV - p2;

			// Find point on minkowski face
			vec2 p = projectOnEdge( vec2(), mfp0, mfp1 );

			// are objects penetrating?
			// i.e is first mf point behind face normal
			// penetration implies support vert behind normal
			float dist = glm::dot( mfp0, n );
			float c_dist = glm::length( sV - A->v2Center );
			bool isPenetrating = dist < 0.f;

			// fp points to the correct feature pair
			FeaturePair * fp( nullptr );
			if ( isPenetrating )
				fp = &mP;
			else
			{
				fp = &mS;
				// Separation dist is the length from p to the origin
				dist = glm::length( p );
			}

			// See whether or not this new feature pair is a good candidate
			// For penetration, we want the largest negative value
			bool accept = false;
			if ( isPenetrating ? (dist > fp->dist) : (dist < fp->dist) )
				accept = true;
			else if ( feq( dist, fp->dist ) )
			{
				// If it's just about as close as the current best feature pair,
				// pick the one whose distance is closest to the center of the face object
				if ( c_dist < fp->c_dist )
					accept = true;
			}

			// Reassign *fp as needed
			if ( accept )
				*fp = FeaturePair( dist, c_dist, fIdx, sIdx, type );
		}
	}
}

// Functions for getting speculative contacts
std::list<Contact> GetSpecContacts( OBB * pA, OBB * pB )
{
	std::list<Contact> ret;

	// TODO are penetrating features working?
	FeaturePair mostSeparated( FLT_MAX );
	FeaturePair mostPenetrating( -FLT_MAX );

	// Check our faces with their verts, then vice versa
	featurePairJudgement( mostSeparated, mostPenetrating, pA, pB, FeaturePair::EType::F_V );
	featurePairJudgement( mostSeparated, mostPenetrating, pA, pB, FeaturePair::EType::V_F );

	// Pick the correct feature pair
	bool sep = (mostSeparated.dist > 0 && mostSeparated.T != FeaturePair::EType::N);
	FeaturePair * fp = sep ? &mostSeparated : &mostPenetrating;

	// We better have something
	assert( fp->T != FeaturePair::EType::N );

	// A is face feature, B is vertex feature
	OBB * pFace( nullptr ), * pVertex( nullptr );
	if ( fp->T == FeaturePair::EType::F_V )
	{
		pFace = pA;
		pVertex = pB;
	}
	else
	{
		pVertex = pA;
		pFace = pB;
	}

	// Get the world space normal and edge points
	// of the face feature
	vec2 wN = GetNormal( pFace, fp->fIdx );
	vec2 wE0 = GetVert( pFace, fp->fIdx );
	vec2 wE1 = GetVert( pFace, fp->fIdx + 1 );

	// Get the world space vertex of the vertex feature,
	// and then get "supporting neighbor" of that vertex
	// along the direction of the face feature edge, clockwise
	std::array<vec2, 2> wV = GetSupportNeighbor( pVertex, -wN, fp->vIdx );

	//std::cout << wN << "\n" << wE0 << "\n" << wE1 << "\n" << wV0 << "\n" << wV1 << "\n" << std::endl;

	// Project edge points along vertex feature edge
	vec2 p1 = projectOnEdge( wE0, wV[0], wV[1] );
	vec2 p2 = projectOnEdge( wE1, wV[0], wV[1] );

	// Project vertex points along face feature edge
	vec2 p3 = projectOnEdge( wV[0], wE0, wE1 );
	vec2 p4 = projectOnEdge( wV[1], wE0, wE1 );

	// distance is point distance along face (contact) normal
	float d1 = glm::dot( p1 - p4, wN );
	float d2 = glm::dot( p2 - p3, wN );

	// If they're equal, collapse the two into one contact
	// This could be used as an early out, if you have the balls
	if ( feq( d1, d2 ) )
	{
		vec2 pA = 0.5f * (p1 + p2);
		vec2 pB = 0.5f * (p3 + p4);
		float d = glm::distance( pA, pB );
		ret.emplace_back( pFace, pVertex, pA, pB, wN, d );
	}
	else
	{   // Otherwise add two contacts points
		ret.emplace_back( pFace, pVertex, p1, p4, wN, d1 );
		ret.emplace_back( pFace, pVertex, p2, p3, wN, d2 );
	}

	return ret;
}

////////////////////////////////////////////////////////////////////////////

bool IsOverlapping( OBB * pA, OBB * pB )
{
	// NYI
	return false;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( OBB * pOBB, int idx )
{
	vec2 ret( 0 ), R = pOBB->HalfDim();
	switch ( idx % 4 )
	{
		case 0:
			return pOBB->v2Center + pOBB->GetRotMat() *  R;
		case 1:
			return pOBB->v2Center + pOBB->GetRotMat() * vec2( R.x, -R.y );
		case 2:
			return pOBB->v2Center - pOBB->GetRotMat() * R;
		case 3:
		default:
			return pOBB->v2Center + pOBB->GetRotMat() * vec2( -R.x, R.y );
	}
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetNormal( OBB * pOBB, int idx )
{
	switch ( idx % 4 )
	{
		case 0:
			return pOBB->GetRotMat() * vec2( 1, 0 );
		case 1:
			return pOBB->GetRotMat() * vec2( 0, -1 );
		case 2:
			return pOBB->GetRotMat() * vec2( -1, 0 );
		case 3:
		default:
			return pOBB->GetRotMat() * vec2( 0, 1 );
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

std::array<glm::vec2, 2> GetSupportNeighbor( OBB * pOBB, glm::vec2 n, int idx )
{
	std::array<vec2, 2> ret;

	vec2 vb = GetVert( pOBB, idx );
	vec2 va = GetVert( pOBB, idx - 1 );
	vec2 vc = GetVert( pOBB, idx + 1 );

	vec2 nab = glm::normalize( perp( vb - va ) );
	vec2 nbc = glm::normalize( perp( vc - vb ) );
	float d1 = glm::dot( nab, n );
	float d2 = glm::dot( nbc, n );

	if ( d1 > d2 )
		return{ { va, vb } };
	return{ { vb, vc } };
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