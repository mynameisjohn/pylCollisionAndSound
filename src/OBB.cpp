#include "RigidBody2D.h"
#include "CollisionFunctions.h"
#include "GL_Util.h"
#include "Util.h"


//
//// Pick the best feature pair (penetrating or separating); A is the "face" object, B is the "vertex" object
//// Penetrating is a bit of a grey area atm
//static void featurePairJudgement( FeaturePair& mS, FeaturePair& mP, AABB * A, OBB * B, FeaturePair::EType type )
//{
//	// For all A's normals
//	for ( int fIdx = 0; fIdx < 4; fIdx++ )
//	{
//		vec2 n = GetNormal( A, fIdx );
//		vec2 p1 = GetVert( A, fIdx );
//		vec2 p2 = GetVert( A, (fIdx + 1) % 4 );
//
//		// For B's support verts relative to the normal
//		std::array<int, 2> supportVerts = { { -1, -1 } };
//		int nVerts = GetSupportIndices( B, -n, supportVerts );
//		for ( int s = 0; s < nVerts; s++ )
//		{
//			int sIdx = supportVerts[s];
//			vec2 sV = GetVert( B, sIdx );
//
//			// minkowski face points
//			vec2 mfp0 = sV - p1;
//			vec2 mfp1 = sV - p2;
//
//			// Find point on minkowski face
//			vec2 p = projectOnEdge( vec2(), mfp0, mfp1 );
//
//			// are objects penetrating?
//			// i.e is first mf point behind face normal
//			// penetration implies support vert behind normal
//			float dist = glm::dot( mfp0, n );
//			float c_dist = glm::length( sV - A->v2Center );
//			bool isPenetrating = dist < 0.f;
//
//			// fp points to the correct feature pair
//			FeaturePair * fp( nullptr );
//			if ( isPenetrating )
//				fp = &mP;
//			else
//			{
//				fp = &mS;
//				// Separation dist is the length from p to the origin
//				dist = glm::length( p );
//			}
//
//			// See whether or not this new feature pair is a good candidate
//			// For penetration, we want the largest negative value
//			bool accept = false;
//			if ( isPenetrating ? (dist > fp->dist) : (dist < fp->dist) )
//				accept = true;
//			else if ( feq( dist, fp->dist ) )
//			{
//				// If it's just about as close as the current best feature pair,
//				// pick the one whose distance is closest to the center of the face object
//				if ( c_dist < fp->c_dist )
//					accept = true;
//			}
//
//			// Reassign *fp as needed
//			if ( accept )
//				*fp = FeaturePair( dist, c_dist, fIdx, sIdx, type );
//		}
//	}
//}

// Functions for getting speculative contacts
std::list<Contact> GetSpecContacts( OBB * pA, OBB * pB )
{
	// I wonder if, as an early out, I can determine if the
	// angles are very simlar and if so treat these as
	// two AABB's at origin and then transform the contacts
	//if ( feq( fabs( glm::dot( GetNormal( pA, 0 ), GetNormal( pB, 0 ) ) ), 1.f ) )
	//{
	//	auto ret = GetSpecContacts( (AABB *) pA, (AABB *) pB );
	//	// For every contact, transform its positions by the 
	//	// translation and rotation of the 
	//}

	// Find the best face-vertex pair between the two
	FaceVertexPair fp( pA, pB );

	// Project the two vertices of the face feature onto
	// the edge formed by two vertices of the vertex feature

	// Get the contact normal
	vec2 cN = GetNormal( fp.pBestFace, fp.ixBestFace );

	// The face edge
	vec2 v2F_e0 = GetVert( fp.pBestFace, fp.ixBestFace );
	vec2 v2F_e1 = GetVert( fp.pBestFace, fp.ixBestFace + 1 );

	// The vertex edge (reversed because we want this CCW)
	vec2 v2V_e0 = GetVert( fp.pBestVert, fp.ixBestVert + 1 );
	vec2 v2V_e1 = GetVert( fp.pBestVert, fp.ixBestVert );

	// The face contact positions are the projections
	// of the two vertex edge points on the face edge
	vec2 v2F_p0 = projectOnEdge( v2V_e0, v2F_e0, v2F_e1 );
	vec2 v2F_p1 = projectOnEdge( v2V_e1, v2F_e0, v2F_e1 );

	// The vertex contact positions are the projection of
	// the two face vertices along the edge formed by V_p0,p1
	vec2 v2V_p0 = projectOnEdge( v2F_e0, v2V_e0, v2V_e1 );
	vec2 v2V_p1 = projectOnEdge( v2F_e1, v2V_e0, v2V_e1 );

	// The contact distances are the projections of the
	// distance between contact points along face normal
	float fDist0 = glm::dot( cN, v2V_p0 - v2F_p0 );
	float fDist1 = glm::dot( cN, v2V_p1 - v2F_p1 );

	//std::cout << glm::vec2( fDist0, fDist1 ) << std::endl;

	// Construct the contact and get out
	return{
		Contact( fp.pBestFace, fp.pBestVert, v2F_p0,  v2V_p0, cN, fDist0 ),
		Contact( fp.pBestFace, fp.pBestVert, v2F_p1,  v2V_p1, cN, fDist1 ),
	};

	//std::list<Contact> ret;

	//// TODO are penetrating features working?
	//FeaturePair mostSeparated( FLT_MAX );
	//FeaturePair mostPenetrating( -FLT_MAX );

	//// Check our faces with their verts, then vice versa
	//featurePairJudgement( mostSeparated, mostPenetrating, pA, pB, FeaturePair::EType::F_V );
	//featurePairJudgement( mostSeparated, mostPenetrating, pA, pB, FeaturePair::EType::V_F );

	//// Pick the correct feature pair
	//bool sep = (mostSeparated.dist > 0 && mostSeparated.T != FeaturePair::EType::N);
	//FeaturePair * fp = sep ? &mostSeparated : &mostPenetrating;

	//// We better have something
	//assert( fp->T != FeaturePair::EType::N );

	//// A is face feature, B is vertex feature
	//OBB * pFace( nullptr ), * pVertex( nullptr );
	//if ( fp->T == FeaturePair::EType::F_V )
	//{
	//	pFace = pA;
	//	pVertex = pB;
	//}
	//else
	//{
	//	pVertex = pA;
	//	pFace = pB;
	//}

	//// Get the world space normal and edge points
	//// of the face feature
	//vec2 wN = GetNormal( pFace, fp->fIdx );
	//vec2 wE0 = GetVert( pFace, fp->fIdx );
	//vec2 wE1 = GetVert( pFace, fp->fIdx + 1 );

	//// Get the world space vertex of the vertex feature,
	//// and then get "supporting neighbor" of that vertex
	//// along the direction of the face feature edge, clockwise
	//std::array<vec2, 2> wV = GetSupportNeighbor( pVertex, -wN, fp->vIdx );

	////std::cout << wN << "\n" << wE0 << "\n" << wE1 << "\n" << wV0 << "\n" << wV1 << "\n" << std::endl;

	//// Project edge points along vertex feature edge
	//vec2 p1 = projectOnEdge( wE0, wV[0], wV[1] );
	//vec2 p2 = projectOnEdge( wE1, wV[0], wV[1] );

	//// Project vertex points along face feature edge
	//vec2 p3 = projectOnEdge( wV[0], wE0, wE1 );
	//vec2 p4 = projectOnEdge( wV[1], wE0, wE1 );

	//// distance is point distance along face (contact) normal
	//float d1 = glm::dot( p1 - p4, wN );
	//float d2 = glm::dot( p2 - p3, wN );

	//// If they're equal, collapse the two into one contact
	//// This could be used as an early out, if you have the balls
	//if ( feq( d1, d2 ) )
	//{
	//	vec2 pA = 0.5f * (p1 + p2);
	//	vec2 pB = 0.5f * (p3 + p4);
	//	float d = glm::distance( pA, pB );
	//	ret.emplace_back( pFace, pVertex, pA, pB, wN, d );
	//}
	//else
	//{   // Otherwise add two contacts points
	//	ret.emplace_back( pFace, pVertex, p1, p4, wN, d1 );
	//	ret.emplace_back( pFace, pVertex, p2, p3, wN, d2 );
	//}

	//return ret;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 OBB::WorldSpaceClamp( const glm::vec2 p ) const
{
	if ( eType == EType::AABB )
		return Clamp( p );

	// Find the clamped point in object space, transform back into world
	vec2 osDiff = glm::inverse( GetRotMat() ) * (p - v2Center);
	osDiff = glm::clamp( osDiff, -boxData.v2HalfDim, boxData.v2HalfDim );
	return v2Center + GetRotMat() * osDiff;

	//vec2 u( cos( fTheta ), sin( fTheta ) );
	//vec2 v = perp( u );
	//vec2 d = p - v2Center;
	//vec2 pP( glm::dot( d, u ), glm::dot( d, v ) );
	//pP = Clamp( pP );
	//return pP.x * u + pP.y * v;
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
	glm::vec2 d = p - pOBB->v2Center;
	glm::vec2 xHat( cosf( pOBB->fTheta ), sinf( pOBB->fTheta ) );
	bool bX = fabs( glm::dot( d, xHat ) ) < pOBB->boxData.v2HalfDim.x;
	bool bY = fabs( glm::dot( d, perp( xHat ) ) ) < pOBB->boxData.v2HalfDim.y;
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