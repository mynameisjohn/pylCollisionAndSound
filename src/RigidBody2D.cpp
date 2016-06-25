#include "RigidBody2D.h"
#include "GL_Util.h"
#include "Util.h"
#include "CollisionFunctions.h"

#include <glm/gtx/norm.hpp>

// Euler integrate rigid body translation/rotation
void RigidBody2D::EulerAdvance( float fDT )
{
	// You should try Verlet...
	v2Center += fDT * v2Vel;

	if ( eType == EType::OBB )
		fTheta += fDT * fOmega;
}

RigidBody2D::RigidBody2D() :
	eType( EType::None ),
	fMass( 0 ),
	fElast( 0 ),
	fTheta( 0 ),
	fOmega( 0 ),
	v2Vel( 0 ),
	v2Center( 0 )
{}

RigidBody2D::RigidBody2D( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float th /*= 0.f*/ ) :
	eType( EType::None ),
	fMass( mass ),
	fElast( elasticity ),
	fTheta( th ),
	fOmega( 0 ),
	v2Vel( vel ),
	v2Center( c )
{
}

/*static*/ RigidBody2D RigidBody2D::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float th /*= 0.f*/ )
{
	return RigidBody2D( vel, c, mass, elasticity, th );
}

vec2 RigidBody2D::GetMomentum() const
{
	return fMass * v2Vel;
}

float RigidBody2D::GetKineticEnergy() const
{
	float fKeTr = 0.5f * fMass * glm::length2( v2Vel );
	float fKeRot = 0.5f * GetInertia() * powf( fOmega, 2 );
	return fKeTr + fKeRot;
}

// Return the graphical quatvec transform of a rigid body
quatvec RigidBody2D::GetQuatVec() const
{
	vec3 pos( v2Center, 0.f );
	fquat rot( cos( fTheta / 2.f ), vec3( 0.f, 0.f, sin( fTheta / 2.f ) ) );
	return quatvec( pos, rot );
}

// Construct rotation matrix
glm::mat2 RigidBody2D::GetRotMat() const
{
	float c = cos( fTheta );
	float s = sin( fTheta ); 
	// glm wants transpose
	return glm::mat2( vec2( c, s ), vec2( -s, c ) );
}

////////////////////////////////////////////////////////////////////////////

/*static*/ std::list<Contact> RigidBody2D::GetSpeculativeContacts( const RigidBody2D * pA, const RigidBody2D * pB )
{
	switch ( pA->eType )
	{
		case EType::Circle: 
		{
			Circle * pCircA = (Circle *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return GetSpecContacts( pCircA, (Circle *) pB );
				case EType::AABB:
					return GetSpecContacts( pCircA, (AABB *) pB );
				case EType::OBB:
					return GetSpecContacts( pCircA, (OBB *) pB );
			}
			break;
		}
		case EType::AABB:
		{
			AABB * pBoxA = (AABB *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return GetSpecContacts( (Circle *) pB, pBoxA );
				case EType::AABB:
					return GetSpecContacts( pBoxA, (AABB *) pB );
				case EType::OBB:
					return GetSpecContacts( pBoxA, (OBB *) pB );
			}
			break;
		}
		case EType::OBB:
		{
			OBB * pBoxA = (OBB *) pA;
			switch ( pB->eType )
			{
				case EType::Circle:
					return GetSpecContacts( (Circle *) pB, pBoxA );
				case EType::AABB:
					return GetSpecContacts( (AABB *) pB, pBoxA );
				case EType::OBB:
					return GetSpecContacts( (OBB *) pA, (OBB *) pB );
			}
			break;
		}
	}

	throw std::runtime_error( "Error: Invalid rigid body type!" );
	return{};
}

float RigidBody2D::GetInertia() const
{
	switch ( eType )
	{
		case RigidBody2D::EType::Circle:
			return 0.5f * fMass * pow( circData.fRadius, 2 );
		case RigidBody2D::EType::AABB:
		case RigidBody2D::EType::OBB:
			return (fMass / 3.f) * (pow( boxData.v2HalfDim.x, 2 ) + pow( boxData.v2HalfDim.y, 2 ));
	}

	throw std::runtime_error( "Error: Inertia queried for invalid rigid body" );
	return 0.f;
}

// I need a good file for these
vec2 perp( vec2 v )
{
	return vec2( -v.y, v.x );
}

vec2 maxComp( vec2 v )
{
	if ( feq( v.x, v.y ) )
		return v;
	else if ( fabs( v.x ) > fabs( v.y ) )
		return vec2( v.x, 0.f );
	else
		return vec2( 0.f, v.y );
}

bool feq( float a, float b, float diff )
{
	return fabs( a - b ) < diff;
}

// Static function to project a point p along the edge
// between points e0 and e1
vec2 projectOnEdge( vec2 p, vec2 e0, vec2 e1 )
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



void FaceVertexPair::judge( OBB * pFace, OBB * pVert )
{
	// For every vertex in the face pair
	for ( int i = 0; i < 4; i++ )
	{
		// Get the vertex
		vec2 V = GetVert( pVert, i );

		// Get the closest point to V on pFace
		vec2 F = pFace->WorldSpaceClamp( V );

		// Get the distance between the face point and vertex
		float fDist = glm::distance2( V, F );

		// If this distance is less than our current dist, 
		// store it as our current best pair
		if ( fDist < fBestDist && !(feq( fDist, fBestDist, 0.01f )) )
		{
			pBestFace = pFace;
			pBestVert = pVert;
			ixBestVert = i;
			fBestDist = fDist;
			v2BestFacePoint = F;
		}
	}
}

FaceVertexPair::FaceVertexPair( OBB * pA, OBB * pB ) :
	ixBestFace( -1 ),
	ixBestVert( -1 ),
	pBestFace( nullptr ),
	pBestVert( nullptr ),
	fBestDist( FLT_MAX )
{
	if ( pA && pB )
	{
		// Get the best pair
		judge( pA, pB );
		judge( pB, pA );



		//if ( pBestFace->eType == RigidBody2D::EType::AABB )
		//{
		//	if ( v2BestFacePoint.x < pBestFace->v2Center.x )
		//	{
		//		if ( v2BestFacePoint.y < pBestFace->Top() )
		//		{
		//			// Left
		//			ixBestFace = 2;
		//		}
		//		else
		//		{
		//			// Top
		//			ixBestFace = 3;
		//		}
		//	}
		//	else
		//	{
		//		if ( v2BestFacePoint.y > pBestFace->Bottom() )
		//		{
		//			// Right
		//			ixBestFace = 0;
		//		}
		//		else
		//		{
		//			// Bottom
		//			ixBestFace = 1;
		//		}
		//	}
		//}
		//else
		{
			//vec2 D = v2BestFacePoint - pBestFace->v2Center;

			// Get the best face idx (from normal)
			// We want the index of the face normal that points
			// most strongly at the closest vertex (ixBestVert)
			// In other words, we want the normal onto which the vector
			// from the best vertex to pBestVert's center projects least
			float fMinProj = FLT_MAX;
			vec2 v2BestVert = GetVert( pBestVert, ixBestVert );
			for ( int i = 0; i < 4; i++ )
			{
				vec2 n = GetNormal( pBestFace, i );
				vec2 p0 = GetVert( pBestFace, i );
				vec2 p1 = GetVert( pBestFace, i + 1 );

				vec2 mfp0 = v2BestVert - p0;
				vec2 mfp1 = v2BestVert - p1;
				vec2 projPt = projectOnEdge( vec2(), mfp0, mfp1 );
				float fProjection = glm::length2( projPt );

				// Get the normal, find the projection, determine if it's a candidate
				
				//float fProjection = glm::dot( n, v2VertexDir );
				if ( fProjection < fMinProj )
				{
					fMinProj = fProjection;
					ixBestFace = i;
				}
			}
		}

		vec2 v2Nrm = GetNormal( pBestFace, ixBestFace );


		// Get the second best vertex, pick it
		// if it's index is lower (edges are CW)
		// (should the normal come into play here?)
		fBestDist = FLT_MAX;
		int ixSecondBest = -2;
		for ( int i = ixBestVert - 1; i <= ixBestVert + 1; i++ )
		{
			if ( i == ixBestVert )
				continue;

			// Get the vertex
			vec2 V = GetVert( pBestVert, i );

			// Get the closest point to V on pFace
			vec2 F = pBestFace->WorldSpaceClamp( V );

			// Get the distance between the face point and vertex
			float fDist = glm::distance2( V, F );

			// If this distance is less than our current dist, 
			// store it as our current best pair
			if ( fDist < fBestDist )
			{
				ixSecondBest = i;
				fBestDist = fDist;
				v2BestFacePoint = F;
			}
		}
		assert( ixSecondBest != -2 );
		if ( ixSecondBest < ixBestVert )
			ixBestVert = ixSecondBest;


		// Store the best face normal
//		vec2 v2FaceNrm = GetNormal( pBestFace, ixBestFace );
		
		// Here we're looking for the neighbor vertex, with ixBestVert, comprises
		// the face whose normal is most out of line with the best face normal

		//// Clockwise neighbor vertex
		//int ixCW = ixBestVert + 1;
		//vec2 nCW = GetNormal( pBestVert, ixCW );
		//float dCW = glm::dot( nCW, v2FaceNrm );

		//// Counterclockwise neigbor vertex
		//int ixCCW = ixBestVert - 1;
		//vec2 nCCW = GetNormal( pBestVert, ixCCW );
		//float dCCW = glm::dot( nCCW, v2FaceNrm );

		//// Store the vertex ix that is the "earliest", clockwise
		//// later on we'll form an edge using ixBestVert, ixBestVert+1
		//if ( dCCW < dCW )
		//	ixBestVert = ixCCW;
	}
}
