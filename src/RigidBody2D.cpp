#include "RigidBody2D.h"
#include "GL_Util.h"
#include "Util.h"

// Euler integrate rigid body translation/rotation
void EulerAdvance( RigidBody2D * pBody )
{
	// You should try Verlet...
	pBody->v2Center += g_fTimeStep * pBody->v2Vel;
	pBody->fTheta += g_fTimeStep * pBody->fOmega;
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
{}

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
	return 0.5f * fMass * glm::dot( v2Vel, v2Vel );
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

//////////////////////////////////////////////////////////////////////////////
//
//std::list<Contact> GetContacts( Circle * pA, Circle * pB )
//{
//
//}
//
//std::list<Contact> GetContacts( Circle * pCirc, AABB * pAABB )
//{
//
//}
//
//std::list<Contact> GetContacts( Circle * pCirc, OBB * pObb )
//{
//
//}
//
//////////////////////////////////////////////////////////////////////////////
//
//std::list<Contact> GetContacts( AABB * pA, AABB * pB )
//{
//
//}
//
//std::list<Contact> GetContacts( AABB * pAABB, OBB * pOBB )
//{
//
//}
//
//////////////////////////////////////////////////////////////////////////////
//
//std::list<Contact> GetContacts( OBB * pA, OBB * pB )
//{
//
//}
//
//////////////////////////////////////////////////////////////////////////////
//
///*static*/ std::list<Contact> RigidBody2D::GetSpeculativeContacts( const RigidBody2D * pA, const RigidBody2D * pB )
//{
//	switch ( pA->eType )
//	{
//		case EType::Circle: 
//		{
//			Circle * pCircA = (Circle *) pA;
//			switch ( pB->eType )
//			{
//				case EType::Circle:
//					return GetContacts( pCircA, (Circle *) pB );
//				case EType::AABB:
//					return GetContacts( pCircA, (AABB *) pB );
//				case EType::OBB:
//					return GetContacts( pCircA, (OBB *) pB );
//			}
//			break;
//		}
//		case EType::AABB:
//		{
//			AABB * pBoxA = (AABB *) pA;
//			switch ( pB->eType )
//			{
//				case EType::Circle:
//					return GetContacts( (Circle *) pB, pBoxA );
//				case EType::AABB:
//					return GetContacts( pBoxA, (AABB *) pB );
//				case EType::OBB:
//					return GetContacts( pBoxA, (OBB *) pB );
//			}
//			break;
//		}
//		case EType::OBB:
//		{
//			OBB * pBoxA = (OBB *) pA;
//			switch ( pB->eType )
//			{
//				case EType::Circle:
//					return GetContacts( (Circle *) pB, pBoxA );
//				case EType::AABB:
//					return GetContacts( (AABB *) pB, pBoxA );
//				case EType::OBB:
//					return GetContacts( (OBB *) pA, (OBB *) pB );
//			}
//			break;
//		}
//	}
//
//	throw std::runtime_error( "Error: Invalid rigid body type!" );
//	return{};
//}

////////////////////////////////////////////////////////////////////////////

/*static*/ RigidBody2D Circle::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, float radius )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.circData.fRadius = radius;
	ret.eType = RigidBody2D::EType::Circle;
	return ret;
}

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

glm::vec2 AABB::Clamp( const glm::vec2 p ) const
{
	return glm::clamp( p, v2Center - boxData.v2HalfDim, v2Center + boxData.v2HalfDim );
}

glm::vec2 AABB::GetFaceNormalFromPoint( const glm::vec2 p ) const
{
	vec2 n;

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

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.boxData.v2HalfDim = v2R;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

/*static*/ RigidBody2D AABB::Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h )
{
	RigidBody2D ret = RigidBody2D::Create( vel, vec2( x, y ), mass, elasticity );
	ret.boxData.v2HalfDim = vec2( w, h ) / 2.f;
	ret.eType = RigidBody2D::EType::AABB;
	return ret;
}

////////////////////////////////////////////////////////////////////////////

glm::vec2 OBB::WorldSpaceClamp( const glm::vec2 p ) const
{
	vec2 p1 = glm::inverse( GetRotMat() ) * (p - v2Center);
	p1 = glm::clamp( p1, -boxData.v2HalfDim, boxData.v2HalfDim );
	return v2Center + GetRotMat() * p1;
}

/*static*/ RigidBody2D OBB::Create( glm::vec2 vel, glm::vec2 c, float mass, float elasticity, glm::vec2 v2R, float th /*= 0.f*/ )
{
	RigidBody2D ret = RigidBody2D::Create( vel, c, mass, elasticity );
	ret.boxData.v2HalfDim = v2R / 2.f;
	ret.eType = RigidBody2D::EType::OBB;
	return ret;
}

/*static*/ RigidBody2D OBB::Create( glm::vec2 vel, float mass, float elasticity, float x, float y, float w, float h, float th /*= 0.f*/ )
{
	RigidBody2D ret = RigidBody2D::Create( vel, vec2( x, y ), mass, elasticity );
	ret.boxData.v2HalfDim = vec2( w, h ) / 2.f;
	ret.eType = RigidBody2D::EType::OBB;
	return ret;
}