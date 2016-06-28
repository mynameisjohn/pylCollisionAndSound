#pragma once

#include <list>
#include "GL_Util.h"

// Forward all these types, it's all pointer based
class Contact;
class RigidBody2D;
class Circle;
class AABB;
class OBB;

vec2 perp( vec2 v );	// returns (-v.y, v.x)
vec2 maxComp( vec2 v );	// zeroes all but the biggest
vec2 projectOnEdge( vec2 p, vec2 e0, vec2 e1 );


// Functions for getting speculative contacts
std::list<Contact> GetSpecContacts( Circle * pA, Circle * pB );
std::list<Contact> GetSpecContacts( Circle * pCirc, AABB * pAABB );
std::list<Contact> GetSpecContacts( Circle * pCirc, OBB * pOBB );
					  
std::list<Contact> GetSpecContacts( AABB * pA, AABB * pB );
std::list<Contact> GetSpecContacts( AABB * pAABB, OBB * pOBB );
					  
std::list<Contact> GetSpecContacts( OBB * pA, OBB * pB );

////////////////////////////////////////////////////////////////////////////

// Functions for detecting overlap
bool IsOverlapping( Circle * pA, Circle * pB );
bool IsOverlapping( Circle * pCirc, AABB * pAABB );
bool IsOverlapping( Circle * pCirc, OBB * pOBB );

bool IsOverlappingX( AABB * pA, AABB * pB );
bool IsOverlappingY( AABB * pA, AABB * pB );
bool IsOverlapping( AABB * pA, AABB * pB );
bool IsOverlapping( AABB * pAABB, OBB * pOBB );

bool IsOverlapping( OBB * pA, OBB * pB );

////////////////////////////////////////////////////////////////////////////

// Determine if a point is inside an object
bool IsPointInside( vec2 p, Circle * pCirc );
bool IsPointInside( vec2 p, AABB * pAABB );
bool IsPointInside( vec2 p, OBB * pOBB );

////////////////////////////////////////////////////////////////////////////

// Special functions used when working with boxes (they have points)
// Box functions that take an index use the following notation
//			                
//	Verts: 3---0  Normals: --3--
//		   |   |		 <-2   0->
//		   2---1		   --1--
////////////////////////////////////////////////////////////////////////////

// AABB Functions
glm::vec2 GetVert( AABB * pAABB, int idx );	
glm::vec2 GetNormal( AABB * pAABB, int idx );

// OBB Functions
glm::vec2 GetVert( OBB * pOBB, int idx );
glm::vec2 GetNormal( OBB * pOBB, int idx );