#pragma once

#include <list>
#include "GL_Util.h"

// Forward all these types, it's all pointer based
class Contact;
class RigidBody2D;
class Circle;
class AABB;
class OBB;

vec2 perp( vec2 v );
vec2 maxComp( vec2 v );

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

bool IsOverlapping( AABB * pA, AABB * pB );
bool IsOverlapping( AABB * pAABB, OBB * pOBB );

bool IsOverlapping( OBB * pA, OBB * pB );

////////////////////////////////////////////////////////////////////////////

glm::vec2 GetVert( AABB * pAABB, int idx );
glm::vec2 GetNormal( AABB * pAABB, int idx );
glm::vec2 GetVert( OBB * pOBB, int idx );
glm::vec2 GetNormal( OBB * pOBB, int idx );
int GetSupportVerts( OBB * pOBB, glm::vec2 n, std::array<glm::vec2, 2>& sV );
int GetSupportIndices( OBB * pOBB, glm::vec2 n, std::array<int, 2>& sV );
std::array<glm::vec2, 2> GetSupportNeighbor( OBB * pOBB, glm::vec2 n, int idx );
glm::vec2 GetSupportNormal( OBB * pOBB, glm::vec2 N );