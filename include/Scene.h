#pragma once

#include "RigidBody2D.h"
#include "Contact.h"
#include "SoundManager.h"
#include "Camera.h"
#include "Shader.h"
#include "Drawable.h"

#include <vector>

#include <SDL.h>

class Scene
{
public:
	Scene();
	~Scene();

	void Draw();
	void Update();

	void SetQuitFlag( bool bQuit );
	bool GetQuitFlag() const;

	void SetDrawContacts( bool bDrawContacts );

	const Shader * GetShaderPtr() const;
	const Camera * GetCameraPtr() const;
	const SoundManager * GetSoundManagerPtr() const;
	const Drawable * GetDrawable( const size_t drIdx ) const;
	const RigidBody2D * GetRigidBody2D( const size_t rbIdx ) const;

	bool InitDisplay( std::string strWindowName, uint32_t glMajor, uint32_t glMinor, uint32_t iScreenW, uint32_t iScreenH, vec4 v4ClearColor );
	
	int AddDrawable( std::string strIqmFile, vec2 T, vec2 S, vec4 C );
	
	int AddRigidBody( RigidBody2D::EType eType, glm::vec2 v2Vel, glm::vec2 v2Pos, float fMass, float fElasticity, std::map<std::string, float> mapDetails );

	std::list<const Contact *> GetContacts() const;

private:
	bool m_bQuitFlag;
	bool m_bDrawContacts;
	SDL_GLContext m_GLContext;
	SDL_Window * m_pWindow;
	Shader m_Shader;
	SoundManager m_SoundManager;
	Camera m_Camera;
	std::list<Contact> m_liSpeculativeContacts;
	Contact::Solver m_ContactSolver;
	std::vector<Drawable> m_vDrawables;
	std::vector<RigidBody2D> m_vRigidBodies;
};
