#include "Scene.h"
#include "Util.h"

#include <glm/gtc/type_ptr.hpp>
#include <algorithm>

Scene::Scene() :
	m_bQuitFlag( false ),
	m_bDrawContacts( false ),
	m_GLContext( nullptr ),
	m_pWindow( nullptr ),
	m_ContactSolver( 10 )
{}

Scene::~Scene()
{
	if ( m_pWindow )
	{
		SDL_DestroyWindow( m_pWindow );
		m_pWindow = nullptr;
	}
	if ( m_GLContext )
	{
		SDL_GL_DeleteContext( m_GLContext );
		m_GLContext = nullptr;
	}
}

void Scene::Draw()
{
	// Clear the screen
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// Bind the shader
	auto sBind = m_Shader.ScopeBind();

	// Get the camera mat as well as some handles
	GLuint pmvHandle = m_Shader.GetHandle( "u_PMV" );
	GLuint clrHandle = m_Shader.GetHandle( "u_Color" );
	mat4 P = m_Camera.GetCameraMat();

	// Draw every Drawable
	for ( Drawable& dr : m_vDrawables )
	{
		mat4 PMV = P * dr.GetMV();
		vec4 c = dr.GetColor();
		glUniformMatrix4fv( pmvHandle, 1, GL_FALSE, glm::value_ptr( PMV ) );
		glUniform4fv( clrHandle, 1, glm::value_ptr( c ) );
		dr.Draw();
	}

	if ( m_bDrawContacts )
	{
		Drawable d1, d2;
		std::array<Drawable, 2> arDrPair;
		for ( Contact& c : m_liSpeculativeContacts )
		{
			// NYI
		}
	}

	// Swap window
	SDL_GL_SwapWindow( m_pWindow );
}

void Scene::Update()
{
	// Update the sound manager (should this happen here?)
	m_SoundManager.Update();

	// Reset the contact list and find contacts
	int nCollisions( 0 );
	float fTotalEnergy( 0.f );
	m_liSpeculativeContacts.clear();

	// Integrate objects
	for ( RigidBody2D& rb : m_vRigidBodies )
		rb.EulerAdvance( g_fTimeStep );

	// Get out if there's less than 2
	if ( m_vRigidBodies.size() < 2 )
		return;

	// Check every one against the other
	for ( auto itOuter = m_vRigidBodies.begin(); itOuter != m_vRigidBodies.end(); ++itOuter )
	{
		for ( auto itInner = itOuter + 1; itInner != m_vRigidBodies.end(); ++itInner )
		{
			std::list<Contact> liNewContacts = RigidBody2D::GetSpeculativeContacts( &*itOuter, &*itInner );
			m_liSpeculativeContacts.splice( m_liSpeculativeContacts.end(), liNewContacts );
		}

		// Increment total energy while we're at it
		fTotalEnergy += itOuter->GetKineticEnergy();
	}

	// Solve contacts
	m_ContactSolver.Solve( m_liSpeculativeContacts );
}

int Scene::AddDrawable( std::string strIqmFile, vec2 T, vec2 S, vec4 C )
{
	Drawable D;
	try
	{
		D.Init( strIqmFile, C, quatvec( vec3( T, 0 ), fquat() ), S );
	}
	catch ( std::runtime_error )
	{
		return -1;
	}

	m_vDrawables.push_back( D );
	return (int) (m_vDrawables.size() - 1);
}

int Scene::AddRigidBody( RigidBody2D::EType eType, glm::vec2 v2Vel, glm::vec2 v2Pos, float fMass, float fElasticity, std::map<std::string, float> mapDetails )
{
	try
	{
		RigidBody2D rb;
		switch ( eType )
		{
			case RigidBody2D::EType::Circle:
			{
				float fRad = mapDetails.at( "r" );
				rb = Circle::Create( v2Vel, v2Pos, fMass, fElasticity, fRad );
				break;
			}
			case RigidBody2D::EType::OBB:
			case RigidBody2D::EType::AABB:
			{
				float w = mapDetails.at( "w" );
				float h = mapDetails.at( "h" );
				if ( eType == RigidBody2D::EType::AABB )
				{
					rb = AABB::Create( v2Vel, v2Pos, fMass, fElasticity, glm::vec2( w, h ) );
				}
				else
				{
					float th = mapDetails.at( "th" );
					rb = OBB::Create( v2Vel, v2Pos, fMass, fElasticity, glm::vec2( w, h ), th );
				}
				break;
			}
			default:
				return -1;
		}

		m_vRigidBodies.push_back( rb );
		return m_vRigidBodies.size() - 1;
	}
	catch ( std::out_of_range )
	{
		std::cerr << "Error! Invalid details provided when creating Rigid Body!" << std::endl;
	}

	return -1;
}

const SoundManager * Scene::GetSoundManagerPtr() const
{
	return &m_SoundManager;
}

const Shader * Scene::GetShaderPtr() const
{
	return  &m_Shader;
}

const Camera * Scene::GetCameraPtr() const
{
	return &m_Camera;
}

const Drawable * Scene::GetDrawable( const size_t drIdx ) const
{
	if ( drIdx < m_vDrawables.size() )
		return &m_vDrawables[drIdx];
	return nullptr;
}

const RigidBody2D * Scene::GetRigidBody2D( const size_t rbIdx ) const
{
	if ( rbIdx < m_vRigidBodies.size() )
		return &m_vRigidBodies[rbIdx];
	return nullptr;
}

void Scene::SetQuitFlag( bool bQuit )
{
	m_bQuitFlag = bQuit;
}

bool Scene::GetQuitFlag() const
{
	return m_bQuitFlag;
}

void Scene::SetDrawContacts( bool bDrawContacts )
{
	m_bDrawContacts = bDrawContacts;
}

bool Scene::InitDisplay( std::string strWindowName, uint32_t glMajor, uint32_t glMinor, uint32_t iScreenW, uint32_t iScreenH, vec4 v4ClearColor )
{
	// Create Window (only used for keyboard input, as of now)
	m_pWindow = SDL_CreateWindow( strWindowName.c_str(),
								  SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
								  iScreenW, iScreenH,
								  SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN );
	if ( m_pWindow == NULL )
	{
		std::cout << "Window could not be created! SDL Error: " << SDL_GetError() << std::endl;
		return false;
	}

	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, glMajor );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, glMinor );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
	m_GLContext = SDL_GL_CreateContext( m_pWindow );
	if ( m_GLContext == nullptr )
	{
		std::cout << "Error creating opengl context" << std::endl;
		return false;
	}

	//Initialize GLEW
	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();
	if ( glewError != GLEW_OK )
	{
		printf( "Error initializing GLEW! %s\n", glewGetErrorString( glewError ) );
		return false;
	}

	//Use Vsync
	if ( SDL_GL_SetSwapInterval( 1 ) < 0 )
	{
		printf( "Warning: Unable to set VSync! SDL Error: %s\n", SDL_GetError() );
	}

	//OpenGL settings
	glClearColor( v4ClearColor.x, v4ClearColor.y, v4ClearColor.z, v4ClearColor.w );
	glEnable( GL_DEPTH_TEST );
	glDepthMask( GL_TRUE );
	glDepthFunc( GL_LESS );
	glEnable( GL_MULTISAMPLE_ARB );

	//For debugging
	glLineWidth( 8.f );
}

// This is a dumb function...
std::list<const Contact *> Scene::GetContacts() const
{
	std::list<const Contact *> liRet;
	std::transform( m_liSpeculativeContacts.begin(), m_liSpeculativeContacts.end(), std::back_inserter( liRet ),
					[] ( const Contact& c )
	{
		return &c;
	} );
	return liRet;
}