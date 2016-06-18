#include "Scene.h"

#include <pyliason.h>
#include <iterator>
#include <string>
#include <map>

using namespace pyl;

bool ExposeEntComponent()
{
	const std::string strModuleName = "pylEntComponent";

	ModuleDef * pModDef = ModuleDef::Create<struct st_ecMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<EntComponent>( "EntComponent" );

	AddMemFnToMod( pModDef, EntComponent, SetID, void, int );
	AddMemFnToMod( pModDef, EntComponent, GetID, int );

	return true;
}

bool ExposeScene()
{
	const std::string strModuleName = "pylScene";

	ModuleDef * pModDef = ModuleDef::Create<struct st_scMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<Scene>( "Scene" );

	AddMemFnToMod( pModDef, Scene, InitDisplay, bool, std::string, uint32_t, uint32_t, uint32_t, uint32_t, vec4 );

	AddMemFnToMod( pModDef, Scene, GetShaderPtr, const Shader * );
	AddMemFnToMod( pModDef, Scene, GetCameraPtr, const Camera * );
	AddMemFnToMod( pModDef, Scene, GetSoundManagerPtr, const SoundManager * );
	AddMemFnToMod( pModDef, Scene, GetDrawable, const Drawable *, size_t );
	AddMemFnToMod( pModDef, Scene, GetRigidBody2D, const RigidBody2D *, size_t );
	AddMemFnToMod( pModDef, Scene, GetContacts, std::list<const Contact *> );

	AddMemFnToMod( pModDef, Scene, AddDrawable, int, std::string, vec2, vec2, vec4 );
	AddMemFnToMod( pModDef, Scene, AddRigidBody, int, RigidBody2D::EType, vec2, vec2, float, float, std::map<std::string, float> );

	AddMemFnToMod( pModDef, Scene, GetQuitFlag, bool );
	AddMemFnToMod( pModDef, Scene, SetQuitFlag, void, bool );
	AddMemFnToMod( pModDef, Scene, SetDrawContacts, void, bool );

	AddMemFnToMod( pModDef, Scene, Update, void );
	AddMemFnToMod( pModDef, Scene, Draw, void );

	return true;
}

using SoundManagerMessage = std::tuple<SoundManager::ECommandID, pyl::Object>;
SoundManager::Command TranslateMessage( SoundManager * pSoundManager, SoundManagerMessage M )
{
	SoundManager::Command cmd;
	if ( pSoundManager == nullptr )
		return cmd;

	using ECommandID = SoundManager::ECommandID;
	ECommandID eCommandID = std::get<0>( M );
	pyl::Object& pylObj = std::get<1>( M );

	// Get data based on CMD ID
	// If a data conversion fails, the default task is 
	// returned (and its CMD ID is none)
	switch ( eCommandID )
	{
		case ECommandID::SetVolume:
		{
			std::tuple<std::string, int, float> setVolData;
			if ( pylObj.convert( setVolData ) == false )
				return cmd;

			cmd.pClip = pSoundManager->GetClip( std::get<0>( setVolData ) );
			cmd.iData = std::get<1>( setVolData );
			cmd.fData = std::get<2>( setVolData );
			break;
		}
		case ECommandID::Start:
		case ECommandID::Stop:
			if ( pylObj.convert( cmd.uData ) == false )
				return cmd;
			break;

		case ECommandID::StartLoop:
		case ECommandID::StopLoop:
		case ECommandID::OneShot:
		{
			std::tuple<std::string, int, float, size_t> setPendingData;
			if ( pylObj.convert( setPendingData ) == false )
				return cmd;

			cmd.pClip = pSoundManager->GetClip( std::get<0>( setPendingData ) );
			cmd.iData = std::get<1>( setPendingData );
			cmd.fData = std::get<2>( setPendingData );
			cmd.uData = std::get<3>( setPendingData );
			break;
		}
		default:
			return cmd;
	}

	cmd.eID = eCommandID;

	return cmd;
}

bool SendMessage( SoundManager * pSoundManager, SoundManagerMessage M )
{
	if ( pSoundManager )
	{
		SoundManager::Command cmd = TranslateMessage( pSoundManager, M );
		return pSoundManager->HandleCommand( cmd );
	}
	return false;
}

bool SendMessages( SoundManager * pSoundManager, std::list<SoundManagerMessage> liMessages )
{
	if ( pSoundManager )
	{
		std::list<SoundManager::Command> liCmds;
		std::transform( liMessages.begin(), liMessages.end(), std::back_inserter( liCmds ), 
						[pSoundManager] ( SoundManagerMessage& M )
		{
			return TranslateMessage( pSoundManager, M );
		} );
		return pSoundManager->HandleCommands( liCmds );
	}
	return false;
}

bool ExposeSoundManager()
{
	const std::string strModuleName = "pylSoundManager";

	ModuleDef * pModDef = ModuleDef::Create<struct st_smMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<SoundManager>( "SoundManager" );

	AddMemFnToMod( pModDef, SoundManager, RegisterClip, bool, std::string, std::string, std::string, size_t );
	AddMemFnToMod( pModDef, SoundManager, Update, void );
	AddMemFnToMod( pModDef, SoundManager, GetSampleRate, size_t );
	AddMemFnToMod( pModDef, SoundManager, GetMaxSampleCount, size_t );
	AddMemFnToMod( pModDef, SoundManager, GetBufferSize, size_t );
	AddMemFnToMod( pModDef, SoundManager, GetNumBufsCompleted, size_t );
	AddMemFnToMod( pModDef, SoundManager, GetNumSamplesInClip, size_t, std::string, bool );
	AddMemFnToMod( pModDef, SoundManager, Init, bool, std::map<std::string, int> );
	AddMemFnToMod( pModDef, SoundManager, PlayPause, bool );

	// These are static functions, so we don't need the macro (doesn't mean there shouldn't be one...)
	pModDef->RegisterFunction<struct st_fnSmSendMessage>( "SendMessage", make_function( SendMessage ) );
	pModDef->RegisterFunction<struct st_fnSmSendMessages>( "SendMessages", make_function( SendMessages ) );

	pModDef->SetCustomModuleInit( [] ( pyl::Object obModule )
	{
		// Expose command enums into the module
		obModule.set_attr( "CMDSetVolume", SoundManager::ECommandID::SetVolume );
		obModule.set_attr( "CMDStart", SoundManager::ECommandID::Start );
		obModule.set_attr( "CMDStartLoop", SoundManager::ECommandID::StartLoop );
		obModule.set_attr( "CMDStop", SoundManager::ECommandID::Stop );
		obModule.set_attr( "CMDStopLoop", SoundManager::ECommandID::StopLoop );
		obModule.set_attr( "CMDPause", SoundManager::ECommandID::Pause );
		obModule.set_attr( "CMDOneShot", SoundManager::ECommandID::OneShot );
	} );

	return true;
}

bool ExposeShader()
{
	const std::string strModuleName = "pylShader";

	ModuleDef * pModDef = ModuleDef::Create<struct st_shMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<Shader>( "Shader" );

	AddMemFnToMod( pModDef, Shader, Init, bool, std::string, std::string, bool );

	AddMemFnToMod( pModDef, Shader, PrintLog_V, int );
	AddMemFnToMod( pModDef, Shader, PrintLog_F, int );
	AddMemFnToMod( pModDef, Shader, PrintSrc_V, int );
	AddMemFnToMod( pModDef, Shader, PrintSrc_F, int );
	AddMemFnToMod( pModDef, Shader, PrintLog_P, int );

	AddMemFnToMod( pModDef, Shader, GetHandle, GLint, const std::string );

	return true;
}

bool ExposeCamera()
{
	const std::string strModuleName = "pylCamera";

	ModuleDef * pModDef = ModuleDef::Create<struct st_caMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<Camera>( "Camera" );

	AddMemFnToMod( pModDef, Camera, InitOrtho, void, float, float, float, float );
	AddMemFnToMod( pModDef, Camera, InitPersp, void, float, float, float, float );

	pModDef->RegisterFunction<struct st_fnCamSetProjH>( "SetCamMatHandle", make_function( Camera::SetCamMatHandle ) );

	return true;
}

bool ExposeDrawable()
{
	const std::string strModuleName = "pylDrawable";

	ModuleDef * pModDef = ModuleDef::Create<struct st_drMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	ModuleDef * pEntMod = ModuleDef::GetModuleDef( "pylEntComponent" );
	pModDef->RegisterClass<Drawable, EntComponent>( "Drawable", pEntMod );

	AddMemFnToMod( pModDef, Drawable, SetTransform, void, quatvec );

	// These are static functions, so we don't need the macro (doesn't mean there shouldn't be one...)
	pModDef->RegisterFunction<struct st_fnDrSetPosH>( "SetPosHandle", make_function( Drawable::SetPosHandle ) );
	pModDef->RegisterFunction<struct st_fnClrSetPosH>( "SetColorHandle", make_function( Drawable::SetColorHandle ) );

	return true;
}

bool ExposeRigidBody2D()
{
	const std::string strModuleName = "pylRigidBody2D";

	ModuleDef * pModDef = ModuleDef::Create<struct st_drMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	ModuleDef * pEntMod = ModuleDef::GetModuleDef( "pylEntComponent" );
	pModDef->RegisterClass<RigidBody2D, EntComponent>( "RigidBody2D", pEntMod );

	AddMemFnToMod( pModDef, RigidBody2D, GetQuatVec, quatvec );

	pModDef->SetCustomModuleInit( [] ( pyl::Object obModule )
	{
		obModule.set_attr( "rbtCircle", RigidBody2D::EType::Circle );
		obModule.set_attr( "rbtAABB", RigidBody2D::EType::AABB );
		obModule.set_attr( "rbtOBB", RigidBody2D::EType::OBB );
	} );

	return true;
}

bool ExposeContact()
{
	const std::string strModuleName = "pylContact";

	ModuleDef * pModDef = ModuleDef::Create<struct st_contactMod>( strModuleName );
	if ( pModDef == nullptr )
		return false;

	pModDef->RegisterClass<Contact>( "Contact" );

	AddMemFnToMod( pModDef, Contact, GetBodyA, const RigidBody2D * );
	AddMemFnToMod( pModDef, Contact, GetBodyB, const RigidBody2D * );

	return true;
}

bool ExposeAll()
{
	// I just wanted to try a polymorphic lambda
	auto liExpose =
	{ ExposeEntComponent,
		ExposeScene,
		ExposeSoundManager,
		ExposeShader,
		ExposeCamera,
		ExposeDrawable,
		ExposeRigidBody2D,
		ExposeContact
	};
	return std::all_of( liExpose.begin(), liExpose.end(), [] ( auto fn ) { return fn(); } );
}

namespace pyl
{
	bool convert( PyObject * o, glm::vec2& v )
	{
		return convert_buf( o, &v[0], 2 );
	}
	bool convert( PyObject * o, glm::vec3& v )
	{
		return convert_buf( o, &v[0], 3 );
	}
	bool convert( PyObject * o, glm::vec4& v )
	{
		return convert_buf( o, &v[0], 4 );
	}
	bool convert( PyObject * o, glm::fquat& v )
	{
		return convert_buf( o, &v[0], 4 );
	}

	bool convert( PyObject * o, quatvec& qv )
	{
		return convert_buf( o, &qv.vec[0], 7 );
	}

	template<typename eType>
	bool convertEnum( PyObject * o, eType& e )
	{
		int tmp( 0 );
		if ( bool bRet = convert( o, tmp ) )
		{
			e = (eType) tmp;
			return bRet;
		}
		return false;
	}

	bool convert( PyObject * o, RigidBody2D::EType& e )
	{
		return convertEnum<RigidBody2D::EType>( o, e );
	}

	bool convert( PyObject * o, SoundManager::ECommandID& e )
	{
		return convertEnum<SoundManager::ECommandID>( o, e );
	}

	PyObject * alloc_pyobject( const SoundManager::ECommandID e )
	{
		return PyLong_FromLong( (long) e );
	}

	PyObject * alloc_pyobject( const RigidBody2D::EType e )
	{
		return PyLong_FromLong( (long) e );
	}

	PyObject * alloc_pyobject( const quatvec& qv )
	{
		if ( PyObject * pList = PyList_New( 7 ) )
		{
			Py_ssize_t i( 0 );
			for ( ; i < 3; i++ )
				PyList_SetItem( pList, i, PyFloat_FromDouble( (double) qv.vec[i] ) );
			for ( ; i < 7; i++ )
				PyList_SetItem( pList, i, PyFloat_FromDouble( (double) qv.quat[i] ) );

			return pList;
		}
		return nullptr;
	}
}