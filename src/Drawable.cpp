#include "Drawable.h"
#include "IqmFile.h"

#include <string>

// Static var declarations
/*static*/ GLint Drawable::s_PosHandle;
/*static*/ GLint Drawable::s_ColorHandle;
/*static*/ std::map<std::string, std::array<GLuint, 2> > Drawable::s_VAOCache;
/*static*/ std::map<std::string, Drawable> Drawable::s_PrimitiveMap;

Drawable::Drawable() :
	m_VAO( 0 ),
	m_nIdx( 0 ),
	m_v2Scale( 1 ),
	m_v4Color( 1 ),
	m_qvTransform( quatvec::Type::TRT )
{}

bool Drawable::Init( std::string strIqmSrcFile, glm::vec4 v4Color, quatvec qvTransform, glm::vec2 v2Scale )
{
	if ( Drawable::s_PosHandle < 0 )
	{
		std::cerr << "Error: you haven't initialized the static pos handle for drawables!" << std::endl;
		return false;
	}

	// See if we've loaded this Iqm File before
	if ( s_VAOCache.find( strIqmSrcFile ) == s_VAOCache.end() )
	{
		// Try and construct the drawable from an IQM file
		try
		{
			// We'll be creating an indexed array of VBOs
			GLuint VAO( 0 ), nIdx( 0 );

			// Lambda to generate a VBO
			auto makeVBO = []
				( GLuint buf, GLint handle, void * ptr, GLsizeiptr numBytes, GLuint dim, GLuint type )
			{
				glBindBuffer( GL_ARRAY_BUFFER, buf );
				glBufferData( GL_ARRAY_BUFFER, numBytes, ptr, GL_STATIC_DRAW );
				glEnableVertexAttribArray( handle );
				glVertexAttribPointer( handle, dim, type, 0, 0, 0 );
				//Disable?
			};

			// Construct the file, this can throw an error
			IQMFile f( strIqmSrcFile.c_str() );

			// Create vertex array object
			glGenVertexArrays( 1, &VAO );
			if ( VAO == 0 )
			{
				std::cerr << "Error creating VAO for " << strIqmSrcFile << std::endl;
				return false;
			}

			// Bind if successful
			glBindVertexArray( VAO );

			// Create VBOs
			std::array<GLuint, 2> vboBuf{ { 0, 0 } };
			glGenBuffers( vboBuf.size(), vboBuf.data() );
			if ( vboBuf[0] == 0 || vboBuf[1] == 0 )
			{
				std::cerr << "Error creating VBOs " << strIqmSrcFile << std::endl;
				return false;
			}

			// If successful, bind position attr and upload data
			GLuint bufIdx( 0 );
			auto pos = f.Positions();
			makeVBO( vboBuf[bufIdx++], s_PosHandle, pos.ptr(), pos.numBytes(), pos.nativeSize() / sizeof( float ), GL_FLOAT );

			// Same for indices
			auto idx = f.Indices();
			glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, vboBuf[bufIdx] );
			glBufferData( GL_ELEMENT_ARRAY_BUFFER, idx.numBytes(), idx.ptr(), GL_STATIC_DRAW );

			// Unbind VAO and cache data
			glBindVertexArray( 0 );

			nIdx = idx.count();

			s_VAOCache[strIqmSrcFile] = { VAO, nIdx };
		}
		catch ( std::runtime_error )
		{
			std::cerr << "Error constructing drawable from " << strIqmSrcFile << std::endl;
			return false;
		}
	}

	// Store the values from the static cache, return true
	m_VAO = s_VAOCache[strIqmSrcFile][0];
	m_nIdx = s_VAOCache[strIqmSrcFile][1];

	return true;
}

vec4 Drawable::GetColor() const
{
	return m_v4Color;
}

vec3 Drawable::GetPos() const
{
	return m_qvTransform.vec;
}

fquat Drawable::GetRot() const
{
	return m_qvTransform.quat;
}

quatvec Drawable::GetTransform() const
{
	return m_qvTransform;
}

mat4 Drawable::GetMV() const
{
	return m_qvTransform.ToMat4() * glm::scale( vec3( m_v2Scale, 1.f ) );
}

void Drawable::SetPos( vec3 t )
{
	m_qvTransform.vec = t;
}

void Drawable::Translate( vec3 t )
{
	m_qvTransform.vec += t;
}

void Drawable::SetPos( vec2 t )
{
	m_qvTransform.vec = vec3( t, 0 );
}

void Drawable::Translate( vec2 t )
{
	m_qvTransform.vec += vec3( t, 0 );
}

void Drawable::SetRot( fquat q )
{
	m_qvTransform.quat = q;
}

void Drawable::Rotate( fquat q )
{
	m_qvTransform.quat *= q;
}

void Drawable::SetTransform( quatvec qv )
{
	m_qvTransform = qv;
}

void Drawable::Transform( quatvec qv )
{
	m_qvTransform *= qv;
}

void Drawable::SetColor( vec4 c )
{
	m_v4Color = glm::clamp( c, vec4( 0 ), vec4( 1 ) );
}

bool Drawable::Draw()
{
	if ( s_PosHandle < 0 || s_ColorHandle < 0 )
	{
		std::cerr << "Error! Static drawable handles not set!" << std::endl;
		return false;
	}

	// Bind VAO, draw, don't bother unbinding
	// We could upload the color and MV here,
	// but because P*MV can be done beforehand
	// it's kind of an optimization to leave it outside
	glBindVertexArray( m_VAO );
	glDrawElements( GL_TRIANGLES, m_nIdx, GL_UNSIGNED_INT, NULL );
	glBindVertexArray( m_VAO );

	return true;
}

/*static*/ void Drawable::SetPosHandle( GLint pH )
{
	s_PosHandle = pH;
}

/*static*/ GLint Drawable::GetPosHandle()
{
	return s_PosHandle;
}

/*static*/ void Drawable::SetColorHandle( GLint cH )
{
	s_ColorHandle = cH;
}

/*static*/ GLint Drawable::GetColorHandle()
{
	return s_ColorHandle;
}

/*static*/ bool Drawable::AddPrimitive( std::string strPrimFile, glm::vec4 v4Color, quatvec qvTransform, glm::vec2 v2Scale )
{
	Drawable D;
	if ( D.Init( strPrimFile, v4Color, qvTransform, v2Scale ) )
	{
		s_PrimitiveMap[strPrimFile] = D;
		return true;
	}

	return false;
}

/*static*/ Drawable * Drawable::GetPrimitive( std::string strPrimFile )
{
	auto it = s_PrimitiveMap.find( strPrimFile );
	if ( it == s_PrimitiveMap.end() )
		return nullptr;
	return &it->second;
}

/*static*/ bool Drawable::DrawPrimitive( std::string strPrimFile )
{
	if ( Drawable * pDr = Drawable::GetPrimitive( strPrimFile ) )
	{
		pDr->Draw();
		return true;
	}

	return false;
}