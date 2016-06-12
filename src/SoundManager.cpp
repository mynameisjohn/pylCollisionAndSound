#include "SoundManager.h"
#include "Clip.h"
#include "Voice.h"

#include <iostream>
#include <algorithm>

SoundManager::SoundManager() :
	m_bPlaying( false ),
	m_uSamplePos( 0 ),
	m_uMaxSampleCount( 0 ),
	m_uNumBufsCompleted( 0 )
{}

// The userdata member of our audio spec
// is set to our this pointer if we initialized
// the audio, in which case we should tear it down
SoundManager::~SoundManager()
{
	if ( this == m_AudioSpec.userdata )
	{
		SDL_CloseAudio();
		memset( &m_AudioSpec, 0, sizeof( SDL_AudioSpec ) );
	}
}

// Register a clip with the SoundManager so it can be recalled later as a voice. A clip can contain a
// head file, tail file, and a sample count for the fade (fade up from zero, fade out to next loop, etc.) 
bool SoundManager::RegisterClip( std::string strClipName, std::string strHeadFile, std::string strTailFile, size_t uFadeDurationMS )
{
	// Because the audio thread walks the map we're about to add something to, we
	// don't allow registering clips while the audio is playing
	if ( m_bPlaying )
	{
		std::cerr << "Error: Attempting to register clip " << strClipName << " with playing SoundManager!" << std::endl;
		return false;
	}

	// If we already have this clip stored, return true
	// (I should make a way of unregistering, or allowing overwrites)
	if ( m_mapClips.find( strClipName ) != m_mapClips.end() )
		return true;

	// This will get filled in if we load successfully
	float * pSoundBuffer( nullptr );
	Uint32 uNumBytesInHead( 0 );
	float * pTailBuffer( nullptr );
	Uint32 uNumBytesInTail( 0 );

	// The spec of the audio, checked against ours
	SDL_AudioSpec wavSpec{ 0 }, refSpec = m_AudioSpec;
	auto checkAudioSpec = [refSpec, &wavSpec] ()
	{
		return (refSpec.freq == wavSpec.freq &&
				 refSpec.format == wavSpec.format &&
				 refSpec.channels == wavSpec.channels &&
				 refSpec.samples == wavSpec.samples);
	};

	// Load the head file, check against our spec
	if ( SDL_LoadWAV( strHeadFile.c_str(), &wavSpec, (Uint8 **) &pSoundBuffer, &uNumBytesInHead ) )
	{
		if ( checkAudioSpec() )
		{
			// Load the tail file, check against our spec
			if ( SDL_LoadWAV( strTailFile.c_str(), &wavSpec, (Uint8 **) &pTailBuffer, &uNumBytesInTail ) )
			{
				if ( checkAudioSpec() == false )
				{
					// It's ok if this fails, just zero these guys
					pTailBuffer = nullptr;
					uNumBytesInTail = 0;
				}
			}

			// Construct the clip
			const size_t uNumSamplesInHead = uNumBytesInHead / sizeof( float );
			const size_t uNumSamplesInTail = uNumBytesInTail / sizeof( float );
			m_uMaxSampleCount = std::max( m_uMaxSampleCount, uNumSamplesInHead );
			m_mapClips[strClipName] = Clip( strClipName, pSoundBuffer, uNumSamplesInHead, pTailBuffer, uNumSamplesInTail, uFadeDurationMS );
			return true;
		}
	}

	return false;
}

// Called by main thread, locks mutex
void SoundManager::getMessagesFromAudThread()
{
	// We gotta lock this while we mess with the public queue
	std::lock_guard<std::mutex> lg( m_muAudioMutex );

	// Get out if empty
	if ( m_liPublicCmdQueue.empty() )
		return;

	// Grab the front, maybe inc buf count and pop
	Command tFront = m_liPublicCmdQueue.front();
	if ( tFront.eID == ECommandID::BufCompleted )
	{
		m_liPublicCmdQueue.pop_front();
		m_uNumBufsCompleted += tFront.uData;
	}
}

// Called by main thread
void SoundManager::Update()
{
	// Just see if the audio thread has left any
	// BufCompleted tasks for us
	getMessagesFromAudThread();
}

// Called by audio thread, locks mutex
void SoundManager::getMessagesFromMainThread()
{
	// Take any tasks the main thread has left us
	// and put them into our queue
	{
		std::lock_guard<std::mutex> lg( m_muAudioMutex );

		// We're going to leave the main thread a task indicating how many
		// buffers have completed since the last time it checked the queue
		Command tNumBufsCompleted;

		// Format the taks - we leave it with one buf
		tNumBufsCompleted.eID = ECommandID::BufCompleted;
		tNumBufsCompleted.uData = 1;

		// See if there's anything still in the public queue
		if ( m_liPublicCmdQueue.empty() == false )
		{
			// The first task might be a leftover numBufsCompleted
			// (happens if our buffer size is lower than the refresh rate)
			Command tFront = m_liPublicCmdQueue.front();
			if ( tFront.eID == ECommandID::BufCompleted )
			{
				// If it's a buf completed task, pop it off
				// and add its sample count to the one declared above
				tNumBufsCompleted.uData += tFront.uData;
				m_liPublicCmdQueue.pop_front();
			}

			// Take all other tasks and add them to our queue
			m_liAudioCmdQueue.splice( m_liAudioCmdQueue.end(), m_liPublicCmdQueue );
		}

		// The public queue is empty, leave it with the # of buffers completed
		m_liPublicCmdQueue = { tNumBufsCompleted };
	}

	// Remove any voices that have stopped
	m_liVoices.remove_if( [] ( const Voice& v ) { return v.GetState() == Voice::EState::Stopped; } );

	// Handle each task
	for ( Command cmd : m_liAudioCmdQueue )
	{
		// Find the voice associated with the command's ID - this is dumb, but easy
		auto prFindVoice = [cmd] ( const Voice& v ) { return v.GetID() == cmd.iData; };
		auto itVoice = std::find_if( m_liVoices.begin(), m_liVoices.end(), prFindVoice );

		// Handle the command
		switch ( cmd.eID )
		{
			// Start every loop
			case ECommandID::Start:
				for ( auto& itLoop : m_mapClips )
					m_liVoices.emplace_back( &itLoop.second, cmd.uData, cmd.fData, false );
				break;

				// Stop every loop
			case ECommandID::Stop:
				for ( Voice& v : m_liVoices )
					v.SetStopping( cmd.uData );
				break;

				// Start a specific loop
			case ECommandID::StartLoop:
			case ECommandID::OneShot:
				// If it isn't already there, construct the voice
				if ( itVoice == m_liVoices.end() )
					m_liVoices.emplace_back( cmd );
				// Otherwise try set the voice to pending
				else
					itVoice->SetPending( cmd.uData, cmd.eID == ECommandID::StartLoop );
				break;

				// Stop a specific loop
			case ECommandID::StopLoop:
				if ( itVoice != m_liVoices.end() )
					itVoice->SetStopping( cmd.uData );
				break;

				// Set the volume of a loop
			case ECommandID::SetVolume:
				if ( itVoice != m_liVoices.end() )
					itVoice->SetVolume( cmd.fData );
				break;

				// Uhhh
			case ECommandID::Pause:
			default:
				break;
		}
	}

	// The audio thread doesn't care about anything else
	// so clear the list
	m_liAudioCmdQueue.clear();
}

// Initialize the sound manager's audio spec
bool SoundManager::Init( std::map<std::string, int> mapAudCfg )
{
	try
	{
		m_AudioSpec.freq = mapAudCfg.at( "freq" );
		m_AudioSpec.channels = mapAudCfg.at( "channels" );
		m_AudioSpec.samples = mapAudCfg.at( "bufSize" );

	}
	catch ( std::out_of_range )
	{
		memset( &m_AudioSpec, 0, sizeof( SDL_AudioSpec ) );
		return false;
	}

	m_AudioSpec.format = AUDIO_F32;
	m_AudioSpec.callback = (SDL_AudioCallback) SoundManager::FillAudio;
	m_AudioSpec.userdata = this;

	SDL_AudioSpec received;
	if ( SDL_OpenAudio( &m_AudioSpec, &received ) )
	{
		std::cout << "Error initializing SDL Audio" << std::endl;
		std::cout << SDL_GetError() << std::endl;
		memset( &m_AudioSpec, 0, sizeof( SDL_AudioSpec ) );
		return false;
	}

	m_bPlaying = false;

	return true;
}

// TODO Have this send a message telling all loops to fade to silence
// without blowing out sample position, and actually pause once that's done
bool SoundManager::PlayPause()
{
	// This gets set if configure is successful
	if ( m_AudioSpec.userdata == nullptr )
		return false;

	// Toggle audio playback (and bool)
	m_bPlaying = !m_bPlaying;

	if ( m_bPlaying )
		SDL_PauseAudio( 0 );
	else
		SDL_PauseAudio( 1 );

	return true;
}

size_t SoundManager::GetSampleRate() const
{
	return m_AudioSpec.freq;
}

size_t SoundManager::GetBufferSize() const
{
	return m_AudioSpec.samples;
}

size_t SoundManager::GetMaxSampleCount() const
{
	return m_uMaxSampleCount;
}

size_t SoundManager::GetNumBufsCompleted() const
{
	return m_uNumBufsCompleted;
}

size_t SoundManager::GetNumSamplesInClip( std::string strClipName, bool bTail /*= false*/ ) const
{
	auto it = m_mapClips.find( strClipName );
	if ( it != m_mapClips.end() )
		return it->second.GetNumSamples( bTail );
	return 0;
}

SDL_AudioSpec const * SoundManager::GetAudioSpecPtr() const
{
	return &m_AudioSpec;
}

// Called via the static fill_audi function
void SoundManager::fill_audio_impl( Uint8 * pStream, int nBytesToFill )
{
	// Don't do nothin if they gave us nothin
	if ( pStream == nullptr || nBytesToFill == 0 )
		return;

	// Silence no matter what
	memset( pStream, 0, nBytesToFill );

	// Get tasks from public thread and handle them
	// Also let them know a buffer is about to complete
	getMessagesFromMainThread();

	// Nothing to do
	if ( m_liVoices.empty() )
		return;

	// The number of float samples we want
	const size_t uNumSamplesDesired = nBytesToFill / sizeof( float );

	// Fill audio data for each loop
	for ( Voice& v : m_liVoices )
		v.RenderData( (float *) pStream, uNumSamplesDesired, m_uSamplePos );

	// Update sample counter, reset if we went over
	m_uSamplePos += uNumSamplesDesired;
	if ( m_uSamplePos > m_uMaxSampleCount )
	{
		// Just do a mod
		m_uSamplePos %= m_uMaxSampleCount;
	}
}

// Static SDL audio callback function (each instance sets its own userdata to this, so I guess
// multiple instances are legit)
/*static*/ void SoundManager::FillAudio( void * pUserData, Uint8 * pStream, int nSamplesDesired )
{
	// livin on a prayer
	((SoundManager *) pUserData)->fill_audio_impl( pStream, nSamplesDesired );
}