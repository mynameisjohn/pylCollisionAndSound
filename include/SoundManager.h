#pragma once

#include <string>
#include <map>
#include <list>
#include <mutex>
#include <stdint.h>
#include <memory>

// Forwards for clip and voice
class Clip;
class Voice;
struct SDL_AudioSpec;

// The SoundManager class is our interface to SDL_Audio
// It fields the callback from SDL Audio and passes data
// to and from our main / audio threads
class SoundManager
{
public:
	// Command IDs
	enum class ECommandID : int
	{
		// These commands are sent to the audio thread
		None = 0,
		SetVolume,
		Start,
		StartLoop,
		Pause,
		Stop,
		StopLoop,
		OneShot,
		////////////////////////////
		// These commands are sent from the audio thread
		BufCompleted
	};

	// Command class, stores the necessary information
	// to carry out an action (and then some)
	struct Command
	{
		ECommandID eID{ ECommandID::None };
		Clip * pClip{ nullptr };
		int iData{ -1 };
		float fData{ 1.f };
		size_t uData{ 0 };
	};

	// Default constructor is boring
	SoundManager();
	bool Init( std::map<std::string, int> mapAudCfg );

	// Destructor tears down SDL Audio if it was started
	~SoundManager();

	// Called periodically to pick up messages posted by aud thread
	void Update();

	// Play / Pause the audio device
	bool GetPlayPause() const;
	void SetPlayPause( bool bPlayPause );

	// Various gets
	size_t GetMaxSampleCount() const;
	size_t GetSampleRate() const;
	size_t GetBufferSize() const;
	size_t GetNumBufsCompleted() const;
	size_t GetNumSamplesInClip( std::string strClipName, bool bTail ) const;
	SDL_AudioSpec const * GetAudioSpecPtr() const;

	bool HandleCommand( Command cmd );
	bool HandleCommands( std::list<Command> cmd );

	Clip * GetClip( std::string strClipName ) const;

	// Add a clip to storage, can be recalled later as a Voice
	bool RegisterClip( std::string strClipName, std::string strHeadFile, std::string strTailFile, size_t uFadeDurationMS );

	// SDL Audio callback, will end up calling fill_audio_impl on a SoundManager instance
	static void FillAudio( void * pUserData, uint8_t * pStream, int nSamplesDesired );

private:
	bool m_bPlaying;						// Whether or not we are filling buffers of audio
	size_t m_uMaxSampleCount;				// Sample count of longest loop
	size_t m_uNumBufsCompleted;             // The number of buffers filled by the audio thread
	std::unique_ptr<SDL_AudioSpec> m_pAudioSpec;				// Audio spec, describes loop format

	std::mutex m_muAudioMutex;				// Mutex controlling communication between audio and main threads
	size_t m_uSamplePos;					// Current sample pos in playback
	std::list<Command> m_liPublicCmdQueue;	// Anyone can put tasks here, will be read by audio thread
	std::list<Command> m_liAudioCmdQueue;	// Audio thread's tasks, only modified by audio thread
	std::map<std::string, Clip> m_mapClips;	// Clip storage, right now the map is a convenience
	std::list<Voice> m_liVoices;

	// The actual callback function used to fill audio buffers
	void fill_audio_impl( uint8_t * pStream, int nBytesToFill );

	// Called by audio thread to get messages from main thread
	void getMessagesFromMainThread();

	// Called from Update to find out if we've filled some buffers
	void getMessagesFromAudThread();
};