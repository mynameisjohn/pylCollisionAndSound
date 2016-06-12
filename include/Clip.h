#pragma once

#include <string>
#include <vector>

// This clip class is kind of like the "weight"
// part of the flyweight... if that makes any sense.
// They own the audio buffers and are instanced out
// in the form of voices that render to the audio buffer
class Clip
{
public:
	Clip();
	Clip( const std::string strName,			// The friendly name of the loop
		  const float * const pHeadBuffer,		// The head buffer
		  const size_t uSamplesInHeadBuffer,	// and its sample count
		  const float * const pTailBuffer,		// The tail buffer
		  const size_t uSamplesInTailBuffer,	// and its sample count
		  const size_t m_uFadeSamples );		// The # of fade samples

	// Get the name
	std::string GetName() const;

	// The sample count, if bTail is true tail samples included
	size_t GetNumSamples( bool bTail = false ) const;

	// The number of fade samles
	size_t GetNumFadeSamples() const;

	// Get a pointer to the internal audio buffer
	float const * GetAudioData() const;

private:
	size_t m_uSamplesInHead;					// The number of samples in the head
	size_t m_uFadeSamples;						// The target sample for the fade-out when stopping
	std::string m_strName;						// The name of the loop (this is never touched by aud thread)
	std::vector<float> m_vAudioBuffer;			// The vector storing the entire head and tail (with fades baked)
};