#pragma once

#include <chrono>
#include <iostream>

// remaps x : [m0, M0] to the range of [m1, M1]
inline float remap( float x, float m0, float M0, float m1, float M1 )
{
	return m1 + ((x - m0) / (M0 - m0)) * (M1 - m1);
}

// Always useful
using Time = std::chrono::high_resolution_clock;
class StopWatch
{
	decltype(Time::now()) m_Begin, m_End;
	std::string m_Name;
public:
	StopWatch( std::string s ) :
		m_Begin( Time::now() ),
		m_Name( s )
	{
	}
	~StopWatch()
	{
		using std::chrono::duration_cast;
		using std::chrono::milliseconds;
		m_End = Time::now();
		std::cout << m_Name << " took " << duration_cast<milliseconds>(m_End - m_Begin).count() << " mS";
	}
};