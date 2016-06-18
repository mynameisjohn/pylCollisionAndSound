#include <pyliason.h>
#include "InitPython.h"
#include "Scene.h"

int main(int argc, char ** argv)
{
	ExposeAll();
	pyl::initialize();

	pyl::Object obMainModule = pyl::Object::from_script( "../scripts/main.py" );
	Scene S;
	obMainModule.call_function( "Initialize", &S );

	while ( S.GetQuitFlag() == false )
	{
		SDL_Event e{ 0 };
		while ( SDL_PollEvent( &e ) )
		{
			obMainModule.call_function( "HandleEvent", &e );
		} 

		obMainModule.call_function( "Update", &S );
	}

	pyl::finalize();
	return 0;
}