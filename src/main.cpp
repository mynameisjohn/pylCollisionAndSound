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

	pyl::finalize();
	return 0;
}