# Used for debugging
#import ptvsd
#ptvsd.enable_attach(secret = None)
#ptvsd.wait_for_attach()

import Init

g_Engine = None

def Initialize(pScene):
    # Init the scene
    global g_Engine
    g_Engine = Init.InitScene(pScene)

    # Start the engine
    g_Engine.StartStop(True)

def Update(pScene):
    g_Engine.Update()

def HandleEvent(pSdlEvent):
    g_Engine.HandleEvent(pSdlEvent)