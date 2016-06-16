import Init

g_Engine = None

def Initialize(pScene):
    # Init the scene
    global g_Engine
    g_Engine = Init.InitScene(pScene)

    # Start the engine
    g_Engine.Start()

def Update(pScene):
    g_Engine.Update()