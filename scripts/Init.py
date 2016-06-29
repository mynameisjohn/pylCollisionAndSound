# pyl modules
import pylSoundManager
import pylScene
import pylDrawable
import pylShader
import pylCamera
import pylRigidBody2D as pylRB2D

import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/sdl2')
import events as SDLEvents
import keycode as SDLK

import math
import networkx as nx
import itertools

from LoopManager import *
from StateGraph import StateGraph
from InputManager import *

import Engine
import Entity

def InitEntities(cScene, loopManager):
    # Iterate loopmanager states
    # Create entities for each state
    # Also create entites that have
    # space reserved for voices, and play
    # those via a OneShot message

    # Returna list of entities
    liEntities = []

    # Add one entity for each state, as an OBB and a state as its sound component
    nodes = loopManager.GetStateGraph().G.nodes()
    dTH = 2 * math.pi / len(nodes)
    for idx, soundNode in zip(range(len(nodes)), nodes):
        # We're going to start them off in a circle 
        th = idx * dTH - math.pi/2
        pos = [10*math.cos(th)/2, 10*math.sin(th)/2]

        # Velocity is toward origin
        vel = [-2*p for p in pos]

        # Pick a random scale between 0.4 and 2.5 
        scale = [random.uniform(0.4, 2.5) for i in range(2)]

        # Elasticty is uniformly 1, mass is random between same range as scale
        elast = 1.
        mass = random.uniform(0.4, 2.5)

        # Color is the sound state color
        clr = DrawableLoopState.clrOff
        if soundNode == loopManager.GetStateGraph().activeState:
            clr = DrawableLoopState.clrPlaying

        # Choose a collision primitive at random
        prim = random.choice([pylRB2D.rbtOBB, pylRB2D.rbtCircle])

        # Fill in missing parameters (detail map for rb, IQM file for dr)
        # OBBs need widtih, height, angle (chosen at random) and quad for IQM
        if prim == pylRB2D.rbtOBB:
            detailMap = { 'w' : scale[0], 'h' : scale[1], 'th' : random.uniform(0., 1.)}
            iqmFile = '../models/quad.iqm'

        # AABB is similar, but no angle
        elif prim == pylRB2D.rbtAABB:
            detailMap = { 'w' : scale[0], 'h' : scale[1]}
            iqmFile = '../models/quad.iqm'

        # Circles have a circle model file and a radius
        elif prim == pylRB2D.rbtCircle:
            # The scale for circles must be uniform
            scale = [max(scale) for i in range(len(scale))]
            detailMap = {'r' : scale[0]/2.}
            iqmFile = '../models/circle.iqm'

        # Something horrible has happened
        else:
            raise RuntimeError('Error: Invalid collision primitive!')

        # Construct the entity and append it to the list
        liEntities.append(Entity.Entity(cScene, 
                          cScene.AddRigidBody(prim, vel, pos, mass, elast, detailMap),
                          cScene.AddDrawable(iqmFile, pos, scale, clr),
                          soundNode))

    # Create some non-entity walls
    wallPos = [[-10, 0], [10, 0], [0, -10], [0, 10]]
    wallDim = [[1, 20], [1, 20], [20, 1], [20, 1]]
    for wP, wD in zip(wallPos, wallDim):
        cScene.AddRigidBody(pylRB2D.rbtAABB, [0, 0], wP, -1, 1, {'w':wD[0], 'h':wD[1]} )
        cScene.AddDrawable('../models/quad.iqm', wP, wD, [0,0,0,1])

    # return the list
    return liEntities 

# Function to create state graph
def InitLoopManager(cScene):
    # Create cSM wrapper
    cSM = pylSoundManager.SoundManager(cScene.GetSoundManagerPtr())

    # Create the states (these three sequences are common to all)
    lSeq_chSustain = LoopSequence('chSustain',[Loop('chSustain1', 'chSustain1_head.wav', 5, 1., 'chSustain1_tail.wav')])
    lSeq_bass = LoopSequence('bass',[Loop('bass', 'bass1_head.wav', 5, 1.,'bass1_tail.wav')])
    lSeq_drums = LoopSequence('drums',[Loop('drum1', 'drum1_head.wav', 5, 1.,'drum1_tail.wav')])

    # State 1 just plays lead1, lead2, lead1, lead2...
    s1 = DrawableLoopState('One', {lSeq_chSustain, lSeq_bass, lSeq_drums,
        LoopSequence('lead',[
            Loop('lead1', 'lead1.wav'),
            Loop('lead2', 'lead2_head.wav', 5, 1.,'lead2_tail.wav')], itertools.cycle)
    })

    # State 2 just plays lead3, lead4, lead3, lead4...
    s2 = DrawableLoopState('Two', {lSeq_chSustain, lSeq_bass, lSeq_drums,
        LoopSequence('lead',[
            Loop('lead3', 'lead3.wav'),
            Loop('lead4', 'lead4.wav')], itertools.cycle)
    })

    # State 3 plays lead5, lead6, lead7, lead7...
    s3 = DrawableLoopState('Three', {lSeq_chSustain, lSeq_bass, lSeq_drums, 
        LoopSequence('lead',[
            Loop('lead5', 'lead5.wav'),
            Loop('lead6', 'lead6.wav'),
            Loop('lead7', 'lead7.wav'),
            Loop('lead7', 'lead7.wav')], itertools.cycle)
    })

    s4 = DrawableLoopState('Four', { lSeq_bass, lSeq_drums})
    s5 = DrawableLoopState('Five', { lSeq_bass, LoopSequence('drums2', [Loop('drum2', 'drum2.wav')])})

    # Store all nodes in a list
    nodes = [s1, s2, s3, s4, s5]

    # Create the directed graph; all nodes connect and self connect
    G = nx.DiGraph()
    G.add_edges_from(itertools.product(nodes, nodes))

    # The advance function just returns a random neighbor
    def fnAdvance(SG):
        # Cartesian product
        def dot(A, B):
            return sum(a * b for a, b in itertools.zip_longest(A, B, fillvalue = 0))
        # Return the target of the edge out of activeState whose pathVec is most in line with stim
        return max(SG.G.out_edges_iter(SG.activeState, data = True), key = lambda edge : dot(SG.stim, edge[2]['pathVec']))[1]
    
    # Define the vectors between edges 
    # (used during dot product calculation, SG.stim is one of these)
    diEdges = {n : [1 if n == nn else 0 for nn in nodes] for n in nodes}
    for n in nodes:
        for nn in G.neighbors(n):
            G[n][nn]['pathVec'] = diEdges[nn]

    # Construct the StateGraph with an attribute 'stim' of s1's stimulus,
    # as well as a reference to the scene, so the states can use it
    SG = StateGraph(G, fnAdvance, s1, stim = diEdges[s1], cScene = cScene)

    # Init audio spec
    if cSM.Init({'freq' : 44100, 'channels' : 1, 'bufSize' : 4096}) == False:
        raise RuntimeError('Invalid aud config dict')

    # Voices (anything that makes sound) need a unique int ID
    voiceID = 0
    diLoopToVoiceID = dict()

    # get the samples per mS
    sampPerMS = int(cSM.GetSampleRate() / 1000)

    # For each loop in the state's loop sequences
    for loopState in nodes:
        # The state's trigger res is its longest loop
        loopState.triggerRes = 0
        for lSeq in loopState.diLoopSequences.values():
            for l in lSeq.loops:
                # Set tailfile to empty string if there is None
                if l.tailFile is None:
                    l.tailFile = ''
                # Add the loop
                strHeadFile = '../audio/' + l.headFile
                strTailFile = '../audio/' + l.tailFile
                if  cSM.RegisterClip(l.name, strHeadFile, strTailFile, int(sampPerMS * l.fadeMS)) == False:
                    raise IOError('Error: Failed to load audio file ' + l.name)

                # If successful, get a handle to c loop and store head/tail duration
                l.uNumHeadSamples = cSM.GetNumSamplesInClip(l.name, False)
                l.uNumTailSamples = cSM.GetNumSamplesInClip(l.name, True) - l.uNumHeadSamples

                # The state's trigger res is its longest loop
                if l.uNumHeadSamples > loopState.triggerRes:
                    loopState.triggerRes = l.uNumHeadSamples

                # Give this loop a voice ID and inc
                if l not in diLoopToVoiceID.keys():
                    diLoopToVoiceID[l] = voiceID
                    voiceID += 1
                l.voiceID = diLoopToVoiceID[l]

    # This dict maps the number keys to edge vectors defined in diEdges
    # (provided there are less than 10 nodes...)
    # the edge vectors are used during state advancement for the graph
    diKeyToStim = {SDLK.SDLK_1 + i : diEdges[n] for i, n in zip(range(len(nodes)), nodes)}

    # The stimulus update function assigns the stim
    # member of the stategraph based on what the
    # button maps to in the dict
    def fnStimKey(btn, keyMgr):
        nonlocal diKeyToStim
        nonlocal SG
        if btn.code in diKeyToStim.keys():
            SG.stim = diKeyToStim[btn.code]
    liButtons = [Button(k, None, fnStimKey) for k in diKeyToStim.keys()]

    arpClip = Loop('arp', 'arplead1.wav', 5, 1.,'arplead1.wav')
    arpClip.voiceID = voiceID + 1
    if cSM.RegisterClip(arpClip.name, '../audio/'+arpClip.headFile, '../audio/'+arpClip.tailFile, int(sampPerMS * arpClip.fadeMS)) == False:
        raise IOError(arpClip.name)

    # Hard coded test for now
    def fnOneShotKey(btn, keyMgR):
        nonlocal cSM
        nonlocal arpClip
        t = (arpClip.name, arpClip.voiceID, arpClip.vol, int(cSM.GetMaxSampleCount() / 4))
        pylSoundManager.SendMessage(cSM.c_ptr, (pylSoundManager.CMDOneShot, t))
    liButtons.append(Button(SDLK.SDLK_f, None, fnOneShotKey))

    # The escape key callback tells the scene to quit
    def fnEscapeKey(btn, keyMgr):
        nonlocal cScene
        cScene.SetQuitFlag(True)
    liButtons.append(Button(SDLK.SDLK_ESCAPE, None, fnEscapeKey))

    # The space key callback tells the loop manager to play/pause
    def fnSpaceKey(btn, keyMgr):
        nonlocal cSM
        cSM.PlayPause()
    liButtons.append(Button(SDLK.SDLK_SPACE, None, fnSpaceKey))

    # Create the input manager (no mouse manager needed)
    inputManager = InputManager(cScene, KeyboardManager(liButtons), MouseManager([]))

    # Create the sound manager
    loopManager = LoopManager(cScene, SG, inputManager)

    # Start the active loop seq
    activeState = loopManager.GetStateGraph().GetActiveState()
    messageList = [(pylSoundManager.CMDStartLoop, (l.name, l.voiceID, l.vol, 0)) for l in activeState.GetActiveLoopGen()]
    pylSoundManager.SendMessages(cSM(), messageList)

    # return the sound manager
    return loopManager

# Called by the C++ Scene class's
# constructor, inits Scene and
# the loop manager, adds drawables
def InitScene(pScene):
    #Construct the CScene
    cScene = pylScene.Scene(pScene)

    # Init the display 
    glVerMajor = 3
    glVerMinor = 0
    screenW = 800
    screenH = 800
    glBackgroundColor = [0.15, 0.15, 0.15, 1.]
    cScene.InitDisplay('pylCollisionAndSound', glVerMajor, glVerMinor, screenW, screenH, glBackgroundColor)

    # After GL context has started, create the Shader
    cShader = pylShader.Shader(cScene.GetShaderPtr())
    if cShader.Init('../shaders/simple.vert', '../shaders/simple.frag', True) == False:
        raise RuntimeError('Error: Shader source not loaded')

    # Get the position and color handles, set static drawable vars
    pylDrawable.SetPosHandle(cShader.GetHandle('a_Pos'))
    pylDrawable.SetColorHandle(cShader.GetHandle('u_Color'))
    
    # Set static PMV handle for camera
    pylCamera.SetCamMatHandle(cShader.GetHandle('u_PMV'))

    # Init camera (TODO give this screen dims or aspect ratio)
    cCamera = pylCamera.Camera(cScene.GetCameraPtr())
    cCamera.InitOrtho(-10., 10, -10., 10)
    
    # Create the loop manager (in LoopManager.py)
    # and give it an input manager
    loopManager = InitLoopManager(cScene)

    # Create the entities
    liEntities = InitEntities(cScene, loopManager)

    return Engine.Engine(liEntities, loopManager, cScene)