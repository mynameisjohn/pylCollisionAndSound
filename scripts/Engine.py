from pylContact import Contact
from pylRigidBody2D import RigidBody2D

class Engine:
    # Construct with a list of entities, the sound manager,
    # and the wrapped C++ scene
    def __init__(self, liEntities, soundManager, cScene):
        self.m_liEntities = liEntities
        self.m_SoundManager = soundManager
        self.m_cScene = cScene

    def Update(self): 
        # Call the C++ scene's update function
        self.m_cScene.Update()

        # Get all colliding objects
        liContacts = [Contact(c) for c in self.m_cScene.GetContacts()]
        liCollidingContacts = [(RigidBody2D(c.GetBodyA()), RigidBody2D(c.GetBodyB())) for c in liContacts if c.IsColliding()]
        
        # Iterate over the colliding contacts 
        for c in liCollidingContacts:
            id1 = c[0].GetID()
            id2 = c[1].GetID()
            # walls have negative IDs
            if id1 >= 0 and id2 >= 0:
                # Let entities know what happened
                self.m_liEntities[id1].HandleCollision(c)
                self.m_liEntities[id2].HandleCollision(c)
               
        # Update every entity in the engine
        for e in self.m_liEntities:
            e.Update()

        # Find the entity with the most collisions, set the
        # loop graph's active stim to be the input stim of the node
        # (so it is queued to play when the loop graph updates)
        eMax = max(self.m_liEntities, key = lambda e : e.colCount)
        self.m_SoundManager.GetStateGraph().stim = eMax.soundNode.inputStim

        # Update the sound manager
        self.m_SoundManager.Update(self)

        # draw objects in the scene
        self.m_cScene.Draw()

    # Clear each entity's collision counter
    def ClearColCount(self):
        for e in self.m_liEntities:
            e.colCount = 0

    # Toggle the play state of the sound manager
    def StartStop(self, bStart):
        self.m_SoundManager.PlayPause(bStart)

    def HandleEvent(self, pSdlEvent):
        # For now delegate to the sound manager handler
        self.m_SoundManager.HandleEvent(pSdlEvent)