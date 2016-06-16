import pylContact

class Engine:
    def __init__(self, liEntities, soundManager, cScene):
        self.m_liEntities = liEntities
        self.m_SoundManager = soundManager
        self.m_cScene = cScene

    def Update(self): 
        liContacts = [pylContact.Contact(c) for c in self.m_cScene.GetContacts()]
        liCollidingContacts = [(c.GetBodyA(), c.GetBodyB()) for c in liCollidingContacts if c.IsColliding()]
        # Iterate over the colliding contacts 
        for c in liCollidingContacts:
            # Let entities know what happened
            self.m_liEntities[c[0].GetID()].HandleCollision(c)
            self.m_liEntities[c[1].GetID()].HandleCollision(c)
               
        # Update every entity in the engine
        for e in m_liEntities:
            e.Update()

        # Update the sound manager
        self.m_SoundManager.Update()

    # Starts SDL audio
    def Start(self):
        self.m_SoundManager.GetCSM().PlayPause()