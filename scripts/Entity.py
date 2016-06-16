import pylRigidBody2D
import pylDrawable

class Entity:
    nEntsCreated = 0
    def __init__(self, cScene, cCol, cDr, cSound):
        self.c_Col = cCol
        self.c_Dr = cDr
        self.c_Sound = cSound
        self.liCollisions = []

        cRB = pylRigidBody2D.EntComponent(cScene.GetRigidBody2D(cCol))
        cdrawable = pylDrawable.EntComponent(cScene.GetDrawable(cDr))

        for eComp in [cRB, cdrawable]:
            eComp.SetID(Entity.nEntsCreated)

        Entity.nEntsCreated += 1
        
    def Update(self):
        # If colliding, update collision count
        for c in self.liCollisions:
            # Update sound state collision count
            # In the future I might want to know what it's
            # colliding with, so iterating over contacts
            # may be a better choice
            cSound.HandleCollision(c)
        # Update drawable transform
        self.c_Dr.SetTransform(self.c_Col.GetTransform())
        # Clear collision list
        self.liCollisions.clear()

    def HandleCollision(self, eOther):
        self.liCollisions.append(self.eOther)     