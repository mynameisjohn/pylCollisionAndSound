from pylRigidBody2D import RigidBody2D
from pylDrawable import Drawable
from pylEntComponent import EntComponent

class Entity:
    nEntsCreated = 0
    def __init__(self, cScene, colIdx, drIdx, cSound):
        self.colIdx = colIdx
        self.drIdx = drIdx
        self.c_Sound = cSound
        self.liCollisions = []

        Drawable(cScene.GetDrawable(self.drIdx)).SetID(Entity.nEntsCreated)
        RigidBody2D(cScene.GetRigidBody2D(self.colIdx)).SetID(Entity.nEntsCreated)

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