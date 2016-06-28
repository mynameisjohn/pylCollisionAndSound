from pylRigidBody2D import RigidBody2D
from pylDrawable import Drawable
from pylEntComponent import EntComponent

class Entity:
    nEntsCreated = 0
    def __init__(self, cScene, colIdx, drIdx, cSound):
        self.cScene = cScene
        self.colIdx = colIdx
        self.drIdx = drIdx
        self.soundComp = cSound
        self.liCollisions = []

        self.GetDrawableComponent().SetID(Entity.nEntsCreated)
        self.GetCollisionComponent().SetID(Entity.nEntsCreated)
        self.soundComp.SetDrawableIdx(self.drIdx)

        Entity.nEntsCreated += 1

    def GetDrawableComponent(self):
        return Drawable(self.cScene.GetDrawable(self.drIdx))
        
    def GetCollisionComponent(self):
        return RigidBody2D(self.cScene.GetRigidBody2D(self.colIdx))

    def Update(self):
        # If colliding, update collision count
        #for c in self.liCollisions:
            # Update sound state collision count
            # In the future I might want to know what it's
            # colliding with, so iterating over contacts
            # may be a better choice
            #cSound.HandleCollision(c)
        # Update drawable transform
        self.GetDrawableComponent().SetTransform(self.GetCollisionComponent().GetQuatVec())
        # Clear collision list
        self.liCollisions.clear()

    def HandleCollision(self, eOther):
        #print(self, 'is colliding with', eOther)
        self.liCollisions.append(eOther)     