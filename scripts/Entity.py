from pylRigidBody2D import RigidBody2D
from pylDrawable import Drawable
from pylEntComponent import EntComponent

class Entity:
    nEntsCreated = 0
    def __init__(self, cScene, colIdx, drIdx, cSound):
        # Cache the scene object, our collision and drawable indices,
        # and our node in the loop graph 
        self.cScene = cScene
        self.colIdx = colIdx
        self.drIdx = drIdx
        self.soundNode = cSound
        self.liCollisions = []
        self.colCount = 0

        # Store our unique ent ID (a class variable)
        # and set the C++ component ID accodingly
        self.ID = Entity.nEntsCreated
        self.GetDrawableComponent().SetID(Entity.nEntsCreated)
        self.GetCollisionComponent().SetID(Entity.nEntsCreated)

        # Give the sound component access to a drawable
        # (this could use a redesign)
        self.soundNode.SetDrawableIdx(self.drIdx)

        # Ensure each ent has its own ID
        Entity.nEntsCreated += 1

    def GetDrawableComponent(self):
        return Drawable(self.cScene.GetDrawable(self.drIdx))
        
    def GetCollisionComponent(self):
        return RigidBody2D(self.cScene.GetRigidBody2D(self.colIdx))

    def Update(self):
        # Update drawable transform
        self.GetDrawableComponent().SetTransform(self.GetCollisionComponent().GetQuatVec())
        # Add collisions and clear collision list
        self.colCount += len(self.liCollisions)
        self.liCollisions.clear()

    # Just make a note that this happened,
    # this list is periodically cleared (above)
    def HandleCollision(self, eOther):
        self.liCollisions.append(eOther)     