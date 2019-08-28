//
// Copyright (c) 2008-2017 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include <Urho3D/Scene/LogicComponent.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>

using namespace Urho3D;

//=============================================================================
//=============================================================================
enum CollisionLayer
{
    COLLAYER_STATIC         = (1 << 0),
    COLLAYER_BALL           = (1 << 1),
    COLLAYER_CAPSULE        = (1 << 2),
    COLLAYER_RAGDOLL        = (1 << 3),
    COLLAYER_RAGDOLL_ROOT   = (1 << 4)
};

enum CollisionMask
{
    COLMASK_STATIC          = ~(COLLAYER_STATIC),
    COLMASK_BALL            = ~(COLLAYER_RAGDOLL_ROOT),
    COLMASK_CAPSULE         = ~(COLLAYER_RAGDOLL | COLLAYER_RAGDOLL_ROOT),
    COLMASK_RAGDOLL         = ~(COLLAYER_CAPSULE | COLLAYER_RAGDOLL_ROOT),
    COLMASK_RAGDOLL_ROOT    = ~(COLLAYER_BALL | COLLAYER_CAPSULE | COLLAYER_RAGDOLL)
};

//=============================================================================
//=============================================================================
class CreateRagdoll : public LogicComponent
{
    URHO3D_OBJECT(CreateRagdoll, LogicComponent);
    
public:
    /// Construct.
    CreateRagdoll(Context* context);
    
    void SetEnableRagdoll(bool enable);
    RigidBody* GetRootBody() const { return rootBody_; }
    bool IsInRagdoll() const { return ragdollEabled_; } 

    void DebugDraw();

protected:
    /// Handle node being assigned.
    virtual void OnNodeSet(Node* node);
    void InitContraints();
    void EnableAnimation(bool enable);

    /// Handle scene node's physics collision.
    void HandleNodeCollision(StringHash eventType, VariantMap& eventData);
    /// Make a bone physical by adding RigidBody and CollisionShape components.
    void CreateRagdollBone(const String& boneName, ShapeType type, const Vector3& size, 
                           const Vector3& position, const Quaternion& rotation, float mass=1.0f);
    /// Join two bones with a Constraint component.
    void CreateRagdollConstraint(const String& boneName, const String& parentName, ConstraintType type, 
                                 const Vector3& axis, const Vector3& parentAxis, 
                                 const Vector2& highLimit, const Vector2& lowLimit, bool disableCollision = true);
protected:
    WeakPtr<RigidBody>      mainBody_;
    float                   mainBodyMass_;
    WeakPtr<RigidBody>      rootBody_;
    WeakPtr<Node>           swapNode_;

    bool                    ragdollEabled_;
    Vector<RigidBody*>      ragdollBodyList_;
    Timer                   timerRagdoll_;

};
