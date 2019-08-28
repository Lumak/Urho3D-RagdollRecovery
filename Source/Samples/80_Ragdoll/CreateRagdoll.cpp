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
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/IO/Log.h>

#include "CreateRagdoll.h"

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
static const float MinImpulse = 15.0f;
static const float MinRecoveryVelocity = 0.005f;
static const int MinRagdollTime = 2000;
//=============================================================================
//=============================================================================
CreateRagdoll::CreateRagdoll(Context* context)
    : LogicComponent(context)
    , ragdollEabled_(true)
    , mainBodyMass_(1.0f)
{
}

void CreateRagdoll::OnNodeSet(Node* node)
{
    // If the node pointer is non-null, this component has been created into a scene node. Subscribe to physics collisions that
    // concern this scene node
    if (node)
    {
        InitContraints();
    }
}

void CreateRagdoll::InitContraints()
{
    // get main rigidbody component
    mainBody_ = node_->GetComponent<RigidBody>();
    mainBodyMass_ = mainBody_->GetMass();
    SubscribeToEvent(node_, E_NODECOLLISION, URHO3D_HANDLER(CreateRagdoll, HandleNodeCollision));

    // create swap node
    swapNode_ = node_->CreateChild("SwapNode");
    SubscribeToEvent(swapNode_, E_NODECOLLISION, URHO3D_HANDLER(CreateRagdoll, HandleNodeCollision));

    //=====================
    // create rigidbodies 
    // *NOTE* don't change the order of creation for the first two rigid bodies
    //=====================
    CreateRagdollBone("", SHAPE_SPHERE, Vector3(0.2f, 0.2f, 0.2f), Vector3(0.0f, 0.0f, -0.2f),
        Quaternion(0.0f, 0.0f, 0.0f), 0.5f);

    CreateRagdollBone("Bip01_L_Foot", SHAPE_SPHERE, Vector3(0.1f, 0.1f, 0.1f), Vector3(0.0f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 0.0f), 0.01f);

    // Create RigidBody & CollisionShape components to bones
    CreateRagdollBone("Bip01_Pelvis", SHAPE_BOX, Vector3(0.3f, 0.2f, 0.25f), Vector3(0.0f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 0.0f));
    CreateRagdollBone("Bip01_Spine2", SHAPE_BOX, Vector3(0.35f, 0.2f, 0.3f), Vector3(0.0f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 0.0f));
    CreateRagdollBone("Bip01_L_Thigh", SHAPE_CAPSULE, Vector3(0.175f, 0.45f, 0.175f), Vector3(0.25f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_R_Thigh", SHAPE_CAPSULE, Vector3(0.175f, 0.45f, 0.175f), Vector3(0.25f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_L_Calf", SHAPE_CAPSULE, Vector3(0.15f, 0.55f, 0.15f), Vector3(0.25f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_R_Calf", SHAPE_CAPSULE, Vector3(0.15f, 0.55f, 0.15f), Vector3(0.25f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_Head", SHAPE_BOX, Vector3(0.2f, 0.2f, 0.2f), Vector3(0.1f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 0.0f));
    CreateRagdollBone("Bip01_L_UpperArm", SHAPE_CAPSULE, Vector3(0.15f, 0.35f, 0.15f), Vector3(0.1f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_R_UpperArm", SHAPE_CAPSULE, Vector3(0.15f, 0.35f, 0.15f), Vector3(0.1f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_L_Forearm", SHAPE_CAPSULE, Vector3(0.125f, 0.4f, 0.125f), Vector3(0.2f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));
    CreateRagdollBone("Bip01_R_Forearm", SHAPE_CAPSULE, Vector3(0.125f, 0.4f, 0.125f), Vector3(0.2f, 0.0f, 0.0f),
        Quaternion(0.0f, 0.0f, 90.0f));

    //=====================
    // create constraints
    //=====================
    Node *bipnode = node_->GetChild("Bip01_L_Foot", true);
    float yang = bipnode->GetWorldRotation().YawAngle();
    Quaternion qnorm(-yang, Vector3(0,1,0));

    // *NOTE* CONSTRAIN the main body to the root body to move the main node_ with the ragdoll
    Constraint* constraint = node_->CreateComponent<Constraint>();
    constraint->SetConstraintType(CONSTRAINT_POINT);
    constraint->SetDisableCollision(true);
    constraint->SetOtherBody(rootBody_);
    constraint->SetWorldPosition(node_->GetWorldPosition());
    constraint->SetAxis(Vector3::UP);
    constraint->SetOtherAxis(Vector3::UP);
    constraint->SetHighLimit(Vector2::ZERO);
    constraint->SetLowLimit(Vector2::ZERO);
    constraint->SetERP(0.2f);

    CreateRagdollConstraint("", "Bip01_L_Foot", CONSTRAINT_CONETWIST, qnorm * Vector3::RIGHT, Vector3::RIGHT,
        Vector2(0.0f, 50.0f), Vector2::ZERO);

    CreateRagdollConstraint("Bip01_L_Foot", "Bip01_L_Calf", CONSTRAINT_HINGE, Vector3::UP, Vector3::FORWARD,
        Vector2(0.0f, 0.0f), Vector2(0.01f, 0.0f));

    // Create Constraints between bones
    CreateRagdollConstraint("Bip01_Spine2", "Bip01_Pelvis", CONSTRAINT_HINGE, Vector3::FORWARD, Vector3::FORWARD,
        Vector2(45.0f, 0.0f), Vector2(-10.0f, 0.0f));
    CreateRagdollConstraint("Bip01_L_Thigh", "Bip01_Pelvis", CONSTRAINT_CONETWIST, Vector3::BACK, Vector3::FORWARD,
        Vector2(45.0f, 45.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_R_Thigh", "Bip01_Pelvis", CONSTRAINT_CONETWIST, Vector3::BACK, Vector3::FORWARD,
        Vector2(45.0f, 45.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_L_Calf", "Bip01_L_Thigh", CONSTRAINT_HINGE, Vector3::BACK, Vector3::BACK,
        Vector2(90.0f, 0.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_R_Calf", "Bip01_R_Thigh", CONSTRAINT_HINGE, Vector3::BACK, Vector3::BACK,
        Vector2(90.0f, 0.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_Head", "Bip01_Spine2", CONSTRAINT_CONETWIST, Vector3::LEFT, Vector3::LEFT,
        Vector2(0.0f, 30.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_L_UpperArm", "Bip01_Spine2", CONSTRAINT_CONETWIST, Vector3::DOWN, Vector3::UP,
        Vector2(45.0f, 45.0f), Vector2::ZERO, false);
    CreateRagdollConstraint("Bip01_R_UpperArm", "Bip01_Spine2", CONSTRAINT_CONETWIST, Vector3::DOWN, Vector3::UP,
        Vector2(45.0f, 45.0f), Vector2::ZERO, false);
    CreateRagdollConstraint("Bip01_L_Forearm", "Bip01_L_UpperArm", CONSTRAINT_HINGE, Vector3::BACK, Vector3::BACK,
        Vector2(90.0f, 0.0f), Vector2::ZERO);
    CreateRagdollConstraint("Bip01_R_Forearm", "Bip01_R_UpperArm", CONSTRAINT_HINGE, Vector3::BACK, Vector3::BACK,
        Vector2(90.0f, 0.0f), Vector2::ZERO);

    // enable animation
    SetEnableRagdoll(false);
}

void CreateRagdoll::SetEnableRagdoll(bool enable)
{
    if (ragdollEabled_ == enable)
    {
        return;
    }

    ragdollEabled_ = enable;
    bool setAsTrigger = !ragdollEabled_;

    // change main body collision layer
    if (mainBody_)
    {
        mainBody_->SetMass(ragdollEabled_ ? M_LARGE_EPSILON : mainBodyMass_);
        mainBody_->SetCollisionLayer(ragdollEabled_ ? 0 : COLLAYER_CAPSULE);
    }
    
    // change ragdoll bodies
    for ( unsigned i = 0; i < ragdollBodyList_.Size(); ++i )
    {
        ragdollBodyList_[i]->SetTrigger(setAsTrigger);
        float mass = 1.0f;
        if (i == 0)
        {
            mass = 0.5f;
        }
        else if (i == 1)
        {
            mass = 0.01f;
        }
        ragdollBodyList_[i]->SetMass(setAsTrigger ? 0.0f : mass);
    }

    EnableAnimation(setAsTrigger);
}

void CreateRagdoll::EnableAnimation(bool enable)
{
    // disable keyframe animation from all bones so that they will not interfere with the ragdoll
    AnimatedModel* model = node_->GetComponent<AnimatedModel>(true);
    Skeleton& skeleton = model->GetSkeleton();
    for (unsigned i = 0; i < skeleton.GetNumBones(); ++i)
    {
        skeleton.GetBone(i)->animated_ = enable;
    }

    AnimationController *animCtrl = node_->GetComponent<AnimationController>(true);
    AnimatedModel *animatedModel = node_->GetComponent<AnimatedModel>(true);
    if (animCtrl)
    {
        if (enable)
        {
            animatedModel->SetRagdollRecovery(true);
            animCtrl->PlayExclusive("Models/Jack_Walk.ani", 0, true, 0.4f);
            animCtrl->SetRagdollRecovery("Models/Jack_Walk.ani", 0.4f);
        }
        else
        {
            animatedModel->SetRagdollRecovery(false);
        }
    }
}

void CreateRagdoll::CreateRagdollBone(const String& boneName, ShapeType type, const Vector3& size, const Vector3& position,
    const Quaternion& rotation, float mass)
{
    // Find the correct child scene node recursively
    Node* boneNode = !boneName.Empty()?node_->GetChild(boneName, true):swapNode_;
    if (!boneNode)
    {
        URHO3D_LOGWARNING("Could not find bone " + boneName + " for creating ragdoll physics components");
        return;
    }

    RigidBody* body = boneNode->CreateComponent<RigidBody>();
    if (!boneName.Empty())
    {
        body->SetCollisionLayerAndMask(COLLAYER_RAGDOLL, COLMASK_RAGDOLL);
    }
    else
    {
        rootBody_ = body;
        // maintain upright orientation (same as the main capsule)
        body->SetAngularFactor(Vector3(0,1,0));
        body->SetCollisionLayerAndMask(COLLAYER_RAGDOLL_ROOT, COLMASK_RAGDOLL_ROOT);
    }

    // save rigidbody
    ragdollBodyList_.Push(body);

    // Set mass to make movable
    body->SetMass(mass);
    // Set damping parameters to smooth out the motion
    body->SetLinearDamping(0.05f);
    body->SetAngularDamping(0.85f);
    // Set rest thresholds to ensure the ragdoll rigid bodies come to rest to not consume CPU endlessly
    body->SetLinearRestThreshold(1.5f);
    body->SetAngularRestThreshold(2.5f);

    CollisionShape* shape = boneNode->CreateComponent<CollisionShape>();
    // We use either a box or a capsule shape for all of the bones
    if (type == SHAPE_BOX)
    {
        shape->SetBox(size, position, rotation);
    }
    else if (type == SHAPE_SPHERE)
    {
        shape->SetSphere(size.x_, position, rotation);
    } 
    else
    {
        shape->SetCapsule(size.x_, size.y_, position, rotation);
    }
}

void CreateRagdoll::CreateRagdollConstraint(const String& boneName, const String& parentName, ConstraintType type,
    const Vector3& axis, const Vector3& parentAxis, const Vector2& highLimit, const Vector2& lowLimit,
    bool disableCollision)
{
    Node* boneNode = !boneName.Empty()?node_->GetChild(boneName, true):swapNode_;
    Node* parentNode = !parentName.Empty()?node_->GetChild(parentName, true):node_;
    if (!boneNode)
    {
        URHO3D_LOGWARNING("Could not find bone " + boneName + " for creating ragdoll constraint");
        return;
    }
    if (!parentNode)
    {
        URHO3D_LOGWARNING("Could not find bone " + parentName + " for creating ragdoll constraint");
        return;
    }

    Constraint* constraint = boneNode->CreateComponent<Constraint>();
    constraint->SetConstraintType(type);
    // Most of the constraints in the ragdoll will work better when the connected bodies don't collide against each other
    constraint->SetDisableCollision(disableCollision);
    // The connected body must be specified before setting the world position
    constraint->SetOtherBody(parentNode->GetComponent<RigidBody>());
    // Position the constraint at the child bone we are connecting
    constraint->SetWorldPosition(boneNode->GetWorldPosition());
    // Configure axes and limits
    constraint->SetAxis(axis);
    constraint->SetOtherAxis(parentAxis);
    constraint->SetHighLimit(highLimit);
    constraint->SetLowLimit(lowLimit);
    constraint->SetERP(0.05f);
}

void CreateRagdoll::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;

    RigidBody *body = static_cast<RigidBody*>(eventData[P_BODY].GetPtr());

    if (body == mainBody_)
    {
        if (!IsInRagdoll())
        {
            SetEnableRagdoll(true);
            timerRagdoll_.Reset();
        }
    }
    else if (body == rootBody_)
    {
        if (IsInRagdoll() && rootBody_->GetLinearVelocity().LengthSquared() < MinRecoveryVelocity)
        {
            if (timerRagdoll_.GetMSec(false) < MinRagdollTime)
            {
                return;
            }
            SetEnableRagdoll(false);
        }
    }
}

void CreateRagdoll::DebugDraw()
{
    DebugRenderer *dbgRenderer = node_->GetScene()->GetComponent<DebugRenderer>();
    Node *boneNode = node_->GetChild("Bip01_L_Foot", true);
    Vector3 pos = boneNode->GetWorldPosition();

    Constraint* constraint = boneNode->GetComponent<Constraint>();
    constraint->DrawDebugGeometry(dbgRenderer, false);

}

