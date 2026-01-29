# Technical Report: Arcade Vehicle Physics System (Unreal Engine)

This project features a **custom-built arcade vehicle physics system** in Unreal Engine.

Initially, the plan was to write the entire vehicle physics engine from scratch using raw force applications. However, I pivoted to a **hybrid approach** after recognizing the immense complexity of recreating stable wheel-ground friction and suspension constraints from zero.

Instead, I used the **UE Chaos Wheeled Vehicle** framework as a robust base for collision + basic movement, then built a comprehensive layer of **custom arcade logic** on top to override the default “simulation” feel.

---

## Table of Contents
- [Video Showcase](#video-showcase)
- [Design Choices](#design-choices)
- [Implementation Highlights](#implementation-highlights)
  - [Hybrid Drift & Lateral Friction](#hybrid-drift--lateral-friction)
  - [Nitro Boost Mechanics](#nitro-boost-mechanics)
  - [Dynamic Camera Effects](#dynamic-camera-effects)
  - [Custom Air Control & Stability](#custom-air-control--stability)
  - [Slip Angle Calculation](#slip-angle-calculation)
  - [Dynamic Grip Management](#dynamic-grip-management)
  - [Visual Wheel Counter-Steer](#visual-wheel-counter-steer)
  - [Launch Boost Logic](#launch-boost-logic)
  - [Artificial Downforce](#artificial-downforce)
- [Key Takeaways](#key-takeaways)

---

## Video Showcase

https://github.com/user-attachments/assets/71f21872-efc8-4081-a666-a1fe7a4d59fb


---

## Design Choices

### Hybrid Physics Approach
By combining the `UChaosWheeledVehicleMovementComponent` with manual force applications, the system achieves:
- **Reliable stability** (collision, suspension, contact constraints)
- **Arcade responsiveness** (custom lateral grip, drift, downforce, boosts)

### State-Driven Handling
Vehicle behavior is driven by explicit gameplay states:
- `bIsDrifting`
- `bIsAirborne`
- `bIsNitroActive`
- `bIsHandbraking`

This makes handling feel like distinct “modes” while still running in the same pawn.

### Visual-Physical Decoupling
Wheel **visual rotation and steering** are driven separately from physics wheels.  
This enables exaggerated counter-steer during drifts for a stronger arcade aesthetic **without breaking** the underlying simulation.

### Dynamic FOV & Camera Swing
To convey speed and inertia, the camera uses a spring arm that dynamically adjusts:
- **Arm length** (speed, reverse state)
- **Yaw swing** (slip angle + drift state)
- (Optionally) **FOV** for speed emphasis

---

## Implementation Highlights

### Hybrid Drift & Lateral Friction
The system calculates slip angle and dynamically adjusts lateral friction for controlled slides.

```cpp
// ArcadeCar.cpp: Applying custom lateral friction logic
float FrictionAmount;
if (bIsHandbraking) {
    FrictionAmount = DriftSideFriction * 0.3f; // Significant grip loss for initiation
}
else if (bIsDrifting) {
    float ThrottleDriftBonus = FMath::Lerp(1.0f, 0.4f, SmoothedThrottleInput);
    FrictionAmount = DriftSideFriction * ThrottleDriftBonus;
}
else {
    FrictionAmount = NormalSideFriction; // Standard high-grip state
}

// Apply counter-acceleration to simulate side grip
PhysicsRoot->AddForce(
    FlatRightVec * FMath::Clamp(-SideSpeed * FrictionAmount, -MaxGripAccel, MaxGripAccel),
    NAME_None,
    true
);
```

### Nitro Boost Mechanics
Nitro provides both **torque scaling** and a **direct forward force** (with a soft speed cap).

```cpp
// ArcadeCar.cpp: Nitro torque and force application
Vehicle->EngineSetup.MaxTorque = BaseEngineTorque * NitroTorqueMultiplier;

if (NitroBoostForce > 0.f && !bIsAirborne) {
    float SpeedCapFactor = 1.f;
    if (NitroMaxSpeed > 0.f && AbsSpeed > NitroMaxSpeed * 0.9f) {
        SpeedCapFactor = FMath::Max(
            0.f,
            1.f - ((AbsSpeed - NitroMaxSpeed * 0.9f) / (NitroMaxSpeed * 0.1f))
        );
    }

    FVector BoostForce = GetActorForwardVector() * NitroBoostForce * SpeedCapFactor * DeltaTime;
    PhysicsRoot->AddForce(BoostForce, NAME_None, true);
}
```

### Dynamic Camera Effects
Spring-arm reacts to movement: distance changes (including reverse) and yaw swing during drifts.

```cpp
// ArcadeCar.cpp: Camera swing logic based on slip angle
if (bIsDrifting) {
    float SlipRatio = FMath::Clamp(CurrentSlipAngle / 45.f, -1.f, 1.f);
    TargetYawOffset = SlipRatio * CameraDriftSwingAngle;
}

CameraArm->TargetArmLength = FMath::FInterpTo(
    CameraArm->TargetArmLength,
    TargetDistance,
    DeltaTime,
    2.0f
);

float NewYaw = FMath::FInterpTo(CurrentRot.Yaw, TargetYawOffset, DeltaTime, CameraSwingSpeed);
CameraArm->SetRelativeRotation(FRotator(-15.f, NewYaw, 0.f));
```

### Custom Air Control & Stability
While airborne, manual air control and extra gravity keep the car controllable and snappy.

```cpp
// ArcadeCar.cpp: Airborne pitch and yaw control
if (bIsAirborne) {
    FVector ExtraGravity = FVector(0, 0, -980.f * 2.0f) * PhysicsRoot->GetMass();
    PhysicsRoot->AddForce(ExtraGravity);

    FVector AngularVelocity = PhysicsRoot->GetPhysicsAngularVelocityInDegrees();
    AngularVelocity.Z += SteerInput * AirControlStrength * 180.f * DeltaTime;
    AngularVelocity.Y -= SmoothedThrottleInput * AirControlStrength * 60.f * DeltaTime;
    PhysicsRoot->SetPhysicsAngularVelocityInDegrees(AngularVelocity);
}
```

### Slip Angle Calculation
Utility function measuring how much the car is sliding relative to forward direction.

```cpp
// ArcadeCar.cpp: Calculating the angle between velocity and forward vector
Velocity.Z = 0.f;
Forward.Z = 0.f;

float Speed = Velocity.Size();
if (Speed < MinSpeedForSlip) return 0.f;

OutVelocityDir = Velocity / Speed;
OutForwardDir  = Forward.GetSafeNormal();

float Dot = FMath::Clamp(FVector::DotProduct(OutForwardDir, OutVelocityDir), -1.f, 1.f);
float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));

FVector Cross = FVector::CrossProduct(OutForwardDir, OutVelocityDir);
return (Cross.Z < 0.f) ? -AngleDeg : AngleDeg;
```

### Dynamic Grip Management
Front/rear friction multipliers adjust in real-time to support drifting behaviors.

```cpp
// ArcadeCar.cpp: Real-time friction adjustments
if (bIsHandbraking) {
    FrontGrip = 2.5f;
    RearGrip  = 0.1f; // Force the rear to slide
} else if (bIsDrifting) {
    FrontGrip = 2.0f;
    RearGrip  = 0.5f; // Maintain drift
}

for (int32 i = 0; i < Vehicle->Wheels.Num(); i++) {
    if (UChaosVehicleWheel* Wheel = Vehicle->Wheels[i]) {
        Wheel->FrictionForceMultiplier = (i < 2) ? FrontGrip : RearGrip;
    }
}
```

### Visual Wheel Counter-Steer
Wheel meshes counter-steer visually while drifting (decoupled from physics steering).

```cpp
// ArcadeCar.cpp: Decoupling visual rotation from physics steering
float SteerAngle = PhysWheel->GetSteerAngle();
if (bIsDrifting) {
    SteerAngle -= CurrentSlipAngle * 1.5f; // Counter-steer visual
    SteerAngle = FMath::Clamp(SteerAngle, -MaxSteerAngle, MaxSteerAngle);
}

WheelMeshes[i]->SetRelativeLocation(NewPos);
WheelMeshes[i]->SetRelativeRotation(FRotator(CurrentRot.Pitch + DeltaSpin, SteerAngle, 0.f));
```

### Launch Boost Logic
Low-speed boost ensures punchy initial acceleration.

```cpp
// ArcadeCar.cpp: Low-speed acceleration boost
if (FMath::Abs(SpeedKMH) < LaunchBoostMaxSpeed && ThrottleInput > 0.5f && !bIsAirborne) {
    float SpeedRatio = FMath::Abs(SpeedKMH) / LaunchBoostMaxSpeed;
    float BoostFactor = FMath::Lerp(LaunchBoostMultiplier, 1.f, SpeedRatio);

    FVector BoostForce = GetActorForwardVector() * (BoostFactor - 1.f) * 15000.f * ThrottleInput;
    PhysicsRoot->AddForce(BoostForce);
}
```

### Artificial Downforce
Speed-dependent downforce improves stability at high velocity.

```cpp
// ArcadeCar.cpp: Quadratic downforce application
float SpeedOver = AbsSpeed - DownforceStartSpeed;
if (SpeedOver > 0.f) {
    float DownforceAmount = DownforceCoefficient * SpeedOver * SpeedOver * 0.5f;
    PhysicsRoot->AddForce(FVector(0.f, 0.f, -DownforceAmount));
}
```

---

## Key Takeaways

- **Hybrid is pragmatic:** Chaos provides stability; custom forces provide arcade feel.
- **States simplify tuning:** drift, air, and nitro “modes” keep behavior readable and controllable.
- **Decoupled visuals matter:** counter-steer and exaggerated wheel motion sell the arcade fantasy.
- **Camera sells speed:** slip-based yaw swing and dynamic arm length add instant “feel”.

---

