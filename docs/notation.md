---
id: notation
title: Mathematical Notation
sidebar_label: Notation
description: Standard mathematical symbols and conventions used in robotics
keywords: [notation, mathematics, kinematics, transforms]
---

# Mathematical Notation

This guide uses standard robotics notation based on conventions from Siciliano & Khatib (2016) and Lynch & Park (2017).

---

## General Conventions

- **Scalars**: Lowercase italic letters (e.g., m for mass, t for time)
- **Vectors**: Lowercase bold letters (e.g., **v** for velocity, **p** for position)
- **Matrices**: Uppercase bold letters (e.g., **R** for rotation matrix, **J** for Jacobian)
- **Coordinate Frames**: Uppercase letters with subscripts (e.g., \{A\}, \{B\})

---

## Kinematics

| Symbol | Description | Units |
|--------|-------------|-------|
| q | Joint angle (revolute) or displacement (prismatic) | rad or m |
| q̇ | Joint velocity | rad/s or m/s |
| q̈ | Joint acceleration | rad/s² or m/s² |
| **q** | Joint configuration vector [q₁, q₂, ..., qₙ]ᵀ | - |
| n | Number of degrees of freedom (DOF) | - |

---

## Transforms and Poses

| Symbol | Description | Dimension |
|--------|-------------|-----------|
| **T** | Homogeneous transformation matrix | 4×4 |
| **R** | Rotation matrix (SO(3)) | 3×3 |
| **p** | Position vector | 3×1 |
| **T**_AB | Transform from frame B to frame A | 4×4 |

**Homogeneous Transform Structure**:
```
T = [ R   p ]
    [ 0   1 ]
```

---

## Jacobians and Velocities

| Symbol | Description | Dimension |
|--------|-------------|-----------|
| **J** | Geometric Jacobian matrix | 6×n |
| **v** | Linear velocity | 3×1 |
| **ω** | Angular velocity | 3×1 |
| **V** | Spatial velocity (twist) = [**v**, **ω**]ᵀ | 6×1 |

---

## Dynamics

| Symbol | Description | Units |
|--------|-------------|-------|
| m | Mass | kg |
| **I** | Inertia tensor | kg·m² |
| **M**(**q**) | Mass matrix | n×n |
| **C**(**q**, **q̇**) | Coriolis and centrifugal matrix | n×n |
| **g**(**q**) | Gravity vector | n×1 |
| **τ** | Joint torque vector | n×1 |

---

## Equations of Motion

### Lagrangian Dynamics

```
M(q)·q̈ + C(q, q̇)·q̇ + g(q) = τ
```

Where:
- **M**(**q**): Mass/inertia matrix (configuration-dependent)
- **C**(**q**, **q̇**): Coriolis and centrifugal forces
- **g**(**q**): Gravitational forces
- **τ**: Applied joint torques

### Jacobian Relationship

```
V = J(q)·q̇
```

Relates joint velocities (**q̇**) to end-effector spatial velocity (**V**).

---

## Coordinate Frame Conventions

- **Right-Hand Rule**: Coordinate frames follow right-hand convention (x-forward, y-left, z-up for humanoid robotics)
- **DH Parameters**: Denavit-Hartenberg notation for kinematic chains (covered in Module 1)
- **Quaternions**: Used for singularity-free rotation representation **q** = [q_w, q_x, q_y, q_z]ᵀ where ||**q**|| = 1

---

## Special Notations

### Subscripts and Superscripts

- **Subscripts**: Frame reference (e.g., **p**_A is position in frame A)
- **Superscripts**: Coordinate frame where vector is expressed (e.g., **p**^A is position expressed in frame A)
- **Leading Superscript**: Source frame in transforms (e.g., **T**^A_B transforms from B to A)

### Time Derivatives

- **Dot Notation**: q̇ = dq/dt (first derivative), q̈ = d²q/dt² (second derivative)
- **Partial Derivatives**: ∂f/∂x for multivariable functions

---

## Unit Conventions

- **Angles**: Radians (not degrees) unless explicitly noted
- **Length**: Meters (SI)
- **Mass**: Kilograms (SI)
- **Time**: Seconds (SI)
- **Force/Torque**: Newtons / Newton-meters (SI)

---

## Common Mathematical Operations

### Vector Operations

- **Dot Product**: **a** · **b** = |**a**| |**b**| cos(θ)
- **Cross Product**: **a** × **b** (produces vector perpendicular to both)
- **Norm**: ||**v**|| = √(v₁² + v₂² + v₃²)

### Matrix Operations

- **Transpose**: **A**ᵀ (rows become columns)
- **Inverse**: **A**⁻¹ (such that **A** · **A**⁻¹ = **I**)
- **Determinant**: det(**A**) or |**A**|

### Rotation Matrix Properties

- **Orthogonal**: **R**ᵀ · **R** = **I**
- **Determinant**: det(**R**) = 1 (proper rotation)
- **Inverse**: **R**⁻¹ = **R**ᵀ

---

## References

- Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.

---

**Note**: For full LaTeX mathematical expressions, please refer to the chapter-specific content where equations are presented with detailed derivations. This notation guide provides quick reference for symbol meanings and conventions.
