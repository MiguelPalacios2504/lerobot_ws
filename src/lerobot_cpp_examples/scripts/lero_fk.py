import numpy as np

# ---- Parámetros DH (unidades en mm, ángulos en rad) ----
L0  = 54.2
L1  = 30.5
L2  = 116.1
L3  = 134.5
L45 = 160.0  # L4 + L5

# Fila i: (theta_i, d_i, a_i, alpha_i)
# Nota: para la fila 4 aplicamos el corrimiento theta4 + pi/2 y alpha4 = -pi/2
def dh_table(theta1, theta2, theta3, theta4, theta5):
    return np.array([
        [theta1 + 0.0,  L0,  L1,   np.pi/2],  # 1: teta1, d=L0, a=L1, alpha=+pi/2
        [theta2 + 0.0,       0,  L2,   0.0     ],  # 2: teta2, d=0,  a=L2, alpha=0
        [theta3 + 0.0,       0,  L3,   0.0     ],  # 3: teta3, d=0,  a=L3, alpha=0
        [theta4 - np.pi/2,   0,   0,  -np.pi/2 ],  # 4: teta4+pi/2, d=0, a=0, alpha=-pi/2
        [theta5 + 0.0,     L45,   0,   0.0     ],  # 5: teta5, d=L4+L5, a=0, alpha=0
    ], dtype=float)

def T_dh(theta, d, a, alpha):
    """Matriz de transformación homogénea con DH estándar."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct ],
        [ st,  ct*ca, -ct*sa, a*st ],
        [  0,     sa,     ca,    d ],
        [  0,      0,      0,    1 ]
    ], dtype=float)

def fk_5dof(theta1, theta2, theta3, theta4, theta5):
    """Cinemática directa: retorna T_05 y (x,y,z) en mm."""
    dh = dh_table(theta1, theta2, theta3, theta4, theta5)
    T = np.eye(4)
    for i in range(5):
        T = T @ T_dh(dh[i,0], dh[i,1], dh[i,2], dh[i,3])
    p = T[:3, 3]
    return T, p

# ---- Ejemplo de uso ----
if __name__ == "__main__":
    # ángulos de prueba (rad)
    q_deg = [90, 90, 0, 0, 0]
    q_rad = np.deg2rad(q_deg)
    
    print("=== CINEMÁTICA DIRECTA ===")
    print(f"Ángulos de entrada:")
    print(f"  En grados: {q_deg}")
    print(f"  En radianes: {np.round(q_rad, 4)}")
    print()
    
    T05, pos = fk_5dof(*q_rad)
    
    print("Matriz de transformación homogénea T_0_5:")
    print(np.round(T05, 4))
    print()
    print(f"Posición del efector final (x, y, z):")
    print(f"  X: {pos[0]:.1f} mm")
    print(f"  Y: {pos[1]:.1f} mm") 
    print(f"  Z: {pos[2]:.1f} mm")
    print()
    
    # Cálculo adicional: orientación en ángulos de Euler
    # Extraer ángulos de Euler ZYX de la matriz de rotación
    R = T05[:3, :3]
    
    # Yaw (ψ), Pitch (θ), Roll (φ) - convención ZYX
    if abs(R[2, 0]) < 0.9999:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        # Caso singular (gimbal lock)
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arcsin(-R[2, 0])
        roll = 0
    
    print("Orientación (ángulos de Euler ZYX):")
    print(f"  Yaw (ψ): {np.rad2deg(yaw):.1f}°")
    print(f"  Pitch (θ): {np.rad2deg(pitch):.1f}°")
    print(f"  Roll (φ): {np.rad2deg(roll):.1f}°")