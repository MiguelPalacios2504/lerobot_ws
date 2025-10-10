import numpy as np
import math

# ---- Parámetros DH (unidades en mm, ángulos en rad) ----
L0  = 54.2
L1  = 30.5
L2  = 116.1
L3  = 134.5
L45 = 160.0  # L4 + L5

class Robot5DOFIK:
    def __init__(self):
        self.L0 = L0
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L45 = L45
        
        # Límites de articulación (en radianes)
        self.joint_limits = [
            (-np.pi/2, np.pi/2),    # theta1
            (-np.pi/8, np.pi),      # theta2  
            (-np.pi, 0),            # theta3
            (-np.pi/2, np.pi/2),    # theta4
            (-np.pi/2, np.pi/2)     # theta5
        ]
    
    def dh_table(self, theta1, theta2, theta3, theta4, theta5):
        """Tabla DH para el robot de 5 DOF"""
        return np.array([
            [theta1 + 0.0,      self.L0, self.L1,  np.pi/2],  # 1
            [theta2 + 0.0,           0,  self.L2,  0.0     ],  # 2
            [theta3 + 0.0,           0,  self.L3,  0.0     ],  # 3
            [theta4 - np.pi/2,       0,       0,  -np.pi/2 ],  # 4
            [theta5 + 0.0,      self.L45,     0,   0.0     ],  # 5
        ], dtype=float)
    
    def T_dh(self, theta, d, a, alpha):
        """Matriz de transformación homogénea con DH estándar"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ], dtype=float)
    
    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5):
        """Cinemática directa"""
        dh = self.dh_table(theta1, theta2, theta3, theta4, theta5)
        T = np.eye(4)
        for i in range(5):
            T = T @ self.T_dh(dh[i,0], dh[i,1], dh[i,2], dh[i,3])
        p = T[:3, 3]
        return T, p

    def compute_analytical_ik(self, target_position, target_orientation=None, verbose=False):
        """
        Cinemática inversa analítica para robot 5 DOF
        
        Args:
            target_position: [x, y, z] en mm
            target_orientation: matriz 3x3 o ángulos de Euler (opcional)
            
        Returns:
            Lista de soluciones válidas [theta1, theta2, theta3, theta4, theta5]
        """
        x, y, z = target_position
        
        solutions = []
        
        # --- Solución para theta1 ---
        theta1 = math.atan2(y, x)
        
        # Verificar límites de theta1
        if not (self.joint_limits[0][0] <= theta1 <= self.joint_limits[0][1]):
            if verbose:
                print(f"Theta1 {np.rad2deg(theta1):.1f}° fuera de límites")
            return []
        
        # --- Cálculo de posición del wrist center ---
        # Para un robot 5DOF, el wrist center está en la articulación 5 (simplificado)
        # Asumimos orientación por defecto (z apuntando hacia abajo/arriba según configuración)
        if target_orientation is not None:
            if isinstance(target_orientation, np.ndarray) and target_orientation.shape == (3,3):
                R = target_orientation
            else:
                R = np.eye(3)
            
            # Vector de aproximación (z-axis del efector final)
            approach_vector = R[:, 2]
            wrist_center = target_position - self.L45 * approach_vector
        else:
            # Sin orientación específica, asumir configuración por defecto
            # Para las pruebas que has mostrado, parece que z apunta hacia arriba
            wrist_center = np.array([x, y, z])
        
        # --- Cálculo de theta2 y theta3 ---
        # Coordenadas relativas al shoulder (articulación 2)
        x_rel = math.sqrt(wrist_center[0]**2 + wrist_center[1]**2) - self.L1
        z_rel = wrist_center[2] - self.L0
        
        if verbose:
            print(f"Posición relativa: x={x_rel:.1f}, z={z_rel:.1f}")
        
        # Distancia al wrist center
        D = math.sqrt(x_rel**2 + z_rel**2)
        
        # Verificar alcanzabilidad
        if D > (self.L2 + self.L3) or D < abs(self.L2 - self.L3):
            if verbose:
                print(f"Posición no alcanzable: D={D:.1f}, L2+L3={self.L2+self.L3:.1f}")
            return []
        
        # Ley de cosenos para theta3
        cos_theta3 = (D**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
        
        theta3_1 = math.acos(cos_theta3)
        theta3_2 = -theta3_1
        
        # Para cada theta3, calcular theta2
        for theta3 in [theta3_1, theta3_2]:
            # Calcular theta2
            alpha = math.atan2(z_rel, x_rel)
            beta = math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))
            theta2 = alpha - beta
            
            # Verificar límites de theta2 y theta3
            if not (self.joint_limits[1][0] <= theta2 <= self.joint_limits[1][1]):
                continue
            if not (self.joint_limits[2][0] <= theta3 <= self.joint_limits[2][1]):
                continue
            
            # --- Cálculo de theta4 y theta5 ---
            # Para un robot 5DOF, simplificamos la orientación final
            # theta4 compensa la orientación del plano formado por theta2 y theta3
            theta4 = -theta2 - theta3
            
            # theta5 controla la rotación final (gripper)
            theta5 = 0.0  # Por defecto
            
            # Verificar límites de theta4 y theta5
            if not (self.joint_limits[3][0] <= theta4 <= self.joint_limits[3][1]):
                continue
            if not (self.joint_limits[4][0] <= theta5 <= self.joint_limits[4][1]):
                continue
            
            solution = [theta1, theta2, theta3, theta4, theta5]
            
            # Verificar la solución con cinemática directa
            T_check, pos_check = self.forward_kinematics(*solution)
            error = np.linalg.norm(pos_check - target_position)
            
            if error < 10.0:  # Tolerancia de 10mm
                solutions.append(solution)
                if verbose:
                    print(f"Solución válida: {[np.rad2deg(t) for t in solution]}")
                    print(f"Error: {error:.1f} mm")
        
        return solutions

    def test_specific_configurations(self):
        """Probar las configuraciones específicas que proporcionaste"""
        print("=== PRUEBA DE CONFIGURACIONES ESPECÍFICAS ===\n")
        
        # Configuración 1: [0, 0, 90, 0, 0]
        print("1. Configuración [0, 0, 90, 0, 0]:")
        q1 = np.deg2rad([0, 0, 90, 0, 0])
        T1, pos1 = self.forward_kinematics(*q1)
        print(f"  Posición FK: {pos1}")
        
        solutions1 = self.compute_analytical_ik(pos1, verbose=True)
        if solutions1:
            for i, sol in enumerate(solutions1):
                deg_sol = [np.rad2deg(angle) for angle in sol]
                print(f"  Solución IK {i+1}: {deg_sol}")
        else:
            print("  No se encontraron soluciones IK")
        
        print("\n" + "-"*50)
        
        # Configuración 2: [90, 90, 90, 0, 0]
        print("2. Configuración [90, 0, 90, 0, 0]:")
        q2 = np.deg2rad([90, 90, 90, 0, 0])
        T2, pos2 = self.forward_kinematics(*q2)
        print(f"  Posición FK: {pos2}")
        
        solutions2 = self.compute_analytical_ik(pos2, verbose=True)
        if solutions2:
            for i, sol in enumerate(solutions2):
                deg_sol = [np.rad2deg(angle) for angle in sol]
                print(f"  Solución IK {i+1}: {deg_sol}")
        else:
            print("  No se encontraron soluciones IK")

    def calculate_for_custom_position(self, x, y, z):
        """Calcular IK para una posición personalizada"""
        print(f"\n=== CÁLCULO PARA POSICIÓN [{x}, {y}, {z}] ===")
        
        target_position = [x, y, z]
        solutions = self.compute_analytical_ik(target_position, verbose=True)
        
        if solutions:
            print(f"\nSe encontraron {len(solutions)} soluciones:")
            for i, sol in enumerate(solutions):
                deg_sol = [np.rad2deg(angle) for angle in sol]
                print(f"Solución {i+1}: {deg_sol}")
                
                # Verificar con cinemática directa
                T_check, pos_check = self.forward_kinematics(*sol)
                error = np.linalg.norm(pos_check - target_position)
                print(f"  Verificación FK: {pos_check}, error: {error:.2f} mm")
        else:
            print("No se encontraron soluciones para esta posición")

# ---- Ejecución de pruebas ----
if __name__ == "__main__":
    ik_solver = Robot5DOFIK()
    
    # Probar las configuraciones específicas
    ik_solver.test_specific_configurations()
    
    # Ejemplo de cálculo para una posición personalizada
    ik_solver.calculate_for_custom_position(200, 0, 300)