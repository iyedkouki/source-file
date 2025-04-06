#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import keyboard
import math

class BoatController:
    def __init__(self, model_name="boatcleaningc"):
        self.model_name = model_name
        self.base_name = f"{model_name}::baseboatclening"
        rospy.init_node('boat_controller_node', anonymous=True)
        
        self._init_services()
        
        if not self.check_model_exists():
            raise Exception(f"Model '{model_name}' not found in Gazebo!")
        
        self.force_magnitude = 10.0
        self.torque_magnitude = 5.0
        self.duration = rospy.Duration(0.1)
        self.current_yaw = 0.0  # Orientation actuelle du bateau

    def _init_services(self):
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def check_model_exists(self):
        try:
            req = GetModelStateRequest()
            req.model_name = self.model_name
            response = self.get_model_state(req)
            return response.success
        except rospy.ServiceException as e:
            print("Model check failed: %s" % e)
            return False

    def get_boat_orientation(self):
        try:
            req = GetModelStateRequest()
            req.model_name = self.model_name
            response = self.get_model_state(req)
            if response.success:
                return response.pose.orientation
            return None
        except rospy.ServiceException as e:
            print("Failed to get orientation: %s" % e)
            return None

    def quaternion_to_yaw(self, q):
        # Convertir quaternion en yaw (rotation autour de Z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def apply_force(self, local_force_x=0.0, tz=0.0):
        try:
            # Obtenir l'orientation actuelle
            orientation = self.get_boat_orientation()
            if orientation is None:
                return False

            # Mettre à jour le yaw actuel
            self.current_yaw = self.quaternion_to_yaw(orientation)
            
            # Calculer la force dans le repère global basée uniquement sur l'orientation actuelle
            fx_world = local_force_x * math.cos(self.current_yaw)
            fy_world = local_force_x * math.sin(self.current_yaw)

            # Configurer la requête de wrench
            wrench = ApplyBodyWrenchRequest()
            wrench.body_name = self.base_name
            wrench.reference_frame = "world"  # Utiliser le repère global
            wrench.reference_point.x = 0.0   # Appliquer au centre
            wrench.reference_point.y = 0.0
            wrench.reference_point.z = 0.0
            wrench.wrench.force.x = fx_world
            wrench.wrench.force.y = fy_world
            wrench.wrench.force.z = 0.0      # Pas de force verticale
            wrench.wrench.torque.x = 0.0      # Pas de roulis
            wrench.wrench.torque.y = 0.0      # Pas de tangage
            wrench.wrench.torque.z = tz       # Rotation autour de Z uniquement
            wrench.duration = self.duration

            response = self.apply_wrench(wrench)
            
            if response.success:
                yaw_deg = math.degrees(self.current_yaw)
                print(f"Orientation (yaw): {yaw_deg:.1f}° | Force: fx={fx_world:.1f}, fy={fy_world:.1f}")
            return response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    def control_loop(self):
        print("Boat Control - Use WASD keys (Q to quit):")
        print("W: Forward, S: Backward, A: Rotate Left, D: Rotate Right")
        
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown() and not keyboard.is_pressed('q'):
            local_fx = 0.0
            tz = 0.0
            
            # Rotation
            if keyboard.is_pressed('a'):
                tz = self.torque_magnitude   # Tourner à gauche
            elif keyboard.is_pressed('d'):
                tz = -self.torque_magnitude  # Tourner à droite
                
            # Mouvement avant/arrière
            if keyboard.is_pressed('w'):
                local_fx = self.force_magnitude  # Avancer
            elif keyboard.is_pressed('s'):
                local_fx = -self.force_magnitude  # Reculer

            # Appliquer la force ou le couple si nécessaire
            if local_fx != 0.0 or tz != 0.0:
                success = self.apply_force(local_fx, tz)
                if success:
                    print(f"Applied: local_force_x={local_fx:.1f}, torque_z={tz:.1f}")

            rate.sleep()
        
        print("Shutting down boat controller")

if __name__ == '__main__':
    try:
        controller = BoatController()
        controller.control_loop()
    except Exception as e:
        print(f"Error: {e}")
