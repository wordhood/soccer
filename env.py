import pybullet as pb
import pybullet_data
import numpy as np

class SimpleEnv:
    """简单的PyBullet仿真环境"""
    
    def __init__(self, gui=True, gravity=-9.8):
        """
        初始化仿真环境
        
        参数:
            gui: 是否显示GUI界面
            gravity: 重力加速度
        """
        # 连接到仿真服务器
        self.mode = pb.GUI if gui else pb.DIRECT
        self.physicsClient = pb.connect(self.mode)
        
        # 设置重力
        pb.setGravity(0, 0, gravity)
        
        # 设置数据路径
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 存储物体ID
        self.objects = {}
        self.step_count = 0
        
        # 设置相机
        self.setup_camera()
        
    def setup_camera(self):
        """设置相机视角"""
        pb.resetDebugVisualizerCamera(
            cameraDistance=5,
            cameraYaw=30,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # 可选：隐藏调试GUI界面
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    
    def create_ground(self):
        """创建地面"""
        plane_id = pb.loadURDF("plane.urdf")
        self.objects['ground'] = plane_id
        return plane_id
    
    def create_box(self, position=[0, 0, 1], size=0.5, mass=1.0, color=[1, 0, 0, 1]):
        """
        创建立方体
        
        参数:
            position: 位置 [x, y, z]
            size: 立方体半边长
            mass: 质量
            color: 颜色 [r, g, b, a]
        """
        # 创建碰撞形状
        box_collision = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[size, size, size])
        
        # 创建视觉形状
        box_visual = pb.createVisualShape(pb.GEOM_BOX, 
                                        halfExtents=[size, size, size],
                                        rgbaColor=color)
        
        # 创建刚体
        box_id = pb.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=box_collision,
            baseVisualShapeIndex=box_visual,
            basePosition=position
        )
        
        # 存储物体ID
        obj_name = f"box_{len([k for k in self.objects.keys() if 'box' in k])}"
        self.objects[obj_name] = box_id
        
        return box_id
    
    def create_sphere(self, position=[0, 0, 1], radius=0.5, mass=1.0, color=[0, 0, 1, 1]):
        """
        创建球体
        
        参数:
            position: 位置 [x, y, z]
            radius: 半径
            mass: 质量
            color: 颜色 [r, g, b, a]
        """
        # 创建碰撞形状
        sphere_collision = pb.createCollisionShape(pb.GEOM_SPHERE, radius=radius)
        
        # 创建视觉形状
        sphere_visual = pb.createVisualShape(pb.GEOM_SPHERE,
                                           radius=radius,
                                           rgbaColor=color)
        
        # 创建刚体
        sphere_id = pb.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=sphere_collision,
            baseVisualShapeIndex=sphere_visual,
            basePosition=position
        )
        
        # 存储物体ID
        obj_name = f"sphere_{len([k for k in self.objects.keys() if 'sphere' in k])}"
        self.objects[obj_name] = sphere_id
        
        return sphere_id
    
    def create_cylinder(self, position=[0, 0, 1], radius=0.5, height=1.0, mass=1.0, color=[0, 1, 0, 1]):
        """
        创建圆柱体
        
        参数:
            position: 位置 [x, y, z]
            radius: 半径
            height: 高度
            mass: 质量
            color: 颜色 [r, g, b, a]
        """
        # 创建碰撞形状
        cylinder_collision = pb.createCollisionShape(
            pb.GEOM_CYLINDER,
            radius=radius,
            height=height
        )
        
        # 创建视觉形状
        cylinder_visual = pb.createVisualShape(
            pb.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=color
        )
        
        # 创建刚体
        cylinder_id = pb.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=cylinder_collision,
            baseVisualShapeIndex=cylinder_visual,
            basePosition=position
        )
        
        # 存储物体ID
        obj_name = f"cylinder_{len([k for k in self.objects.keys() if 'cylinder' in k])}"
        self.objects[obj_name] = cylinder_id
        
        return cylinder_id
    
    def step(self, sleep_time=1./240.):
        """
        执行一步仿真
        
        参数:
            sleep_time: 步进延迟时间
        """
        pb.stepSimulation()
        self.step_count += 1
        
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    def get_object_state(self, obj_id):
        """获取物体状态"""
        pos, orn = pb.getBasePositionAndOrientation(obj_id)
        lin_vel, ang_vel = pb.getBaseVelocity(obj_id)
        
        return {
            'position': np.array(pos),
            'orientation': np.array(orn),
            'linear_velocity': np.array(lin_vel),
            'angular_velocity': np.array(ang_vel)
        }
    
    def get_keyboard_events(self):
        """获取键盘事件"""
        return pb.getKeyboardEvents()
    
    def apply_force(self, obj_id, force, position=[0, 0, 0], frame=pb.WORLD_FRAME):
        """给物体施加力"""
        pb.applyExternalForce(obj_id, -1, force, position, frame)
    
    def reset_object(self, obj_id, position, orientation=[0, 0, 0, 1]):
        """重置物体位置和姿态"""
        pb.resetBasePositionAndOrientation(obj_id, position, orientation)
    
    def is_connected(self):
        """检查是否连接到仿真服务器"""
        return pb.isConnected()
    
    def close(self):
        """关闭环境"""
        pb.disconnect()
        print(f"仿真结束，总共运行了 {self.step_count} 步")
    
    def print_status(self):
        """打印环境状态"""
        print(f"=== 环境状态 (Step: {self.step_count}) ===")
        print(f"物体数量: {len(self.objects)}")
        for name, obj_id in self.objects.items():
            state = self.get_object_state(obj_id)
            print(f"  {name}: 位置={state['position']}")
        print("=" * 40)


# 简单的时间模块，如果不需要可以删除
import time