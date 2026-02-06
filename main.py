import time
from env import SimpleEnv

def main():
    """主函数：运行仿真环境"""
    
    # 1. 创建环境
    print("正在初始化仿真环境...")
    env = SimpleEnv(gui=True, gravity=-9.8)
    
    # 2. 创建场景
    print("正在创建场景...")
    
    # 创建地面
    ground = env.create_ground()
    
    # 创建一些物体
    red_box = env.create_box(
        position=[0, 0, 3],
        size=0.5,
        mass=1.0,
        color=[1, 0, 0, 1]  # 红色
    )
    
    blue_sphere = env.create_sphere(
        position=[1, 0, 5],
        radius=0.5,
        mass=0.5,
        color=[0, 0, 1, 1]  # 蓝色
    )
    
    green_cylinder = env.create_cylinder(
        position=[-1, 0, 4],
        radius=0.3,
        height=1.0,
        mass=2.0,
        color=[0, 1, 0, 1]  # 绿色
    )
    
    print("场景创建完成！")
    print("控制命令:")
    print("  q - 退出仿真")
    print("  r - 重置红色方块位置")
    print("  s - 显示状态")
    print("  f - 给红色方块施加随机力")
    print("  c - 创建新物体")
    print("=" * 50)
    
    try:
        # 3. 仿真循环
        while True:
            # 执行一步仿真
            env.step()
            
            # 每200步显示一次状态
            if env.step_count % 200 == 0:
                pos, _ = pb.getBasePositionAndOrientation(red_box)
                print(f"Step {env.step_count}: 红色方块位置 = [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
            
            # 处理键盘输入
            keys = env.get_keyboard_events()
            
            # 退出键
            if ord('q') in keys and keys[ord('q')] & pb.KEY_WAS_TRIGGERED:
                print("检测到 'q' 键，退出仿真")
                break
            
            # 重置键
            if ord('r') in keys and keys[ord('r')] & pb.KEY_WAS_TRIGGERED:
                env.reset_object(red_box, [0, 0, 3])
                print("重置红色方块位置")
            
            # 显示状态键
            if ord('s') in keys and keys[ord('s')] & pb.KEY_WAS_TRIGGERED:
                env.print_status()
            
            # 施加力键
            if ord('f') in keys and keys[ord('f')] & pb.KEY_WAS_TRIGGERED:
                import random
                force = [random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(0, 20)]
                env.apply_force(red_box, force)
                print(f"给红色方块施加力: {force}")
            
            # 创建新物体键
            if ord('c') in keys and keys[ord('c')] & pb.KEY_WAS_TRIGGERED:
                import random
                colors = [[1,0,0,1], [0,1,0,1], [0,0,1,1], [1,1,0,1], [1,0,1,1]]
                color = random.choice(colors)
                
                if random.random() > 0.5:
                    # 创建立方体
                    env.create_box(
                        position=[random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(3, 6)],
                        size=random.uniform(0.2, 0.4),
                        mass=random.uniform(0.5, 2.0),
                        color=color
                    )
                    print("创建了一个新的立方体")
                else:
                    # 创建球体
                    env.create_sphere(
                        position=[random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(3, 6)],
                        radius=random.uniform(0.2, 0.4),
                        mass=random.uniform(0.5, 2.0),
                        color=color
                    )
                    print("创建了一个新的球体")
            
            # 检查连接状态
            if not env.is_connected():
                print("仿真窗口已关闭")
                break
            
    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)，退出仿真")
    
    finally:
        # 4. 关闭环境
        env.close()

# 需要在主程序中导入pybullet，因为main.py使用了pb
import pybullet as pb

if __name__ == "__main__":
    main()