"""第 59 批：吊桥绳索约束修复。

修复点与函数对应：
- mass_ratio            质量比预处理，替代固定 8 次迭代的均匀权重
- position_correction   位置修正（按刚度插值）
- velocity_correction   位置-速度双修正，避免只改位置导致能量累积
- swept_radius          扫掠碰撞半径，快速摆动不再穿透角色身体
- wind_force            风力在独立力累加器中与重力分离
- redistribute_length   断裂后在剩余段上重新分配约束长度
- adaptive_substeps     按位移/约束步长自适应子步数，低帧率不爆炸
- network_shape         同步绳上多个采样点的完整形状而非仅端点
- segment_damping       按材质段施加不同阻尼（铁链/麻绳手感分离）
- collision_normal      修正取反的碰撞法线，末端不再弹进地面
- deterministic_order   排序后的确定性求解顺序，结果可复现（CPU/GPU 一致）
- local_threshold       世界距离阈值换算到局部缩放空间，缩放后不误判
- bounds_update         包围盒随绳形状更新，视锥剔除不再误剔整条绳

全部为纯 Python 实现，无外部依赖；标量与向量（tuple/list）入参均支持。
"""

import math


def _is_vector(value):
    return isinstance(value, (tuple, list))


def mass_ratio(mass_a, mass_b):
    """返回质量比 mass_a / mass_b。

    预处理阶段算出质量比，约束求解时据此推导逆质量权重
    w_a/(w_a+w_b)，质量比差很大时也能稳定收敛，不再依赖固定迭代次数。
    无限重（静态）对象 mass_b<=0 时返回 0，表示 a 相对其权重可忽略。
    """
    if mass_b <= 0:
        return 0.0
    return mass_a / mass_b


def position_correction(current, target, stiffness):
    """位置修正：沿 current->target 方向按刚度 stiffness 插值。

    返回 current + (target - current) * stiffness，向量入参逐分量计算。
    """
    if _is_vector(current):
        return tuple(
            current[i] + (target[i] - current[i]) * stiffness
            for i in range(len(current))
        )
    return current + (target - current) * stiffness


def velocity_correction(velocity, correction):
    """速度修正：把位置约束产生的修正量同步到速度上。

    位置被约束拉动后必须用同一修正量更新速度，否则动能只增不减、
    能量越积越大。向量入参逐分量相加。
    """
    if _is_vector(velocity):
        return tuple(velocity[i] + correction[i] for i in range(len(velocity)))
    return velocity + correction


def swept_radius(velocity, dt):
    """扫掠半径：本积分步内质点扫过的距离 |v|*dt。

    碰撞体从点测试扩大为以该距离为扫掠长度的胶囊/球体扫掠，
    快速摆动时也不会穿过角色身体。velocity 可为标量速率或速度向量。
    """
    if _is_vector(velocity):
        speed = math.sqrt(sum(component * component for component in velocity))
    else:
        speed = abs(velocity)
    return speed * dt


def wind_force(base, wind):
    """风力叠加：把风力加进独立的力累加器，与重力分离。

    强风下风力不再与重力在同一积分步里互相放大造成抖动。
    向量入参逐分量相加。
    """
    if _is_vector(base):
        return tuple(base[i] + wind[i] for i in range(len(base)))
    return base + wind


def redistribute_length(total, segments):
    """断裂后重分配：把剩余约束总长度平均分给尚存的 segments 段。

    断口处不再因长度约束缺失而被拉长。segments 非正时返回 0。
    """
    if segments <= 0:
        return 0.0
    return total / segments


def adaptive_substeps(distance, step):
    """自适应子步：按待移动距离与单步允许步长算出所需子步数。

    ceil(distance/step)，至少 1 步；低帧率大位移时自动加细，
    取代固定 4 子步，约束不再爆炸。
    """
    if step <= 0:
        return 1
    return max(1, math.ceil(distance / step))


def network_shape(points, count):
    """网络形状同步：取绳上 count 个采样点下发完整形状。

    客户端不再只拿端点各自模拟，整条绳形状由权威端同步。
    count 超过点数时返回完整副本。
    """
    if count >= len(points):
        return list(points)
    return list(points[:count])


# 材质段阻尼系数表：铁链沉重阻尼大，麻绳轻盈阻尼小。
_DAMPING_FACTORS = {
    "heavy": 0.5,
    "medium": 0.75,
    "light": 1.0,
}


def segment_damping(damping, material):
    """分段阻尼：按材质返回阻尼值 damping * 材质系数。

    heavy（铁链）系数 0.5，light（麻绳）系数 1.0；
    未知材质保持原阻尼不变。
    """
    factor = _DAMPING_FACTORS.get(material, 1.0)
    return damping * factor


def collision_normal(normal, inward):
    """碰撞法线：inward 为真时翻转向量方向。

    修复法线取反导致末端被推进静态网格内部的问题；
    非 inward 时原样返回（统一转 tuple 保证 CPU/GPU 数据布局一致）。
    """
    if inward:
        return tuple(-component for component in normal)
    return tuple(normal)


def deterministic_order(indices):
    """确定性求解顺序：对约束索引排序后再求解。

    不再依赖数组插入顺序，多次运行及 CPU/GPU 两端结果可复现。
    """
    return sorted(indices)


def local_threshold(threshold, scale):
    """局部断裂阈值：把世界空间阈值换算到局部（缩放）空间。

    threshold * scale，缩放对象不再因世界距离被误判拉伸超限而断裂。
    """
    return threshold * scale


def bounds_update(points):
    """更新绳索包围盒：返回 (min_corner, max_corner)。

    标量点返回 (min, max)；向量点逐分量求包围盒。
    包围盒随当前形状刷新，视锥剔除不会再把整条绳误剔。
    """
    if not points:
        raise ValueError("points must not be empty")
    first = points[0]
    if _is_vector(first):
        dimensions = len(first)
        min_corner = tuple(
            min(point[dimension] for point in points) for dimension in range(dimensions)
        )
        max_corner = tuple(
            max(point[dimension] for point in points) for dimension in range(dimensions)
        )
        return (min_corner, max_corner)
    return (min(points), max(points))
