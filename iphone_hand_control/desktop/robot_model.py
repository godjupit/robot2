"""Standalone SO100 model adapted from robot2; positions in meters."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parent

class FeedbackClient:
    """通过原生接口逐舵机读取位置，避免全组读缓存参与到位判定。"""
    def __init__(self, client, config):
        self.client, self.config = client, config
        self.last_ticks = None

    def call(self, *args, **kwargs):
        return self.client.call(*args, **kwargs)

    def read(self):
        # 非默认顺序使当前 SO100 实现走单舵机读取分支。使用 motor_units
        # 避免服务对子集/重排后的数组使用错误的角度标定索引。
        ids = [6,5,4,3,2,1]
        data = self.call('/joints/read',{'unit':'motor_units','source':'robot','joints_ids':ids})
        values = data.get('angles',[])
        if (data.get('unit') != 'motor_units' or len(values) != 6
                or any(v is None or not math.isfinite(v) or not 0 <= v <= 4095 for v in values)):
            raise RuntimeError('逐舵机原始位置反馈无效，停止运动')
        ticks = dict(zip(ids,values))
        self.last_ticks = [ticks[i] for i in range(1,7)]
        return [(ticks[i]-self.config['servos_offsets'][i-1])
                *self.config['servos_offsets_signs'][i-1]*360/4095 for i in range(1,7)]

class Model:
    """Phosphobot 原始 SO100 URDF；独立 SciPy IK + 原始网格碰撞检查。"""
    def __init__(self, tcp_offset_cm=None):
        import numpy as np
        import trimesh
        from scipy.spatial.transform import Rotation
        from trimesh.collision import CollisionManager
        self.np, self.rotation = np, Rotation
        self.tcp_mode = tcp_offset_cm is not None
        self.tcp_offset = np.asarray([0.,0.,0.] if tcp_offset_cm is None else tcp_offset_cm,dtype=float)/100
        if self.tcp_offset.shape != (3,) or not np.isfinite(self.tcp_offset).all():
            raise ValueError('TCP 偏移必须为 Fixed_Jaw 局部坐标系中的三个有限数值（cm）')
        self.root = ET.parse(ROOT/'models/so-100.urdf').getroot()
        self.joints = self.root.findall('joint')
        self.lower = np.array([float(j.find('limit').get('lower')) for j in self.joints[:5]])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in self.joints[:5]])
        self.origins = [self.origin(j.find('origin')) for j in self.joints]
        self.axes = [np.fromstring(j.find('axis').get('xyz'),sep=' ') for j in self.joints]
        self.manager = CollisionManager()
        self.geometry = []
        adjacent = {frozenset((j.find('parent').get('link'),j.find('child').get('link'))) for j in self.joints}
        assets = ROOT/'models'
        for link in self.root.findall('link'):
            for i, collision in enumerate(link.findall('collision')):
                mesh_info = collision.find('geometry/mesh')
                mesh = trimesh.load_mesh(assets/mesh_info.get('filename').replace('package://',''),process=False)
                mesh.apply_scale(np.fromstring(mesh_info.get('scale','1 1 1'),sep=' '))
                if not np.isfinite(mesh.vertices).all():
                    raise RuntimeError('碰撞网格包含非有限坐标')
                name = f'{link.get("name")}:{i}'
                self.manager.add_object(name,mesh)
                self.geometry.append((name,link.get('name'),self.origin(collision.find('origin')),mesh.vertices))
        self.pairs = [(a,b) for i,a in enumerate(self.manager._objs) for b in list(self.manager._objs)[i+1:]
                      if frozenset((a.split(':')[0],b.split(':')[0])) not in adjacent
                      and a.split(':')[0] != b.split(':')[0]]

    def origin(self, element):
        np = self.np
        t = np.eye(4)
        if element is not None:
            t[:3,:3] = self.rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
            t[:3,3] = np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        return t

    def transforms(self, q, jaw):
        np = self.np
        base = np.eye(4);base[:3,:3] = self.rotation.from_quat([0,0,1,1]).as_matrix()
        transforms = {'Base':base}
        for j,origin,axis,angle in zip(self.joints,self.origins,self.axes,list(q)+[jaw]):
            motion = np.eye(4);motion[:3,:3] = self.rotation.from_rotvec(axis*angle).as_matrix()
            transforms[j.find('child').get('link')] = transforms[j.find('parent').get('link')]@origin@motion
        return transforms

    def position(self, q):
        t = self.transforms(q,0)['Fixed_Jaw']
        return t[:3,3]+t[:3,:3]@self.tcp_offset

    def solve_position(self, target, seed):
        """位置优先 TCP IK：XYZ 残差为 mm，不加入工具朝向约束。"""
        from scipy.optimize import least_squares
        np = self.np
        return least_squares(lambda angles: np.r_[(self.position(angles)-target)*1000,
                                                   (angles-seed)*.001],
                             seed,bounds=(self.lower,self.upper),max_nfev=200).x

    def check(self, q, jaw):
        import fcl
        np = self.np
        if not np.isfinite(q).all() or not math.isfinite(jaw):
            raise RuntimeError('碰撞检查关节角无效')
        if np.any(q < self.lower-1e-9) or np.any(q > self.upper+1e-9):
            raise RuntimeError('关节角超出 URDF 限位')
        transforms = self.transforms(q,jaw)
        for name,link,origin,vertices in self.geometry:
            t = transforms[link]@origin
            self.manager.set_transform(name,t)
            if link not in ('Base','Rotation_Pitch'):
                z = np.sum(vertices*t[2,:3],axis=1)+t[2,3]
                if not np.isfinite(z).all():
                    raise RuntimeError('碰撞变换产生非有限坐标')
                if z.min() < .005:
                    raise RuntimeError(f'{link} 网格距模型 Z=0 平面不足 5 mm')
        for a,b in self.pairs:
            objects = self.manager._objs
            if fcl.collide(objects[a]['obj'],objects[b]['obj'],fcl.CollisionRequest(),fcl.CollisionResult()):
                raise RuntimeError(f'模型自碰撞：{a} / {b}')
