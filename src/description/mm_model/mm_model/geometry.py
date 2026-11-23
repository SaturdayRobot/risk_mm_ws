"""URDF-derived sampling geometry shared by perception and task stop validation."""
import math
import numpy as np
from scipy.spatial import cKDTree

class RobotGeometry:
    def __init__(self,model):
        self.base=np.asarray(model['mm.base_collision_points']).reshape(-1,3)
        self.links=[np.asarray(model[f'mm.arm_collision_points_{i}']).reshape(-1,3) for i in range(6)]
        self.mount=np.asarray(model['mm.arm_mount_transform']).reshape(4,4)
        self.origins=np.asarray(model['mm.arm_joint_origins']).reshape(6,4,4);self.axes=np.asarray(model['mm.arm_joint_axes']).reshape(6,3)
        self.base_radius=float(model['mm.mobile_base_check_radius']);self.arm_radius=float(model['mm.manipulator_thickness'])
        self.base_owners=np.asarray(model['mm.base_collision_owners'],dtype=int)
        self.arm_owners=[np.asarray(model[f'mm.arm_collision_owners_{i}'],dtype=int) for i in range(6)]
        self.disabled_pairs={tuple(sorted((int(a),int(b)))) for a,b in
                             np.asarray(model['mm.self_collision_disabled_pairs']).reshape(-1,2)}
        self.body_tree=cKDTree(self.base);self.chassis_tree=cKDTree(self.base[self.base_owners!=1])
    def spheres(self,pose,q):
        x,y,yaw=pose[:3];T=np.eye(4);c,s=math.cos(yaw),math.sin(yaw);T[:2,:2]=[[c,-s],[s,c]];T[:2,3]=[x,y]
        groups=[((np.c_[self.base,np.ones(len(self.base))]@T.T)[:,:3],self.base_radius)]
        T=T@self.mount
        for i,a in enumerate(q):
            axis=self.axes[i]/np.linalg.norm(self.axes[i]);u,v,w=axis;K=np.array([[0,-w,v],[w,0,-u],[-v,u,0]])
            J=self.origins[i].copy();J[:3,:3]=J[:3,:3]@(np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K));T=T@J
            points=self.links[i];groups.append(((np.c_[points,np.ones(len(points))]@T.T)[:,:3],self.arm_radius))
        return groups
    def valid(self,pose,q,boxes,margin=.01,*,base_margin=None):
        centers = np.asarray([box.center for box in boxes], dtype=float).reshape(-1, 3)
        half_sizes = np.asarray([box.size for box in boxes], dtype=float).reshape(-1, 3) / 2.
        box_min, box_max = centers - half_sizes, centers + half_sizes
        for index,(points,radius) in enumerate(self.spheres(pose,q)):
            gap = radius + (base_margin if index == 0 and base_margin is not None else margin)
            padding = max(0., gap)
            group_min, group_max = points.min(axis=0) - padding, points.max(axis=0) + padding
            # Disjoint expanded boxes cannot contain a point within the required gap.
            nearby = np.flatnonzero(np.all((box_min <= group_max) & (box_max >= group_min), axis=1))
            for index in nearby:
                delta=np.abs(points-centers[index])-half_sizes[index]
                distance=np.linalg.norm(np.maximum(delta,0.),axis=1)+np.minimum(np.max(delta,axis=1),0.)
                if float(distance.min())<gap:return False
        return True

    def self_valid(self,q):
        groups=[points for points,_ in self.spheres((0.,0.,0.),q)][1:]
        bounds=[(points.min(axis=0),points.max(axis=0)) for points in groups]
        trees={}
        for a,points in enumerate(groups):
            if a>0:
                tree=self.chassis_tree if a==1 else self.body_tree
                distance,_=tree.query(points)
                if np.any(distance<self.base_radius+self.arm_radius):return False
            for b in range(a):
                separation=np.maximum(bounds[a][0]-bounds[b][1],0.)+np.maximum(bounds[b][0]-bounds[a][1],0.)
                if np.linalg.norm(separation)>2.*self.arm_radius:continue
                for source_owner in np.unique(self.arm_owners[a]):
                    selected=points[self.arm_owners[a]==source_owner]
                    for target_owner in np.unique(self.arm_owners[b]):
                        if source_owner==target_owner or tuple(sorted((int(source_owner),int(target_owner)))) in self.disabled_pairs:continue
                        key=(b,int(target_owner))
                        if key not in trees:trees[key]=cKDTree(groups[b][self.arm_owners[b]==target_owner])
                        distance,_=trees[key].query(selected)
                        if np.any(distance<2.*self.arm_radius):return False
        return True
