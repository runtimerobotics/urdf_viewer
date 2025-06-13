import numpy as np
from urdfpy import URDF
from scipy.spatial.transform import Rotation as R


class Joint:
    def __init__(self, urdf_joint):
        self.name = urdf_joint.name
        self.joint_type = urdf_joint.joint_type
        self.limit = urdf_joint.limit if urdf_joint.limit else None
        self.origin = urdf_joint.origin  # 4x4 transform matrix from parent link to child link frame
        # Default axis is x if none specified (rare)
        self.axis = urdf_joint.axis if urdf_joint.axis is not None else np.array([1, 0, 0])
        self.current_value = 0.0
        self.parent_link = None  # set later
        self.child_link = None  # set later


class Link:
    def __init__(self, urdf_link):
        self.name = urdf_link.name
        self.visuals = urdf_link.visuals  # List of Visual objects with geometry and origin
        self.parent_joint = None  # set later
        self.child_joints = []
        self.child_links = []
        self.tf = np.eye(4)  # 4x4 transform matrix of this link in world frame


class Robot:
    def __init__(self, urdf_path):
        self.urdf = URDF.load(urdf_path)
        self.name = self.urdf.name

        self.links = []
        self.joints = []
        self.link_map = {}
        self.joint_map = {}

        # Create Link instances
        for link in self.urdf.links:
            l = Link(link)
            self.links.append(l)
            self.link_map[link.name] = l

        # Create Joint instances
        for joint in self.urdf.joints:
            j = Joint(joint)
            self.joints.append(j)
            self.joint_map[joint.name] = j

        # Setup parent/child relations between links and joints
        for joint in self.urdf.joints:
            parent_link = self.link_map[joint.parent]
            child_link = self.link_map[joint.child]
            j = self.joint_map[joint.name]

            child_link.parent_joint = j
            j.parent_link = parent_link
            j.child_link = child_link

            parent_link.child_joints.append(j)
            parent_link.child_links.append(child_link)

        # Find root link (the one without a parent joint)
        self.root_link = None
        for link in self.links:
            if link.parent_joint is None:
                self.root_link = link
                break

        # List of actuated joints
        self.actuated_joints = [j for j in self.joints if j.joint_type in ("revolute", "continuous", "prismatic")]

    def update_cfg(self, cfg):
        """
        Update joint values and compute forward kinematics transforms.
        :param cfg: list of float joint values, same order as self.actuated_joints
        """

        # Assign current joint values from cfg list
        for joint, val in zip(self.actuated_joints, cfg):
            joint.current_value = val

        def joint_transform(joint):
            # Start from joint origin transform (parent link -> joint frame)
            tf = joint.origin.copy()  # 4x4

            axis = np.array(joint.axis)

            if joint.joint_type in ("revolute", "continuous"):
                # Rotation about axis by current_value radians
                r = R.from_rotvec(joint.current_value * axis)
                tf[:3, :3] = tf[:3, :3] @ r.as_matrix()
            elif joint.joint_type == "prismatic":
                # Translation along axis by current_value meters
                tf[:3, 3] += joint.current_value * axis
            # fixed joints no change

            return tf

        def update_link_transform(link, parent_tf=np.eye(4)):
            # Compute this link's transform in world frame
            if link.parent_joint is None:
                # root link transform = parent transform (usually identity)
                link.tf = parent_tf
            else:
                jt = joint_transform(link.parent_joint)
                link.tf = parent_tf @ jt

            # Recursively update children links
            for child_link in link.child_links:
                update_link_transform(child_link, link.tf)

        # Compute forward kinematics from root link
        update_link_transform(self.root_link)

    def link_tf_tree(self):
        """
        Return dict mapping each Link instance to its 4x4 transform matrix in world frame.
        """
        return {link: link.tf for link in self.links}

    def get_visual_world_transforms(self):
        """
        Return dict mapping each Link instance to a list of visual transforms in world frame.
        Each visual transform = link transform * visual.origin
        """
        visual_transforms = {}
        for link in self.links:
            vis_tfs = []
            for visual in link.visuals:
                # visual.origin is 4x4 transform relative to link frame
                full_tf = link.tf @ visual.origin
                vis_tfs.append((visual, full_tf))
            visual_transforms[link] = vis_tfs
        return visual_transforms


def load_urdf(file_path):
    """
    Convenience function to load Robot instance.
    """
    return Robot(file_path)
