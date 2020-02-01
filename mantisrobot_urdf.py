import sys
import math
from odio_urdf import *


# https://github.com/hauptmech/odio_urdf

# this script generates the urdf file for mantis robot (aka robot v5)
# https://github.com/4ndreas/Mantis-Robot-Arm 
# 
# more infos at:
# https://hackaday.io/project/3800-3d-printable-robot-arm


filepath = ""
robot_name = "MantisRobot"

PI = math.pi
safety_controller_k_pos = 100
safety_controller_k_vel = 2
joint_damping = 0.5
max_effort = 300
max_velocity = 10
# filepath = "package://iiwa_description/meshes/iiwa7/"
filepath = "./mantis/"

def link(N,robot_name,origin,mass,I,material,geom_origin):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    N = str(N)
    ret = Link(
        Inertial(
            Origin(origin),
            Mass(value=mass),
            Inertia(I)),
        Visual(
            Origin(geom_origin),
            Geometry(Mesh(filename = filepath+"visual/link_"+N+".stl")),
            Material(material)),
        Collision(
            Origin(geom_origin),
            Geometry(Mesh(filename = filepath+"visual/link_"+N+".stl")),
            Material(material)),
       name = robot_name+"_link_"+N)
    return ret

def joint(N,robot_name,origin,limit=None,safe=None, axis="0 0 1"):
    """
        Most of the joints are the same except for the passed in info.
        This function just allows grouping the important numbers better. 
    """
    N = int(N)
    if limit != None:
        ret = Joint(
            Parent(link=robot_name+"_link_"+str(N-1)),
            Child(link=robot_name+"_link_"+str(N)),
            Origin(origin),
            Axis(xyz=axis),
            Limit(lower=limit[0]*PI/180, upper=limit[1]*PI/180, effort=max_effort, velocity=max_velocity),
            Safety_controller(soft_lower_limit=safe[0]*PI/180,soft_upper_limit=safe[1]*PI/180,
                k_position=safety_controller_k_pos, k_velocity=safety_controller_k_vel),
            Dynamics(damping=joint_damping),
            type="revolute",
            name= robot_name+"_joint_"+str(N))
    else:   # no limit joint
        ret = Joint(
            Parent(link=robot_name+"_link_"+str(N-1)),
            Child(link=robot_name+"_link_"+str(N)),
            Origin(origin),
            Axis(xyz=axis),
            Limit(effort=max_effort, velocity=max_velocity),
            Safety_controller(k_position=safety_controller_k_pos, k_velocity=safety_controller_k_vel),
            Dynamics(damping=joint_damping),
            type="revolute",
            name= robot_name+"_joint_"+str(N))                    
    return ret


def mantis_main(parent, hardware_interface, robot_name):
    """
        Main definition of the iiwa7. 
    """
    ret = Group(
        Joint(parent+"_"+robot_name+"_joint", Origin(xyz="0 0 0", rpy="0 0 0"),
            Parent(link=parent), Child(link=robot_name+"_link_0"), type="fixed"),
        link(0, robot_name,[0.0,0.0   ,0.00  ,0,0,0],5,[0,0,0.0,0,0,0], "Grey",[0,0,0]),

        joint(1,robot_name,[0.0 ,0.0   ,0.00  ,0,0,0],[-180,180],[-170,170],),
        link(1, robot_name,[0.0 ,0.0 ,0.0  ,0,0,0],4,[0,1,0,0.0,0,0.02], "Orange",[0,0,0.0]),

        joint(2,robot_name,[0.0 ,0.0   ,0.297  ,0,0,0],[-120,120],[-100,100],  "1 0 0"),
        #    N, robot_name, origin            ,mass, I,                  material,   geom_origin):
        link(2, robot_name,[0 ,0.00 , 0.0 , 0,0,0],4, [0.0,0,0.0,0.0,0,0.0], "Orange",[0,0,0 ]),

        joint(3,robot_name,[0.0 ,0.0  ,0.225   ,0,0,0],[-130,130],[-120,120],"1 0 0"),
        link(3, robot_name,[0.0 ,0.0  ,0.0  ,0,0,0],3,[0.0,0,0,0.0,0,0.0], "Orange",[0,0.0,0 ]),

        joint(4,robot_name,[0.0 ,0.0   ,0.0  ,0,0,0]),
        # joint(4,robot_name,[0.0 ,0.0   ,0.0  ,0,0,0],[-720,720],[-720,720]),
        link(4, robot_name,[0.0 ,0.0 ,0.0  ,0,0,0],2.7,[0.03,0,0,0.0,0,0.0], "Orange",[0,0,0 ]),

        joint(5,robot_name,[0.0 ,0.0 ,0.228855   ,0,0,0],[-130,130],[-120,120], "1 0 0"),
        link(5, robot_name,[0.0 ,0.0 ,0.0 ,0,0,0],1.7,[0.0,0,0,0.0,0,0.0], "Orange",[0,0,0]),

        # joint(6,robot_name,[0.0 ,0.0 ,0.047  ,0,0,0],[-720,720],[-720,720]),
        joint(6,robot_name,[0.0 ,0.0 ,0.047  ,0,0,0]),
        link(6, robot_name,[0.0 ,0.0 ,0.0  ,0,0,0],1.8,[0.0,0,0,0.0,0,0.0], "Orange",[0,0,0]),

        # joint(7,robot_name,[0.0 ,0.081 ,0.0607,-PI/2,PI,0],[-175,175],[-173,173]),
        # link(7, robot_name,[0.0 ,0.0   ,0.02  ,0,0,0],0.3,[0.001,0,0,0.001,0,0.001], "Grey",[0,0,-0.0005]),
        # Joint(  
        #     Parent(link=robot_name+"_link_7"),
        #     Child(link=robot_name+"_joint_ee_kuka"),
        #     Origin([0,0,0.045,PI,PI,PI]), 
        #     Axis(xyz="0 0 1"),
        #     name = robot_name+"_joint_ee",
        #     type = "fixed"),
        # Link(robot_name+"_link_ee_kuka"),

        Joint("tool0_joint",
            Parent(link=robot_name+"_link_6"),
            Child(robot_name+"_link_ee"),
            Origin([0,0,0.04,0,-PI/2,0]),
            type="fixed"),
        Link(robot_name+"_link_ee"))

    # ret(iiwa_gazebo.iiwa_gazebo(robot_name),iiwa_transmission.transmissions(hardware_interface))

    #Link 0 has some unique collision checking
    # ret[1](Self_collision_checking(
    #         Origin(xyz = "0 0 0", rpy = "0 0 0"),
    #         Geometry(Capsule(radius = 0.15, length = 0.25))))
 
    return ret



if __name__ == "__main__":
    
    # Default command line parameters 
    defaults = {"hardware_interface": "PositionJointInterface", "robot_name": "mantis"}

    # Extract all 'name:=value' arguments from the command line and add them
    # to kwargs
    kwargs = {}
    for a in sys.argv:
        vals = a.split(":=")
        if len(vals) == 2:
            kwargs[vals[0]]=vals[1]

    # Make sure that parameters not passed in on commandline are set to their
    # default value.
    for k,v in defaults.items():
        if not k in kwargs:
            kwargs[k]=v

    # Build the robot structure
    mantis = Robot(
        # iiwa_materials.materials,
        Link("world"),
        mantis_main(parent="world",**kwargs),
        name = kwargs["robot_name"]
    )    


    f= open("mantis\\%s.urdf" % robot_name ,"w+")
    print(mantis, file=f)
    f.close() 
    print(mantis)
    # print (" leds %i" % i)
