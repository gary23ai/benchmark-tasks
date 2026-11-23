"""Original real-scale knife and passive desktop lever disposal station."""
import argparse,sys
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rdb_knife_refresh import layout as L,fixed_scene as F
def fmt(v):return '('+', '.join(f'{float(x):.12g}' for x in v)+')'
def material(name,color,friction):
 opacity='float inputs:opacity = 0.22' if name=='window' else ''
 return f'''    def Material "{name}" (prepend apiSchemas = ["PhysicsMaterialAPI", "MaterialBindingAPI"])
    {{
        float physics:staticFriction = {friction}
        float physics:dynamicFriction = {friction}
        float physics:restitution = 0
        token outputs:surface.connect = </Fixture/{name}/Surface.outputs:surface>
        def Shader "Surface" {{ uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = {fmt(color)}
            float inputs:roughness = 0.35
            {opacity}
            token outputs:surface
        }}
    }}
'''
def mesh(name,vertices,mat='metal'):
 vertices=np.asarray(vertices,float);hull=ConvexHull(vertices);faces=[]
 for face,eq in zip(hull.simplices,hull.equations):
  f=list(map(int,face));a,b,c=vertices[f]
  if np.dot(np.cross(b-a,c-a),eq[:3])<0:f.reverse()
  faces+=f
 return f'''            def Mesh "{name}" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"])
            {{
                point3f[] points = [{', '.join(fmt(v*1000) for v in vertices)}]
                int[] faceVertexCounts = {[3]*len(hull.simplices)}
                int[] faceVertexIndices = {faces}
                uniform token subdivisionScheme = "none"
                token physics:approximation = "convexHull"
                double3 xformOp:scale = (0.001, 0.001, 0.001)
                uniform token[] xformOpOrder = ["xformOp:scale"]
                rel material:binding = </Fixture/{mat}>
                rel material:binding:physics = </Fixture/{mat}>
                float physxCollision:contactOffset = 0.00002
                float physxCollision:restOffset = 0
            }}
'''
def box(name,size,center=(0,0,0),mat='plastic'):
 s=np.asarray(size)/2;c=np.asarray(center)
 return mesh(name,[c+np.array((x,y,z))*s for x in (-1,1) for y in (-1,1) for z in (-1,1)],mat)
def body(name,pose,mass,com,inertia,parts,axes=(1,0,0,0)):
 pos=np.asarray(pose[:3])+[0,0,F.TOP];q=pose[3:]
 return f'''        def Xform "{name}" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"])
        {{
            bool physics:rigidBodyEnabled = 1
            bool physics:kinematicEnabled = 0
            float physics:mass = {mass}
            point3f physics:centerOfMass = {fmt(com)}
            float3 physics:diagonalInertia = {fmt(inertia)}
            quatf physics:principalAxes = {fmt(axes)}
            float physxRigidBody:linearDamping = 0.01
            float physxRigidBody:angularDamping = 0.01
            float physxRigidBody:sleepThreshold = 0
            bool physxRigidBody:enableCCD = 1
            bool physxRigidBody:enableSpeculativeCCD = 1
            int physxRigidBody:solverPositionIterationCount = 64
            int physxRigidBody:solverVelocityIterationCount = 16
            float physxContactReport:threshold = 0
            double3 xformOp:translate = {fmt(pos)}
            quatf xformOp:orient = {fmt((q[3],*q[:3]))}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
'''+parts+'        }\n'
def staticbox(name,size,pos,mat):
 return f'''        def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI", "PhysxCollisionAPI"])
        {{
            double size = 1
            double3 xformOp:translate = {fmt(np.array(pos)+[0,0,F.TOP])}
            double3 xformOp:scale = {fmt(size)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            rel material:binding = </Fixture/{mat}>
            rel material:binding:physics = </Fixture/{mat}>
            float physxCollision:contactOffset = 0.00002
            float physxCollision:restOffset = 0
        }}
'''
def usd():
 from scipy.spatial.transform import Rotation
 knife=box('steel_sheath_and_handle',(.115,.013,.0056),(-.0575,0,0),'dark')
 knife+=box('locked_slider_button',(.021,.010,.0018),(-.080,0,.0037),'plastic')
 knife+=mesh('remaining_blade',[(x,y,z) for y in (-.0045,.0045) for x in (-.005,L.SCORE_X+L.SCORE_SLOPE*y) for z in (-.00019,.00019)],'metal')
 # Box and original broad desktop dock form one rigid moulded assembly.
 case=box('original_dock_base',(.066,.0665,.003),(0,.00925,.0015),'plastic')
 case+=box('collection_floor',(.062,.038,.002),(0,0,.003),'plastic')
 case+=box('box_right_side',(.002,.038,.108),(.030,0,.058),'plastic')
 case+=box('box_left_side_lower',(.002,.038,.071),(-.030,0,.0395),'plastic')
 case+=box('box_left_side_upper',(.002,.027,.037),(-.030,.0055,.0935),'plastic')
 case+=box('box_back',(.062,.002,.108),(0,.018,.058),'plastic')
 case+=box('front_window_lower',(.062,.002,.071),(0,-.018,.0395),'window')
 case+=box('front_window_upper',(.043,.002,.037),(.0095,-.018,.0935),'window')
 # Actual through guide. Neither sheath nor blade is attached to this box.
 case+=box('M_guide_right',(.0235,.038,.007),(.01925,0,.1125),'plastic')
 case+=box('M_guide_left_inner',(.0045,.038,.007),(-.00975,0,.1125),'plastic')
 case+=box('M_guide_left_outer',(.019,.025,.007),(-.0215,.0065,.1125),'plastic')
 for i,y in enumerate((-.0115,.0115)):case+=box('M_guide_face_'+str(i),(.015,.015,.007),(0,y,.1125),'metal')
 # A shoulder supports the end of the stainless sheath while the narrow
 # steel blade protrudes below; two ledges leave the blade slot open.
 for i,x in enumerate((-.00675,.00675)):case+=box('sheath_depth_stop_'+str(i),(.0015,.008,.001),(x,0,.1085),'metal')
 # Independent original dock storage pocket behind the blade container.
 case+=box('dock_storage_floor',(.062,.020,.002),(0,.029,.004),'plastic')
 case+=box('dock_storage_back',(.062,.002,.055),(0,.040,.0325),'plastic')
 for i,x in enumerate((-.030,.030)):case+=box('dock_storage_side_'+str(i),(.002,.024,.055),(x,.029,.0325),'plastic')
 # Front thumb lever turns about its estimated horizontal lower pin. Its inner
 # tooth presses the segment below the score while the sheath stays seated.
 h=L.HINGE_LOCAL
 lever=box('front_thumb_flap',(.020,.002,.036),np.array((-.023,-.027,.094))-h,'metal')
 lever+=box('external_link',(.003,.020,.003),np.array((-.023,-.018,.079))-h,'metal')
 lever+=box('internal_crossbar',(.026,.004,.003),np.array((-.012,-.0075,.081))-h,'metal')
 lever+=box('internal_press_tooth',(.012,.002,.004),np.array((0,-.0033,.1005))-h,'metal')
 lever+=box('tooth_stem',(.0025,.004,.022),np.array((-.00675,-.004,.091))-h,'metal')
 lever+=box('pivot_web',(.004,.008,.0105),np.array((-.023,-.006,.07525))-h,'metal')
 m=L.MASSES[L.SEGMENT];vy=L.BLADE_WIDTH**2/12;vz=L.BLADE_THICKNESS**2/12
 vx=L.SEGMENT_LENGTH**2/12+L.SCORE_SLOPE**2*vy;xy=L.SCORE_SLOPE*vy
 tensor=m*np.array(((vy+vz,-xy,0),(-xy,vx+vz,0),(0,0,vx+vy)))
 inertia,basis=np.linalg.eigh(tensor)
 if np.linalg.det(basis)<0:basis[:,0]*=-1
 q=Rotation.from_matrix(basis).as_quat()
 actors=body('blade_segment',L.START_POSES[L.SEGMENT],m,(0,0,0),inertia,mesh('spent_segment',L.segment_vertices(),'dull'),axes=(q[3],*q[:3]))
 actors+=body('case',L.START_POSES[L.CASE],L.MASSES[L.CASE],(0,.005,.044),(1.5e-4,1.5e-4,8.5e-5),case)
 actors+=body('knife',L.START_POSES[L.KNIFE],L.MASSES[L.KNIFE],(-.057,0,0),(3.0e-7,2.4e-5,2.4e-5),knife)
 actors+=body('lever',L.START_POSES[L.LEVER],L.MASSES[L.LEVER],(-.015,-.018,.022),(3.0e-6,2.0e-6,1.8e-6),lever)
 joints=f'''    def PhysicsFixedJoint "scored_break"
    {{
        rel physics:body0 = </Fixture/Actors/knife>
        rel physics:body1 = </Fixture/Actors/blade_segment>
        point3f physics:localPos0 = ({L.SCORE_X},0,0)
        point3f physics:localPos1 = ({-L.SEGMENT_LENGTH/2},0,0)
        quatf physics:localRot0 = (1,0,0,0)
        quatf physics:localRot1 = (1,0,0,0)
        float physics:breakTorque = {L.BREAK_TORQUE}
        float physics:breakForce = {L.BREAK_FORCE}
        bool physics:excludeFromArticulation = 1
        bool physics:collisionEnabled = 0
    }}
    def PhysicsRevoluteJoint "thumb_lever_hinge"
    {{
        rel physics:body0 = </Fixture/Actors/case>
        rel physics:body1 = </Fixture/Actors/lever>
        point3f physics:localPos0 = {fmt(L.HINGE_LOCAL)}
        point3f physics:localPos1 = (0,0,0)
        token physics:axis = "X"
        float physics:lowerLimit = -25
        float physics:upperLimit = 0
        bool physics:excludeFromArticulation = 1
        bool physics:collisionEnabled = 1
    }}
'''
 props=staticbox('cutting_mat',(.300,.200,.004),L.MAT_CENTER+[0,0,.002],'mat')
 return '#usda 1.0\n( defaultPrim="Fixture"\n metersPerUnit=1\n upAxis="Z"\n)\ndef Xform "Fixture"\n{\n'+material('metal',(.65,.68,.70),.3)+material('dark',(.15,.16,.18),.35)+material('dull',(.35,.36,.37),.3)+material('plastic',(.88,.85,.75),.7)+material('window',(.28,.44,.47),.3)+material('mat',(.08,.29,.20),.5)+'    def Scope "Actors"\n    {\n'+actors+'    }\n'+joints+props+'}\n'
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args();data=usd()
 if a.check:assert F.USD.read_text()==data
 else:F.USD.write_text(data)
