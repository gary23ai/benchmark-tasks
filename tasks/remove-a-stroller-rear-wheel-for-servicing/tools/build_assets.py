"""Build full-size stroller wheels, steel spindles and a passive spring retainer."""
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rdb_stroller_remove import layout as L,fixed_scene as F

def fmt(v):return '('+', '.join(f'{float(x):.10g}' for x in v)+')'


def material(name,color,roughness,friction,restitution=0.,metallic=0.):
    return f'''        def Material "{name}" (prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"])
        {{
            float physics:staticFriction = {friction}
            float physics:dynamicFriction = {friction}
            float physics:restitution = {restitution}
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {fmt(color)}
                float inputs:metallic = {metallic}
                float inputs:roughness = {roughness}
                token outputs:surface
            }}
        }}
'''


def cube(name,size,position,material,indent='        '):
    return f'''{indent}def Cube "{name}" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"])
{indent}{{
{indent}    double size = 1
{indent}    double3 xformOp:translate = {fmt(position)}
{indent}    double3 xformOp:scale = {fmt(size)}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{indent}    rel material:binding = </Fixture/Materials/{material}>
{indent}    rel material:binding:physics = </Fixture/Materials/{material}>
{indent}    float physxCollision:contactOffset = {L.CONTACT_OFFSET}
{indent}    float physxCollision:restOffset = 0
{indent}}}
'''


def cylinder(name,a,b,radius,material,collide=False,indent='            '):
    a,b=np.asarray(a),np.asarray(b);d=b-a;length=np.linalg.norm(d);d=d/length
    if d[2]<-.999999:q=np.array((1.,0.,0.,0.))
    else:q=np.r_[np.cross((0.,0.,1.),d),1.+d[2]];q/=np.linalg.norm(q)
    schemas='"MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"' if collide else '"MaterialBindingAPI"'
    return f'''{indent}def Cylinder "{name}" (prepend apiSchemas = [{schemas}])
{indent}{{
{indent}    double radius = {radius}
{indent}    double height = {length:.10g}
{indent}    uniform token axis = "Z"
{indent}    double3 xformOp:translate = {fmt((a+b)/2)}
{indent}    quatf xformOp:orient = {fmt((q[3],*q[:3]))}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
{indent}    rel material:binding = </Fixture/Materials/{material}>
{indent}    rel material:binding:physics = </Fixture/Materials/{material}>
{indent}}}
'''


def body(name,pose,mass,inertia,children,com=(0,0,0)):
    p=pose[:3]+(0,0,F.TOP);q=pose[3:]
    return f'''        def Xform "{name}" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsMassAPI", "PhysxContactReportAPI"])
        {{
            bool physics:rigidBodyEnabled = 1
            bool physics:kinematicEnabled = 0
            float physics:mass = {mass:.10g}
            point3f physics:centerOfMass = {fmt(com)}
            float3 physics:diagonalInertia = {fmt(inertia)}
            quatf physics:principalAxes = (1, 0, 0, 0)
            float physxRigidBody:linearDamping = {L.LINEAR_DAMPING}
            float physxRigidBody:angularDamping = {L.ANGULAR_DAMPING}
            float physxRigidBody:maxDepenetrationVelocity = 0.5
            int physxRigidBody:solverPositionIterationCount = 64
            int physxRigidBody:solverVelocityIterationCount = 16
            float physxRigidBody:sleepThreshold = 0
            float physxContactReport:threshold = 0
            double3 xformOp:translate = {fmt(p)}
            quatf xformOp:orient = {fmt((q[3],*q[:3]))}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
'''+children+'        }\n'



def prism(name,polygon,xmin,xmax,mat,indent='            '):
    pts=[(x,y,z) for x in (xmin,xmax) for y,z in polygon];n=len(polygon)
    faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
    return f'''{indent}def Mesh "{name}" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI"])
{indent}{{
{indent}    point3f[] points = [{', '.join(fmt(np.asarray(p)*1000) for p in pts)}]
{indent}    int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]
{indent}    int[] faceVertexIndices = [{', '.join(str(k) for f in faces for k in f)}]
{indent}    double3 xformOp:scale = (.001,.001,.001)
{indent}    uniform token[] xformOpOrder = ["xformOp:scale"]
{indent}    uniform token subdivisionScheme = "none"
{indent}    uniform token physics:approximation = "convexHull"
{indent}    float physxCollision:contactOffset = {L.CONTACT_OFFSET}
{indent}    float physxCollision:restOffset = 0
{indent}    rel material:binding = </Fixture/Materials/{mat}>
{indent}    rel material:binding:physics = </Fixture/Materials/{mat}>
{indent}}}
'''


def ring(name,inner,outer,length,mat,segments=64,xoff=0.,indent='            '):
    out=''
    # Facet inner apothem is precisely the specified bore radius.
    ir=inner/np.cos(np.pi/segments)
    for j in range(segments):
        a,b=2*np.pi*np.array((j,j+1))/segments
        poly=[(ir*np.sin(a),ir*np.cos(a)),(ir*np.sin(b),ir*np.cos(b)),(outer*np.sin(b),outer*np.cos(b)),(outer*np.sin(a),outer*np.cos(a))]
        out+=prism(f'{name}_{j}',poly,xoff-length/2,xoff+length/2,mat,indent)
    return out


def world_cube(name,size,p,mat):return cube(name,size,np.asarray(p)+(0,0,F.TOP),mat,'        ')

def world_ring(name,inner,outer,length,position,mat,segments=24):
 raw=ring(name,inner,outer,length,mat,segments=segments,indent='            ')
 return f'        def Xform "{name}_placement"\n        {{\n            double3 xformOp:translate = {fmt(np.asarray(position)+(0,0,F.TOP))}\n            uniform token[] xformOpOrder = ["xformOp:translate"]\n'+raw+'        }\n'

def axle_children():
 out=cylinder('axle_shank',(0,0,0),(L.GROOVE_START,0,0),L.AXLE_RADIUS,'steel',True)
 out+=cylinder('groove_root',(L.GROOVE_START,0,0),(L.GROOVE_END,0,0),L.GROOVE_RADIUS,'steel',True)
 out+=cylinder('distal_shank',(L.GROOVE_END,0,0),(L.AXLE_END-.005,0,0),L.AXLE_RADIUS,'steel',True)
 # The real insertion chamfer is represented explicitly by a 12-sided frustum.
 pts=[(x,r*np.sin(a),r*np.cos(a)) for x,r in [(L.AXLE_END-.005,.006),(L.AXLE_END,.002)] for a in np.linspace(0,2*np.pi,12,endpoint=False)]
 n=12;faces=[list(range(n-1,-1,-1)),list(range(n,2*n))]+[[j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n)]
 out+='''            def Mesh "insertion_chamfer" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI"])
            {
'''+f'                point3f[] points = [{", ".join(fmt(np.asarray(p)*1000) for p in pts)}]\n                int[] faceVertexCounts = [{", ".join(str(len(f)) for f in faces)}]\n                int[] faceVertexIndices = [{", ".join(str(k) for f in faces for k in f)}]\n'+'''                double3 xformOp:scale = (.001,.001,.001)
                uniform token[] xformOpOrder = ["xformOp:scale"]
                uniform token subdivisionScheme = "none"
                uniform token physics:approximation = "convexHull"
                float physxCollision:contactOffset = .00015
                float physxCollision:restOffset = 0
                rel material:binding = </Fixture/Materials/steel>
                rel material:binding:physics = </Fixture/Materials/steel>
            }
'''
 return out

def wheel_children(new):
 out=ring('tire',L.TIRE_INNER,L.RADIUS,L.TIRE_WIDTH,'rubber_new' if new else 'rubber_old',32)
 out+=ring('rim',.087,.106,.026,'rim',32)
 out+=cylinder('bearing_hub',(-.026,0,0),(.026,0,0),.018,'rim',True)
 for j in range(6):
  a=j*np.pi/3
  # Three opposite spoke pairs. Local box long axis follows the radius.
  p=(0,.052*np.cos(a),.052*np.sin(a))
  out+=f'            def Xform "spoke_pose_{j}"\n            {{\n                double3 xformOp:translate = {fmt(p)}\n                quatf xformOp:orient = {fmt((np.cos(a/2),np.sin(a/2),0,0))}\n                uniform token[] xformOpOrder = ["xformOp:translate","xformOp:orient"]\n'+cube('spoke',(.026,.075,.020),(0,0,0),'rim','                ')+'            }\n'
 # Color mark is ordinary spare-part identification, with no extra grip geometry.
 out+=cube('replacement_color_mark',(.0003,.020,.010),(-.0262,0,.004),'new_mark' if new else 'old_mark').replace(', "PhysicsCollisionAPI", "PhysxCollisionAPI"','')
 return out

def usd():
 actors=''
 for i,name in enumerate(L.LINKS):
  if name.endswith('axle'):
   m=L.AXLE_MASS;l=L.AXLE_END;r=L.AXLE_RADIUS
   inertia=(m*r*r/2,m*(3*r*r+l*l)/12,m*(3*r*r+l*l)/12)
   actors+=body(name,L.START_POSES[i],m,inertia,axle_children(),com=(l/2,0,0))
  elif name.endswith('wheel'):
   m=L.WHEEL_MASS;r1=L.TIRE_INNER;r2=L.RADIUS;w=L.TIRE_WIDTH
   inertia=(m*(r1*r1+r2*r2)/2,m*((r1*r1+r2*r2)/4+w*w/12),m*((r1*r1+r2*r2)/4+w*w/12))
   actors+=body(name,L.START_POSES[i],m,inertia,wheel_children(name.startswith('new')))
  else:
   children=cube('steel_retaining_tip',(.003,.005,.009),(0,0,L.PIN_TOP_LOCAL-.0045),'steel','            ')
   children+=cube('vertical_leg',(.006,.010,.012),(0,0,-.003),'steel','            ')
   children+=cube('cross_link',(.010,.055,.006),(0,.026,-.008),'steel','            ')
   children+=cube('release_tab',(.018,.018,.012),L.TAB_LOCAL,'release','            ')
   actors+=body(name,L.START_POSES[i],L.PIN_MASS,(.000015,.000004,.000015),children,com=(0,.025,-.004))
 hardware=world_cube('bench_clamp_base',(.11,.14,.012),(.53,0,.006),'stand')
 hardware+=world_cube('clamped_frame_leg',(.040,.040,.17),(.53,0,.097),'aluminium')
 # Actual rear-wheel receiver, bolted to a supported stroller frame member.
 hardware+=world_ring('front_bearing_socket',L.BORE_RADIUS,.023,.018,(L.BORE_FRONT+.009,0,L.CENTER[2]),'hub',24)
 hardware+=world_ring('back_bearing_socket',L.BORE_RADIUS,.023,L.BORE_BACK-(L.CENTER[0]+.092),((L.BORE_BACK+L.CENTER[0]+.092)/2,0,L.CENTER[2]),'hub',24)
 # Actual lower opening through the housing lets the steel retainer move radially.
 middle=ring('slotted_lock_housing',L.BORE_RADIUS,.023,.014,'hub',24,0.,indent='            ')
 import re
 for j in range(9,15):middle=re.sub(r'            def Mesh "slotted_lock_housing_'+str(j)+r'".*?            }\n','',middle,flags=re.S)
 hardware+=f'        def Xform "lock_housing_pose"\n        {{\n            double3 xformOp:translate = {fmt(L.CENTER+(.085,0,F.TOP))}\n            uniform token[] xformOpOrder = ["xformOp:translate"]\n'+middle+'        }\n'
 hardware+=world_cube('frame_to_hub',(.065,.027,.032),(.505,0,.110),'aluminium')
 for label,c in [('old',L.OLD_CRADLE)]:
  cheek_height=L.NEW_CRADLE_CHEEK_HEIGHT if label=='new' else L.OLD_CRADLE_CHEEK_HEIGHT
  hardware+=world_cube(label+'_cradle_floor',(.066,.27,L.CRADLE_FLOOR),(c[0],c[1],L.CRADLE_FLOOR/2),'cradle')
  for side in (-1,1):
   hardware+=world_cube(label+f'_cradle_cheek_{side+1}',(.006,.255,cheek_height),(c[0]+side*(L.CRADLE_INNER_X/2+.003),c[1],L.CRADLE_FLOOR+cheek_height/2),'cradle')
   hardware+=world_cube(label+f'_cradle_saddle_{side+1}',(.043,.024,.010),(c[0],c[1]+side*.044,.009),'cradle')
 joints=''
 for prefix in ['old']:
  joints+=f'''        def PhysicsRevoluteJoint "{prefix}_wheel_bearing" (prepend apiSchemas = ["PhysxJointAPI"])
        {{
            rel physics:body0 = </Fixture/Actors/{prefix}_axle>
            rel physics:body1 = </Fixture/Actors/{prefix}_wheel>
            point3f physics:localPos0 = (0,0,0)
            point3f physics:localPos1 = (0,0,0)
            uniform token physics:axis = "X"
            uniform bool physics:excludeFromArticulation = 1
            bool physics:collisionEnabled = 0
            # Ordinary free-body bearing: no articulation-only friction claim.
        }}
'''
 joints+=f'''        def PhysicsPrismaticJoint "spring_release_slider" (prepend apiSchemas = ["PhysxJointAPI", "PhysicsDriveAPI:linear"])
        {{
            rel physics:body1 = </Fixture/Actors/release_slider>
            point3f physics:localPos0 = {fmt(L.PIN_CENTER+(0,0,F.TOP))}
            point3f physics:localPos1 = (0,0,0)
            uniform token physics:axis = "Z"
            float physics:lowerLimit = {-L.PIN_STROKE}
            float physics:upperLimit = 0
            uniform bool physics:excludeFromArticulation = 1
            bool physics:collisionEnabled = 1
            float physxJoint:jointFriction = 0
            token drive:linear:physics:type = "force"
            float drive:linear:physics:stiffness = {L.PIN_SPRING}
            float drive:linear:physics:damping = {L.PIN_DAMPING}
            float drive:linear:physics:targetPosition = {L.PIN_TARGET}
            float drive:linear:physics:targetVelocity = 0
        }}
'''
 mats=material('steel',(.50,.54,.58),.25,.22,metallic=1.)+material('rim',(.12,.13,.15),.45,.45)+material('rubber_old',(.14,.12,.10),.8,.85)+material('rubber_new',(.025,.025,.025),.8,.85)+material('release',(.84,.14,.025),.45,.6)+material('aluminium',(.45,.48,.50),.3,.35,metallic=.8)+material('hub',(.075,.080,.085),.5,.25)+material('stand',(.08,.20,.27),.5,.5)+material('cradle',(.16,.34,.49),.6,.7)+material('old_mark',(.7,.25,.08),.7,.4)+material('new_mark',(.04,.62,.28),.7,.4)
 return '#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\ndef Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n'+actors+'    }\n    def Scope "Hardware"\n    {\n'+hardware+'    }\n    def Scope "Joints"\n    {\n'+joints+'    }\n    def Scope "Materials"\n    {\n'+mats+'    }\n}\n'

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args()
 if a.check:assert F.USD.read_text()==usd()
 else:F.USD.write_text(usd())
