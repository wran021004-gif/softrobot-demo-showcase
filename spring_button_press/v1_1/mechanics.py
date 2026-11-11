"""Task-local mechanical guide revision; reads immutable v1.0 environment."""
import copy,sys
from pathlib import Path
from io import BytesIO
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
HERE=Path(__file__).resolve().parent;BASE=HERE.parent;ROOT=BASE.parent
sys.path.insert(0,str(BASE))
import environment as v10
from environment import read,write,fmt,load_model,LABEL
from extensions.tendon_family.generated_serial import generate_family
from extensions.tendon_family.compiler import resolve
from extensions.tendon_family.mjcf import compile_xml
COLORS=['#1b9e77','#d95f02','#7570b3','#555555']
def rgba(s):return [int(s[i:i+2],16)/255 for i in [1,3,5]]+[1.]

def generate(candidate=None):
    source=read(ROOT/'C1_four_tendons/design.json')
    if candidate is None:return source['design'],source['discretization']['data'],None
    request=copy.deepcopy(source['generation']['request'])
    for i in range(2):
        part=next(c for c in source['design']['components'] if c['id']==f'segment_{i}')
        sections=[]
        for station in part['sections']:
            section=copy.deepcopy(station['section'])
            for key in section['parameters']:
                axis=0 if key in ['semi_y_m','width_y_m'] else 1
                section['parameters'][key]*=candidate['section_yz_scale'][i][axis]
            sections.append(section)
        request['segments'][str(i)].update(length_m=candidate['lengths_m'][i],young_pa=candidate['young_pa'][i],sections=sections)
    for i,span in enumerate(request['spans']):
        span['radius_m']=candidate['span_radii_m'][i]
        span['phase_rad']+=candidate.get('phase_rotation_rad',[0,0])[i]
    request['spans'][0]['angular_offsets_rad'][1]+=candidate.get('short_pair_offset_delta_rad',0.)
    design,mesh,*_=generate_family({'space':source['generation']['space'],'request':request})
    return design,mesh,request

def annulus_mesh(asset,name,ri,ro,thickness,n=32):
    verts=[]
    for x in [-thickness/2,thickness/2]:
        for r in [ri,ro]:
            for j in range(n):a=2*np.pi*j/n;verts.append([x,r*np.cos(a),r*np.sin(a)])
    faces=[]
    for j in range(n):
        k=(j+1)%n
        for a,b,c,d in [(j,k,n+k,n+j),(2*n+j,3*n+j,3*n+k,2*n+k),(n+j,n+k,3*n+k,3*n+j),(j,2*n+j,2*n+k,k)]:faces.extend([[a,b,c],[a,c,d]])
    ET.SubElement(asset,'mesh',name=name,vertex=fmt(verts),face=' '.join(str(v) for f in faces for v in f))

def add_guides(tree,m):
    cfg=read(HERE/'design_space.json')['guide_standard'] if (HERE/'design_space.json').exists() else {'hole_diameter_m':.0014,'outer_diameter_m':.004,'thickness_m':.001,'spoke_radius_m':.0005,'density_kg_m3':1200}
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);asset=tree.find('asset')
    ri=cfg['hole_diameter_m']/2;ro=cfg['outer_diameter_m']/2;th=cfg['thickness_m'];rho=cfg['density_kg_m3'];sr=cfg['spoke_radius_m']
    annulus_mesh(asset,'guide_eye_open_mesh',ri,ro,th)
    parents={e.get('name'):e for e in tree.findall('.//body')}
    joint_parent='guide_1'
    # The old 50 mm solid plate rendered the declared holes as solid material.
    # Preserve its supplied 2 g inertial, replace only that solid envelope by
    # a compact hub; newly added eye/spoke mass is explicit and separate.
    old=parents[joint_parent].find('geom')
    old.set('size','.002 .006 .006');old.set('contype','0');old.set('conaffinity','0')
    groups=[('base','fixed_base',[(i,f'tendon_{i}_point_0') for i in range(4)]),
        ('segment0_mid','segment_0_cell_6',[(i,f'tendon_{i}_point_1') for i in range(4)]),
        ('junction',joint_parent,[(0,'tendon_0_point_2'),(1,'tendon_1_point_2'),(2,'tendon_2_point_2'),(3,'tendon_3_point_2')]),
        ('segment1_mid','segment_1_cell_6',[(2,'tendon_2_point_3'),(3,'tendon_3_point_3')]),
        ('segment1_end','segment_1_cell_11',[(2,'tendon_2_point_4'),(3,'tendon_3_point_4')])]
    records=[]
    for group,parent_name,active in groups:
        bid=m.body(parent_name).id;R=d.xmat[bid].reshape(3,3);origin=d.xpos[bid]
        mapping={i:s for i,s in active};centers={i:R.T@(d.site_xpos[m.site(s).id]-origin) for i,s in active}
        xp=float(np.mean([p[0] for p in centers.values()]));radius=float(np.mean([np.linalg.norm(p[1:]) for p in centers.values()]))
        for i in range(4):
            if i not in centers:
                start=m.site(f'tendon_{i}_point_0').id;rel=d.site_xpos[start]-[0,0,.15];direction=rel[1:]/np.linalg.norm(rel[1:])
                centers[i]=np.array([xp,0,0])+R.T@np.r_[0,direction*radius]
        for i,p in sorted(centers.items()):
            name=f'guide_{group}_T{i}';body=ET.SubElement(parents[parent_name],'body',name=name,pos=fmt([0,0,0]))
            r=np.linalg.norm(p[1:]);unit=np.r_[0,p[1:]/r]
            # Eye annulus plus a radial solid cylindrical spoke, with tangent
            # end surfaces. Hole geometry is genuinely open in the mesh.
            start=np.array([p[0],0,0])+unit*.006;end=p-unit*ro;length=max(.0005,float(np.linalg.norm(end-start)));center=(start+end)/2
            eye_mass=rho*np.pi*(ro*ro-ri*ri)*th;spoke_mass=rho*np.pi*sr*sr*length
            eye_I=np.diag([eye_mass*(ri*ri+ro*ro)/2,eye_mass*(3*(ri*ri+ro*ro)+th*th)/12,eye_mass*(3*(ri*ri+ro*ro)+th*th)/12])
            parallel=spoke_mass*sr*sr/2;perp=spoke_mass*(3*sr*sr+length*length)/12
            spoke_I=perp*np.eye(3)+(parallel-perp)*np.outer(unit,unit)
            mass=eye_mass+spoke_mass;com=(eye_mass*p+spoke_mass*center)/mass;I=np.zeros((3,3))
            for c,mi,Ii in [(p,eye_mass,eye_I),(center,spoke_mass,spoke_I)]:
                z=c-com;I+=Ii+mi*(z@z*np.eye(3)-np.outer(z,z))
            v10.inertial(body,{'mass_kg':mass,'com_m':com.tolist(),'inertia_com_kg_m2':I.tolist()})
            common={'contype':'0','conaffinity':'0','density':'0','group':'3','rgba':fmt(rgba(COLORS[i]))}
            ET.SubElement(body,'geom',name=name+'_eye',type='mesh',mesh='guide_eye_open_mesh',pos=fmt(p),**common)
            ET.SubElement(body,'geom',name=name+'_spoke',type='cylinder',fromto=fmt(np.r_[start,end]),size=str(sr),**common)
            ET.SubElement(body,'site',name=name+'_hole_center',pos=fmt(p),size='.00015',rgba=fmt(rgba(COLORS[i])))
            records.append({'name':name,'group':group,'parent_body':parent_name,'tendon':f'T{i}','routing_site':mapping.get(i),
                'function':'active guide/anchor' if i in mapping else 'unused spare eye; tendon terminates at junction',
                'center_local_m':p.tolist(),'mass_kg':mass,'com_local_m':com.tolist(),'inertia_com_local_kg_m2':I.tolist(),
                'fixed_to_world':group=='base','collision_geometry':False,'hole_diameter_m':ri*2})
    return records

def assemble(candidate=None,folder=None):
    design,mesh,request=generate(candidate);physics=resolve(design,mesh)
    # Read the exact frozen environment XML, replacing only the robot subtree,
    # assets, tendons and actuators. Button/environment nodes are untouched.
    tree=ET.parse(BASE/'environment.xml').getroot();b=BytesIO()
    compile_xml(physics,{'timestep_s':.001,'gravity':[0,0,-9.81],'floor_id':'floor','floor_z_m':-.02,
        'mount_position':[0,0,.15],'mount_rotation':np.eye(3),'control':{'tension_execution_mode':'ideal_tension'}},None,b)
    robot=ET.fromstring(b.getvalue());v10.add_tool(robot)
    world=tree.find('worldbody');world.remove(world.find('body[@name="fixed_base"]'));world.append(robot.find('worldbody/body[@name="fixed_base"]'))
    for tag in ['asset','tendon','actuator']:
        old=tree.find(tag)
        if old is not None:tree.remove(old)
        tree.append(robot.find(tag))
    base=world.find('body[@name="fixed_base"]')
    for g in base.findall('.//geom'):
        g.set('condim','1');g.set('friction','0 0 0');g.set('solref','.003 1');g.set('solimp','.98 .995 .0001');g.set('contype','1');g.set('conaffinity','13')
    before=load_model(xml=ET.tostring(tree,encoding='unicode'));guides=add_guides(tree,before)
    for i,t in enumerate(tree.find('tendon')):t.set('rgba',fmt(rgba(COLORS[i])));t.set('width','.0005')
    ET.indent(tree);xml=ET.tostring(tree,encoding='unicode');m=load_model(xml=xml)
    robot_mass=sum(m.body_mass[i] for i in range(1,m.nbody) if m.body(i).name not in ['button_slider','button_housing'])
    guide_mass=sum(g['mass_kg'] for g in guides);moving=sum(g['mass_kg'] for g in guides if not g['fixed_to_world'])
    flexmass=sum(p['mass_kg'] for p in physics['parts'] if p['section_properties'])
    info={'guides':guides,'added_guide_mass_kg':guide_mass,'added_moving_guide_mass_kg':moving,
        'original_flexible_mass_kg':flexmass,'original_rigid_mass_excluding_tool_kg':robot_mass-guide_mass-flexmass-.010,
        'tool_mass_kg':.010,'total_structure_mass_kg':float(robot_mass),'request':request,'candidate':candidate,
        'guide_model':'real open-hole mesh; explicit mass/inertia; no collision/friction on guide meshes; routing enforced by ideal spatial tendon sites'}
    if folder:
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);(folder/'environment.xml').write_text(xml,encoding='utf8');write(folder/'mechanics.json',info);write(folder/'design.json',design)
    return m,info

def precheck():
    p=HERE/'precheck.json'
    if p.exists():raise RuntimeError('The single assembly precheck is already consumed')
    m,info=assemble(folder=HERE/'baseline_model');d=mujoco.MjData(m);mujoco.mj_forward(m,d);errors=[]
    for g in info['guides']:
        if g['routing_site']:
            errors.append(np.linalg.norm(d.site_xpos[m.site(g['routing_site']).id]-d.site_xpos[m.site(g['name']+'_hole_center').id]))
    assert max(errors)<1e-12 and d.ncon==0
    write(p,{'assembly_prechecks':1,'physics_steps':0,'load_success':True,'initial_contacts':d.ncon,
        'max_routing_site_hole_center_error_m':float(max(errors)),'eyes':len(info['guides']),
        'active_eyes':sum(g['routing_site'] is not None for g in info['guides']),
        'added_guide_mass_kg':info['added_guide_mass_kg'],'added_moving_guide_mass_kg':info['added_moving_guide_mass_kg'],
        'positive_inertias':all(np.linalg.eigvalsh(g['inertia_com_local_kg_m2']).min()>0 for g in info['guides'])})
    print(read(p))
if __name__=='__main__':precheck()
