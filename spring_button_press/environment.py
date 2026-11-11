"""Task-local real slide/spring/contact assembly. Core remains unmodified."""
import copy, json, os, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
HERE=Path(__file__).resolve().parent
CORE=Path(os.environ.get('SOFTROBOT_CORE',r'D:\softrobot-agent-core'))
sys.path.insert(0,str(CORE))
from extensions.tendon_family.compiler import resolve
from extensions.tendon_family.mjcf import compile_xml, load_model, fmt
from extensions.tendon_family.generated_serial import generate_family

LABEL='Codex-assisted engineering design; not Main autonomous Study'

def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,a):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(a,indent=2,allow_nan=False)+'\n',encoding='utf8')

def compound(parts,total_mass):
    """Uniform-density primitive union with explicit non-overlap COM tensor."""
    volumes=np.array([p['volume'] for p in parts]);masses=total_mass*volumes/volumes.sum()
    centers=np.array([p['center'] for p in parts]);com=np.sum(centers*masses[:,None],axis=0)/total_mass
    inertia=np.zeros((3,3))
    for p,m,c in zip(parts,masses,centers):
        d=c-com;inertia+=m*np.array(p['unit_inertia'])+m*(d@d*np.eye(3)-np.outer(d,d))
    return {'mass_kg':total_mass,'com_m':com.tolist(),'inertia_com_kg_m2':inertia.tolist(),
        'uniform_density_kg_m3':float(total_mass/volumes.sum()),'primitive_masses_kg':masses.tolist()}

def tool_properties():
    # Base block ends at y=4 mm; stem starts there and ends at y=18 mm;
    # sphere center y=21 mm, r=3 mm. Primitives only touch, do not overlap.
    parts=[]
    for lengths,center in [([.014,.008,.008],[.007,0,0]),([.004,.014,.004],[.014,.011,0])]:
        x,y,z=lengths;parts.append({'center':center,'volume':x*y*z,
            'unit_inertia':np.diag([(y*y+z*z)/12,(x*x+z*z)/12,(x*x+y*y)/12]).tolist()})
    parts.append({'center':[.014,.021,0],'volume':4*np.pi*.003**3/3,
        'unit_inertia':(np.eye(3)*2*.003**2/5).tolist()})
    return compound(parts,.010)

def button_properties():
    parts=[]
    for radius,length,cy in [(.006,.003,0),(.002,.012,.0075)]:
        parts.append({'center':[0,cy,0],'volume':np.pi*radius**2*length,
            'unit_inertia':np.diag([(3*radius**2+length**2)/12,radius**2/2,(3*radius**2+length**2)/12]).tolist()})
    return compound(parts,.005)

def inertial(body,p):
    old=body.find('inertial')
    if old is not None:body.remove(old)
    I=np.array(p['inertia_com_kg_m2'])
    ET.SubElement(body,'inertial',mass=str(p['mass_kg']),pos=fmt(p['com_m']),
        fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))

def add_tool(tree):
    payload=tree.find('.//body[@name="payload"]')
    for geom in list(payload.findall('geom')):payload.remove(geom)
    inertial(payload,tool_properties())
    common={'density':'0','friction':'0 0 0','condim':'1','contype':'1','conaffinity':'13'}
    ET.SubElement(payload,'geom',name='tool_block',type='box',pos='.007 0 0',size='.007 .004 .004',rgba='.32 .35 .4 1',**common)
    ET.SubElement(payload,'geom',name='tool_bracket',type='box',pos='.014 .011 0',size='.002 .007 .002',rgba='.9 .55 .1 1',**common)
    ET.SubElement(payload,'geom',name='probe',type='sphere',pos='.014 .021 0',size='.003',rgba='.95 .55 .1 1',**common)
    ET.SubElement(payload,'site',name='probe_center',pos='.014 .021 0',size='.0008',rgba='0 1 0 1')

def baseline_tree():
    tree=ET.parse(ROOT/'C1_four_tendons/robot.xml').getroot()
    add_tool(tree)
    return tree

def reference_environment():
    tree=baseline_tree();m=load_model(xml=ET.tostring(tree,encoding='unicode'));d=mujoco.MjData(m)
    mujoco.mj_forward(m,d);p=d.site_xpos[m.site('probe_center').id].copy()
    center=p+np.array([0,.003+.005+.0015,0])
    return {'version':'spring_button_press_v1.0','label':LABEL,'gravity_m_s2':[0,0,-9.81],
        'mount_position_m':[0,0,.15],'reference_probe_center_m':p.tolist(),
        'button_center_unpressed_m':center.tolist(),'press_axis_world':[0,1,0],
        'initial_gap_m':.005,'cap_radius_m':.006,'cap_thickness_m':.003,
        'rod_radius_m':.002,'rod_length_m':.012,'button':button_properties(),
        'tool':tool_properties(),'spring_n_m':50.,'damping_ns_m':.20,
        'slide_range_m':[0,.003],'initial_slide_m':0.,'contact_friction':[0,0,0],
        'contact_dimension':1,'contact_solref':[.003,1.],
        'contact_solimp':[.98,.995,.0001],'position_rule':'C1 zero qpos, zero qvel, mj_forward; frozen for every candidate'}

def assemble(candidate=None,path=None):
    env=read(HERE/'environment.json') if (HERE/'environment.json').exists() else reference_environment()
    source=read(ROOT/'C1_four_tendons/design.json')
    if candidate is None:
        tree=baseline_tree();design=source['design'];physics=resolve(design,source['discretization']['data'])
    else:
        request=copy.deepcopy(source['generation']['request'])
        for i in range(2):
            original=[c for c in source['design']['components'] if c['id']==f'segment_{i}'][0]
            sections=copy.deepcopy(original['sections'])
            for station in sections:
                for key in station['section']['parameters']:station['section']['parameters'][key]*=candidate['section_scale']
            request['segments'][str(i)].update(length_m=candidate['lengths_m'][i],young_pa=candidate['young_pa'][i],sections=[s['section'] for s in sections])
        for span in request['spans']:span['radius_m']=candidate['tendon_radius_m']
        design,mesh,*_=generate_family({'space':source['generation']['space'],'request':request})
        physics=resolve(design,mesh)
        from io import BytesIO
        b=BytesIO();compile_xml(physics,{'timestep_s':.001,'gravity':[0,0,-9.81],
            'floor_id':'floor','floor_z_m':-.02,'mount_position':[0,0,.15],
            'mount_rotation':np.eye(3),'control':{'tension_execution_mode':'ideal_tension'}},None,b)
        tree=ET.fromstring(b.getvalue());add_tool(tree)
    opt=tree.find('option');opt.set('timestep','.001');opt.set('solver','Newton');opt.set('iterations','100');opt.set('tolerance','1e-10')
    # Enable native robot self-contact and all environment collision checks.
    for g in tree.findall('.//geom'):
        g.set('condim','1');g.set('friction','0 0 0');g.set('solref','.003 1');g.set('solimp','.98 .995 .0001')
        if g.get('name')=='floor':g.set('contype','8');g.set('conaffinity','1')
        else:g.set('contype','1');g.set('conaffinity','13')
    world=tree.find('worldbody');center=np.array(env['button_center_unpressed_m'])
    b=ET.SubElement(world,'body',name='button_slider',pos=fmt(center))
    ET.SubElement(b,'joint',name='button_slide',type='slide',axis='0 1 0',limited='true',
        range='0 .003',stiffness='50',springref='0',damping='.20',solreflimit='.003 1',solimplimit='.98 .995 .0001')
    inertial(b,env['button'])
    common={'density':'0','quat':'.7071067811865476 .7071067811865476 0 0',
        'friction':'0 0 0','condim':'1','solref':'.003 1','solimp':'.98 .995 .0001'}
    ET.SubElement(b,'geom',name='button_cap',type='cylinder',size='.006 .0015',
        rgba='.1 .72 .45 1',contype='4',conaffinity='1',**common)
    # Rod slides through an open hole; housing uses four rails, not a solid
    # colliding volume through which the slider would be forced to pass.
    ET.SubElement(b,'geom',name='button_rod',type='cylinder',pos='0 .0075 0',size='.002 .006',rgba='.65 .7 .75 1',contype='4',conaffinity='1',**common)
    housing=ET.SubElement(world,'body',name='button_housing',pos=fmt(center))
    for j,(x,z,sx,sz) in enumerate([(-.009,0,.002,.011),(.009,0,.002,.011),(0,-.009,.007,.002),(0,.009,.007,.002)]):
        ET.SubElement(housing,'geom',name=f'housing_rail_{j}',type='box',pos=fmt([x,.011,z]),size=fmt([sx,.01,sz]),rgba='.25 .28 .35 1',contype='8',conaffinity='1',friction='0 0 0',condim='1')
    ET.SubElement(housing,'geom',name='mount_plate',type='box',pos='0 .026 0',size='.018 .003 .018',rgba='.35 .38 .45 1',contype='8',conaffinity='1',friction='0 0 0',condim='1')
    # Visual pedestal is outside the task contact geometry.
    ET.SubElement(world,'geom',name='visual_base',type='box',pos='0 0 .015',size='.035 .035 .015',rgba='.4 .45 .5 1',contype='0',conaffinity='0')
    ET.SubElement(world,'geom',name='visual_mount',type='cylinder',pos='0 0 .083',size='.009 .053',rgba='.5 .55 .6 1',contype='0',conaffinity='0')
    ET.indent(tree);xml=ET.tostring(tree,encoding='unicode')
    if path:Path(path).parent.mkdir(parents=True,exist_ok=True);Path(path).write_text(xml,encoding='utf8')
    m=load_model(xml=xml)
    return m,physics,design

if __name__=='__main__':
    write(HERE/'environment.json',reference_environment())
    m,_,_=assemble(path=HERE/'environment.xml');d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    print(json.dumps({'nq':m.nq,'nu':m.nu,'initial_contacts':d.ncon,'environment':read(HERE/'environment.json')},indent=2))
