"""FamilySpace candidates and new physical tunnel; no historical XML writes."""
import copy,json,os,sys,hashlib
from io import BytesIO
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
H=Path(__file__).resolve().parent
CORE=Path(os.environ.get('SOFTROBOT_CORE','D:/softrobot-agent-core'))
def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def fmt(v):return ' '.join(format(float(x),'.17g') for x in v)
def assemble(name='baseline_fixed',candidate=None):
    task=read(H/'task.json');source=read(H.parent/'C1_four_tendons/design.json');req=copy.deepcopy(source['generation']['request'])
    if candidate:
        sys.path.insert(0,str(CORE))
        from extensions.tendon_family.generated_serial import generate_family
        from extensions.tendon_family.compiler import resolve
        from extensions.tendon_family.mjcf import compile_xml
        if candidate.get('six',False):
            req['tendon_count']=6
            # Retain all four original routes exactly; two extra independent
            # routes are declared through the existing FamilySpace API.
            req['spans'][0].update(count=3,angular_offsets_rad=[0,np.pi/6,np.pi/6])
            req['spans'][1].update(count=3,angular_offsets_rad=[0,-np.pi/6,-np.pi/12])
        for k,v in candidate.get('segments',{}).items():req['segments'][k].update(v)
        if 'section_scale' in candidate:
            for i in range(2):
                part=next(c for c in source['design']['components'] if c['id']==f'segment_{i}');sections=[]
                for station in part['sections']:
                    section=copy.deepcopy(station['section'])
                    for key in section['parameters']:section['parameters'][key]*=candidate['section_scale']
                    sections.append(section)
                req['segments'][str(i)]['sections']=sections
        if 'radius_m' in candidate:
            for span in req['spans']:span['radius_m']=candidate['radius_m']
        design,mesh,*_=generate_family({'space':source['generation']['space'],'request':req})
        physics=resolve(design,mesh);buf=BytesIO()
        compile_xml(physics,{'timestep_s':.0005,'gravity':[0,0,-9.81],'floor_id':'floor','floor_z_m':-.02,'mount_position':task['base_position_m'],'mount_rotation':np.eye(3),'control':{'tension_execution_mode':'ideal_tension'}},None,buf)
        tree=ET.fromstring(buf.getvalue())
    else:tree=ET.parse(H.parent/'C1_four_tendons/robot.xml').getroot();design=source['design'];mesh=source['discretization']['data']
    tree.find('option').set('timestep',str(task['physics_dt_s']))
    base=tree.find('.//body[@name="fixed_base"]');base.set('pos',fmt(task['base_position_m']));yaw=task['base_yaw_rad'];base.set('quat',fmt([np.cos(yaw/2),0,0,np.sin(yaw/2)]))
    # Environment collision bit 2 works even if original robot bit masks are 0.
    # Explicit pairs enable native wall contacts without changing robot geometry.
    robot=[g for b in tree.findall('.//body') if b.get('name')!='fixed_base' for g in b.findall('geom')]
    world=tree.find('worldbody');x0=task['tunnel']['entry_x_m'];x1=task['tunnel']['exit_x_m'];yc,zc=task['tunnel']['center_yz_m'];w,h=task['tunnel']['width_m'],task['tunnel']['height_m'];th=.004
    walls=[('left',[ (x0+x1)/2,yc-w/2-th/2,zc],[(x1-x0)/2,th/2,h/2+th]),('right',[(x0+x1)/2,yc+w/2+th/2,zc],[(x1-x0)/2,th/2,h/2+th]),('bottom',[(x0+x1)/2,yc,zc-h/2-th/2],[(x1-x0)/2,w/2,th/2]),('top',[(x0+x1)/2,yc,zc+h/2+th/2],[(x1-x0)/2,w/2,th/2])]
    contact=tree.find('contact')
    if contact is None:contact=ET.SubElement(tree,'contact')
    for label,pos,size in walls:
        wn='tunnel_'+label;ET.SubElement(world,'geom',name=wn,type='box',pos=fmt(pos),size=fmt(size),rgba='.45 .45 .45 1',contype='2',conaffinity='1',friction='0 0 0',condim='1')
        for g in robot:ET.SubElement(contact,'pair',geom1=g.get('name'),geom2=wn,condim='1',friction='0 0 0 0 0')
    folder=H/'cases'/name;folder.mkdir(parents=True,exist_ok=True);ET.indent(tree);(folder/'model.xml').write_text(ET.tostring(tree,encoding='unicode'),encoding='utf8')
    write(folder/'structure.json',{'design':design,'discretization':mesh,'generation_request':req,'source':'C1_four_tendons/design.json','candidate':candidate,'active_actuators':len(design['actuators']),'historical_robot_geometry_preserved':candidate is None})
    return mujoco.MjModel.from_xml_path(str(folder/'model.xml'))
