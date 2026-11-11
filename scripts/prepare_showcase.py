"""Add visual-only scene XML and sample two accepted P3 recordings; no physics steps."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path(r'D:\softrobot-benchmark-exports\base_only_accepted_20261028')
def write(p,v):p.write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
def decorate_xml(source,dest,passive=False):
    tree=ET.parse(source);r=tree.getroot();w=r.find('worldbody')
    asset=r.find('asset')
    if asset is None:asset=ET.SubElement(r,'asset')
    ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.78 .86 .94',rgb2='.97 .98 1',width='512',height='512')
    vis=r.find('visual')
    if vis is None:vis=ET.SubElement(r,'visual')
    ET.SubElement(vis,'global',offwidth='1600',offheight='1000')
    ET.SubElement(vis,'rgba',haze='.92 .95 .98 1')
    ET.SubElement(vis,'headlight',diffuse='.7 .7 .7',ambient='.4 .4 .4',specular='.15 .15 .15')
    for x in r.findall('.//geom'):
        n=x.get('name','')
        if n=='floor':x.set('rgba','.91 .94 .97 1')
        elif n.startswith('segment_'):x.set('rgba','.40 .69 .86 1')
        elif n.startswith(('guide_','connector_')):x.set('rgba','.4 .44 .49 1')
        elif n=='payload_shape':x.set('rgba','.45 .49 .54 1')
    for t in r.findall('./tendon/spatial'):t.set('rgba','.3 .34 .4 1')
    for s in r.findall('.//site'):
        if s.get('name') in ['task_target','tip_site']:s.set('rgba','0 0 0 0')
    def geom(parent,name,**kwargs):
        return ET.SubElement(parent,'geom',name='visual_'+name,density='0',contype='0',conaffinity='0',group='3',**kwargs)
    if passive:
        geom(w,'base',type='box',pos='-.105 0 .01',size='.065 .055 .01',rgba='.35 .39 .44 1')
        geom(w,'pillar',type='capsule',fromto='-.105 0 .02 -.105 0 .5',size='.011',rgba='.52 .56 .61 1')
        geom(w,'crossbar',type='capsule',fromto='-.105 0 .5 0 0 .5',size='.014',rgba='.4 .44 .49 1')
    else:
        z=float(r.find('.//body[@name="fixed_base"]').get('pos').split()[2])
        geom(w,'base',type='box',pos='-.03 0 -.005',size='.065 .055 .012',rgba='.35 .39 .44 1')
        geom(w,'pillar',type='capsule',fromto=f'-.03 0 .005 -.03 0 {z}',size='.014',rgba='.52 .56 .61 1')
        geom(w,'root_mount',type='capsule',fromto=f'-.03 0 {z} 0 0 {z}',size='.018',rgba='.4 .44 .49 1')
    for b in r.findall('.//body'):
        n=b.get('name','')
        if n.startswith('segment_'):
            child=b.find('body')
            length=float(child.get('pos').split()[0]) if child is not None else .02
            # Existing physical mesh remains. Capsule is only an outer visual sleeve.
            geom(b,n+'_sleeve',type='capsule',fromto=f'.001 0 0 {max(.002,length-.001)} 0 0',size='.0085' if not passive else '.010',rgba='.45 .75 .91 .8')
        if n=='payload' and not passive:
            geom(b,'probe_shaft',type='capsule',fromto='0 0 0 .014 0 0',size='.0023',rgba='.25 .3 .36 1')
            geom(b,'probe_head',type='sphere',pos='.014 0 0',size='.0035',rgba='.1 .9 .4 1')
    ET.SubElement(w,'light',pos='.2 -.5 1',dir='0 0 -1',diffuse='.7 .7 .7',directional='true')
    ET.indent(r);tree.write(dest,encoding='unicode')
def main():
    decorate_xml(ROOT/'M1_reach_hold'/'robot.xml',ROOT/'M1_reach_hold'/'scene.xml')
    decorate_xml(ROOT/'figure_eight_tracking'/'robot.xml',ROOT/'figure_eight_tracking'/'scene.xml')
    p=SOURCE/'P3_passive_progressive_stiffness';out=ROOT/'P3_progressive_stiffness'
    src=json.loads((p/'summary.json').read_text(encoding='utf8'))
    summary={k:src[k] for k in ['status','scope','label','tendons','segments','cells_per_segment','full_length_m','payout_lengths_m','force_point_binding','stage_names']};summary['cases']={}
    for case in ['plus_x','minus_x']:
        folder=out/case;folder.mkdir(exist_ok=True);original=p/'cases'/case
        (folder/'robot.xml').write_bytes((original/'model.xml').read_bytes())
        (folder/'original_result.json').write_bytes((original/'result.json').read_bytes())
        decorate_xml(folder/'robot.xml',folder/'scene.xml',passive=True)
        path=original/'trajectory.csv'
        columns=path.open(encoding='utf8').readline().strip().split(',');data=np.loadtxt(path,delimiter=',',skiprows=1)
        stages=data[:,columns.index('stage')]
        changes=np.flatnonzero(np.diff(stages))+1
        ids=np.unique(np.r_[0,np.arange(0,len(data),80),changes-1,changes,len(data)-1])
        np.savez_compressed(folder/'display_replay.npz',columns=columns,rows=data[ids])
        summary['cases'][case]=dict(evaluation=src['cases'][case]['evaluation'],original_trajectory=str(path),original_steps=len(data),display_rows=len(ids),sampling='DISPLAY REPLAY ONLY: every 80th original row plus stage boundaries and endpoints; original timestamps retained; not full physical timestep data',nominal_display_hz=100)
    write(out/'summary.json',summary)
    for f in ['task.json','design.json']:(out/f).write_bytes((p/f).read_bytes())
    write(ROOT/'provenance.json',dict(source_root=str(SOURCE),M1='Original accepted XML, task, controls, full timestep CSVs; separate independent initial states',figure_eight='New single segment four-tendon design. New independent task and solved full timestep recording. M4 unchanged.',P3='Original three segments and two passive fixed-payout tendons. plus_x and minus_x accepted records sampled for display only. No integration.',visuals='scene.xml adds only density=0 contype=0 conaffinity=0 group=3 geometry, lights and colors. robot.xml is physics baseline.',omitted='Optional PHS2D skipped to keep scope focused on three requested demos'))
    print('Scenes and P3 display samples prepared; zero dynamics integration')
if __name__=='__main__':main()
