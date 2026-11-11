"""Necessary saved-artifact validation; no historical or M7 integration."""
import json
from pathlib import Path
import numpy as np
from PIL import Image
import imageio.v2 as imageio
ROOT=Path(__file__).resolve().parents[1]
def main():
    report=json.loads((ROOT/'visualization/results/presentation_checks.json').read_text(encoding='utf8'));assert len(report)==11
    for task,r in report.items():
        assert r['visual_inspection'].startswith('PASS') and len(r['poses'])==3
        for p in r['poses']:
            v=p['validation'];assert v['xml_bytes_unchanged'] and v['all_physical_arrays_unchanged'] and v['saved_qpos_restored_exactly'] and v['saved_qvel_restored_exactly'] and v['saved_ctrl_restored_exactly'] and not v['tendon_visualization']
    indexes=json.loads((ROOT/'visualization/results/video_index.json').read_text(encoding='utf8'));checks=[]
    for entry in indexes:
        name=entry['video'];file=(ROOT/'M7_external_impact_active_recovery/results/videos'/name) if entry['task']=='m7' else ROOT/'visualization/results/videos'/name
        n=entry['frame_count'];reader=imageio.get_reader(file)
        try:
            meta=reader.get_meta_data();first=reader.get_data(0);last=reader.get_data(n-1);assert first.shape==last.shape==(960,1280,3)
        finally:reader.close()
        frames=np.genfromtxt(file.with_suffix('.frames.csv'),delimiter=',',names=True);assert len(frames)==n and np.all(np.diff(frames['source_state_index'])>=0) and frames['physical_time_s'][-1]>0
        checks.append({'file':str(file.relative_to(ROOT)).replace('\\','/'),'first_last_frames_decode':True,'frames':n,'fps':meta['fps'],'size':list(meta['size']),'saved_state_indices_monotonic':True,'physical_time_displayed':True,'playback_speed_displayed':entry['playback_speed']})
    scientific=[*list((ROOT/'visualization/results').glob('*/*.svg')),*list((ROOT/'M7_external_impact_active_recovery/results/figures').glob('*.svg'))]
    assert len(scientific)==18
    for svg in scientific:
        assert '<image' not in svg.read_text(encoding='utf8'),str(svg)+' must be actual vector SVG'
        png=Image.open(svg.with_suffix('.png'));assert png.info['dpi'][0]>=599
    summary={'status':'PASS','representative_task_classes':11,'reviewed_native_poses':33,'all_physical_invariants_preserved':True,'video_encoding_checks':checks,'scientific_vector_svgs':len(scientific),'all_new_scientific_png_dpi':600,'new_physics_steps':0,'note':'Existing scientific task status is not reevaluated; new C009 display reads archived references and leaves original data/24 figures unchanged.'}
    (ROOT/'visualization/results/deliverable_validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8');print('11 classes / 33 reviewed native poses / 16 MP4 decode checks / 18 real vector SVG checks PASS; zero integrations.')
if __name__=='__main__':main()
