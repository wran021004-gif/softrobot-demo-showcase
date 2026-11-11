"""Add unified presentation/M7/C009 to the authorized frozen export root.

Never overwrite original scientific files. The explicitly requested root README
upgrade first keeps a byte-exact historical copy. No deletes, moves or simulation.
"""
import argparse,hashlib,json,shutil,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE_SHA='bd4fe89f13b286f127f1fc336a860318be109119'
EXPECTED_RESULT_SHA='9928d7f71233ee931005bfe1f8b4cc5d34cdf61fd315d91b217e65686f5c7c0f'
def read(p):return json.loads(Path(p).read_text(encoding='utf8'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        while chunk:=f.read(4*1024*1024):h.update(chunk)
    return h.hexdigest()
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf8').strip()
def snapshot(root,label):
    result={};total=0;files=sorted(p for p in root.rglob('*') if p.is_file())
    for i,p in enumerate(files):
        rel=p.relative_to(root).as_posix()
        if rel=='export_manifest_v2.json':continue
        result[rel]=sha(p);total+=p.stat().st_size
        if (i+1)%150==0:print(label,i+1,'files,',round(total/1e9,3),'GB hashed',flush=True)
    return result,total
def legacy_audit(manifest,hashes):
    files=manifest.get('files',{});missing=[p for p in files if p not in hashes];changed=[p for p,v in files.items() if p in hashes and hashes[p]!=v]
    return {'status':'PASS' if not missing and not changed else 'FAIL','recorded_files':len(files),'missing':missing,'changed':changed,'scope':'historical manifest entries only; never a claim that the current complete bundle is covered'}
def main():
    p=argparse.ArgumentParser();p.add_argument('--destination',type=Path,required=True);p.add_argument('--apply',action='store_true');a=p.parse_args();dest=a.destination.resolve()
    if not a.apply:raise SystemExit('Explicit --apply required')
    if not (dest/'M2/robot.xml').is_file() or not (dest/'manifest.json').is_file():raise RuntimeError('Expected frozen benchmark root not found')
    if (dest/'export_manifest_v2.json').exists():raise RuntimeError('Refuse to overwrite a prior v2 delivery; create a new delivery version instead')
    for name in ['export_audit_v2.json','unified_source_provenance_v2.json']:
        if (dest/name).exists():raise RuntimeError('Refuse to overwrite an existing versioned report: '+name)
    if git('diff',SOURCE_SHA,'--','spring_button_press/v1_2'):raise RuntimeError('Frozen C009 source differs from specified commit')
    files=git('ls-tree','-r','--name-only',SOURCE_SHA,'--','spring_button_press/v1_2').splitlines()
    required=['candidates/C009/result.json','candidates/FINAL_C009/replay_check.json','controller.py','run.py','replay.py','README.md','results/figures/index.json']
    source=ROOT/'spring_button_press/v1_2'
    assert all((source/x).is_file() for x in required)
    assert sha(source/'candidates/C009/result.json')==EXPECTED_RESULT_SHA
    assert read(source/'candidates/C009/result.json')['status']=='PASS'
    assert read(source/'candidates/FINAL_C009/replay_check.json')['identical']
    assert len(read(source/'results/figures/index.json')['independent_figures'])==24
    print('Verified source commit,',len(files),'version files, result SHA, PASS, independent closed loop and 24 figure entries.',flush=True)
    before,before_bytes=snapshot(dest,'Frozen pre-export snapshot');old_manifest=read(dest/'manifest.json');legacy_before=legacy_audit(old_manifest,before)
    added=[]
    def safe_target(relative):
        target=(dest/relative).resolve()
        if not target.is_relative_to(dest):raise RuntimeError('Path escapes authorized export root')
        return target
    def copy_new(src,relative):
        target=safe_target(relative);target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():
            if sha(target)!=sha(src):raise RuntimeError('Refuse to overwrite historical/existing file: '+str(relative))
            return
        shutil.copyfile(src,target);added.append(str(relative).replace('\\','/'))
    def tree(src,relative):
        for f in sorted(src.rglob('*')):
            if f.is_file() and '__pycache__' not in f.parts and f.suffix!='.pyc':copy_new(f,Path(relative)/f.relative_to(src))
    tree(ROOT/'visualization','visualization');tree(ROOT/'M7_external_impact_active_recovery','M7_external_impact_active_recovery')
    for relative in files:copy_new(ROOT/relative,relative)
    # Frozen dependencies of the original v1.2 scripts, not scientific reruns.
    parent=ROOT/'spring_button_press'
    for f in parent.iterdir():
        if f.is_file() and (f.suffix in ['.py','.json','.xml'] or f.name in ['task_spec.md','README.md']):copy_new(f,Path('spring_button_press')/f.name)
    for f in (parent/'v1_1').iterdir():
        if f.is_file() and f.suffix in ['.py','.json']:copy_new(f,Path('spring_button_press/v1_1')/f.name)
    tree(parent/'v1_1/candidates/S4','spring_button_press/v1_1/candidates/S4');tree(parent/'candidates/B5','spring_button_press/candidates/B5')
    copy_new(ROOT/'C1_four_tendons/design.json','C1_four_tendons/design.json')
    copy_new(source/'candidates/C009/environment.xml','spring_button_press/v1_2/model.xml')
    copy_new(ROOT/'requirements_visualization_v2.txt','requirements_visualization_v2.txt')
    copy_new(dest/'README.md','history/README_before_unified_visualization_v2.md')
    shutil.copyfile(ROOT/'README.md',dest/'README.md') # explicitly authorized root index revision
    source_copies={relative:sha(dest/relative) for relative in files}
    assert all(source_copies[x]==sha(ROOT/x) for x in files)
    export_note='''# C009 v1.2 frozen export\n\nThe original v1.2 README/configuration/source/results/24 PNG+SVG are preserved byte for byte from commit bd4fe89f13b286f127f1fc336a860318be109119. model.xml is an extra byte-exact alias of candidates/C009/environment.xml. No C009 optimization, physics integration or evaluator rerun occurred.\n\nFrom the export root: `python -B -m visualization.mujoco_renderer spring_button --loop` or `python -B -m visualization.video_export spring_button --output visualization/results/videos/user_spring.mp4`. These portable saved-state entries require only the visualization dependencies.\n\nThe original run.py and mechanics helpers are also preserved with frozen parent v1.0/v1.1/C1 dependencies. Explicit future physical reproduction through those original scripts retains their original read-only Core dependency D:\\softrobot-agent-core; this delivery does not secretly rerun them.\n'''
    note=dest/'spring_button_press/EXPORT_README_v1_2.md';note.write_text(export_note,encoding='utf8');added.append(note.relative_to(dest).as_posix())
    provenance={'source_repository':'wran021004-gif/softrobot-demo-showcase','source_branch':'codex/button-control-autosearch-v1-2-20261111','source_commit':SOURCE_SHA,'development_branch':'codex/external-impact-active-recovery-20261111','source_v1_2_file_count':len(files),'source_v1_2_copies_all_sha256_match':True,'source_v1_2_files_sha256':source_copies,'c009_result_sha256':EXPECTED_RESULT_SHA,'c009_result_git_blob_sha1':git('rev-parse',SOURCE_SHA+':spring_button_press/v1_2/candidates/C009/result.json'),'model_alias_sha256':sha(dest/'spring_button_press/v1_2/model.xml'),'model_alias_source':'spring_button_press/v1_2/candidates/C009/environment.xml','m7_original_m2_model_sha256':sha(dest/'M2/robot.xml'),'m7_export_model_sha256':sha(dest/'M7_external_impact_active_recovery/model.xml'),'history_physics_steps_this_round':0,'m7_physics_budget':read(ROOT/'M7_external_impact_active_recovery/results/budget_usage.json')}
    assert provenance['m7_original_m2_model_sha256']==provenance['m7_export_model_sha256']
    write(dest/'unified_source_provenance_v2.json',provenance)
    # One final full read both validates old bytes and supplies the new manifest.
    final,total=snapshot(dest,'Final current-file snapshot')
    changed=[relative for relative,expected in before.items() if relative!='README.md' and final.get(relative)!=expected]
    if changed:raise RuntimeError('Preexisting frozen files changed: '+repr(changed))
    legacy_after={'status':'FAIL' if legacy_before['status']=='FAIL' or before['README.md']!=sha(dest/'README.md') else 'PASS','before_v2_export':legacy_before,'root_readme_version_update_intentional':before['README.md']!=sha(dest/'README.md'),'original_manifest_retained_byte_exact':sha(dest/'manifest.json')==before['manifest.json'],'interpretation':'Historical coverage/status is retained as history, never promoted to current full-bundle integrity.'}
    audit={'version':'unified_export_v2','root':str(dest),'source_commit':SOURCE_SHA,'preexisting_files':len(before),'preexisting_bytes':before_bytes,'preexisting_file_changes_except_authorized_root_readme':changed,'root_readme_history_copy':'history/README_before_unified_visualization_v2.md','legacy_manifest':legacy_after,'c009_version_copy_complete':True,'m7_model_byte_equal_to_original_m2':True,'presentation_checks':read(ROOT/'visualization/results/presentation_checks.json'),'historical_physics_steps':0,'m7_budget':provenance['m7_physics_budget'],'scope':'New unified package snapshot with original scientific data unchanged; no claim that render success changes historical scientific acceptance.'}
    write(dest/'export_audit_v2.json',audit)
    final['export_audit_v2.json']=sha(dest/'export_audit_v2.json');total+=(dest/'export_audit_v2.json').stat().st_size
    manifest={'version':'softrobot_unified_export_manifest_v2','root_binding':str(dest),'hash_algorithm':'SHA-256','coverage':'all current files, including all historical manifests/reports and caches already present; only this manifest itself excluded to avoid self reference','excluded':['export_manifest_v2.json'],'file_count':len(final),'total_bytes':total,'files':final,'original_data_unchanged':not changed,'legacy_manifest_status_before':legacy_before['status'],'c009_source_commit':SOURCE_SHA,'m7_status':'PASS','m7_physics_steps':40000,'historical_physics_steps_this_round':0}
    write(dest/'export_manifest_v2.json',manifest)
    # The public development branch carries the export-bound inventory/audit;
    # it does not copy multi-GB historical datasets back into Git.
    for name in ['export_manifest_v2.json','export_audit_v2.json','unified_source_provenance_v2.json']:shutil.copyfile(dest/name,ROOT/name)
    print(json.dumps({'destination':str(dest),'files_in_new_manifest':len(final),'total_gb':total/1e9,'historical_manifest_before':legacy_before['status'],'frozen_original_changes':changed,'c009_source_files':len(files),'m7_status':'PASS','m7_physics_steps':40000,'historical_physics_steps':0},ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':main()
