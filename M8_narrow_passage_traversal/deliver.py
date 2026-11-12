"""Bounded incremental export. Never enumerates or rehashes historical data."""
import argparse,subprocess,shutil,json,hashlib
from pathlib import Path
H=Path(__file__).resolve().parent;ROOT=H.parent
DEST=Path('D:/softrobot-benchmark-exports/base_only_accepted_20261028')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.parent.mkdir(exist_ok=True,parents=True);p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf8').splitlines()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--copy',action='store_true');a=parser.parse_args()
    changed=git('diff','--name-only','2e5d1088746df0916e2a7ec60f4780998c6ba004');new=git('ls-files','--others','--exclude-standard')
    # Explicit allowlist: requested task, shared presentation layer, root index,
    # and a new byte-exact historical README snapshot. No old task writes.
    names=sorted(set(changed+new)-{'export_manifest_v2.json'})
    def permitted(n):return n=='README.md' or n.startswith('M8_narrow_passage_traversal/') or n.startswith('visualization/') or n=='history/README_before_M8_20261112.md'
    assert all(permitted(n) for n in names),[n for n in names if not permitted(n)]
    # Existing Figure-eight has no local export folder; copy only its own small
    # original folder, byte for byte. It is already tracked in Git.
    eight=[p.relative_to(ROOT).as_posix() for p in (ROOT/'figure_eight_tracking').rglob('*') if p.is_file()]
    names=sorted(set(names+eight+['M8_narrow_passage_traversal/results/delivery_audit.json']))
    previous=json.loads((DEST/'export_manifest_v2.json').read_text(encoding='utf8'));updates={};size_delta=0;added=[];modified=[]
    for n in names:
        p=ROOT/n
        if not p.exists():continue
        target=(DEST/n).resolve();assert target.is_relative_to(DEST.resolve())
        if n.startswith('figure_eight_tracking/') and target.exists():assert sha(target)==sha(p),'Refuse to overwrite historical Figure-eight data'
        if n.startswith('M8_narrow_passage_traversal/') and target.exists() and sha(target)!=sha(p):raise RuntimeError('Refuse to overwrite a different existing M8 export')
        updates[n]=sha(p);size_delta+=p.stat().st_size-(target.stat().st_size if target.exists() else 0)
        (modified if n in previous['files'] else added).append(n)
    audit={'status':'PREPARED','destination':str(DEST),'only_requested_task_and_presentation_files':True,'historical_models_trajectories_results_recomputed':False,'historical_tree_rehashed':False,'previous_manifest_sha256':sha(DEST/'export_manifest_v2.json'),'added_paths':added,'modified_paths':modified,'physics_steps':48000,'independent_processes':1,'task_status':'FAIL','public_repository_verified':True,'git_base':'2e5d1088746df0916e2a7ec60f4780998c6ba004','branch':'codex/narrow-passage-traversal-20261112','new_manifest_rule':'Retain every untouched historical SHA; hash only explicitly listed new/changed files'}
    write(H/'results/delivery_audit.json',audit)
    rel='M8_narrow_passage_traversal/results/delivery_audit.json';p=ROOT/rel
    if rel not in added:added.append(rel)
    # The audit file has just been finalized; account for it exactly once.
    updates={n:sha(ROOT/n) for n in names};size_delta=sum((ROOT/n).stat().st_size-((DEST/n).stat().st_size if (DEST/n).exists() else 0) for n in names)
    manifest={**previous,'files':{**previous['files'],**updates}};manifest['file_count']=len(manifest['files']);manifest['total_bytes']=previous['total_bytes']+size_delta
    write(ROOT/'export_manifest_v2.json',manifest)
    if a.copy:
        # Preserve the old manifest as a separately named new history record.
        old=DEST/'history/export_manifest_before_M8_20261112.json'
        if not old.exists():
            old.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(DEST/'export_manifest_v2.json',old)
        # Its bytes must be included, without recursively hashing the tree.
        manifest['files']['history/export_manifest_before_M8_20261112.json']=sha(old)
        manifest['file_count']=len(manifest['files']);manifest['total_bytes']+=old.stat().st_size
        shutil.copyfile(old,ROOT/'history/export_manifest_before_M8_20261112.json')
        for n in names:
            p=ROOT/n;target=DEST/n;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
        write(ROOT/'export_manifest_v2.json',manifest);shutil.copyfile(ROOT/'export_manifest_v2.json',DEST/'export_manifest_v2.json')
        mismatch=[n for n,v in updates.items() if sha(DEST/n)!=v];assert not mismatch,mismatch
        print(json.dumps({'status':'COPIED_AND_CHECKED','copied_files':len(names),'incremental_sha_checks':len(updates),'task_status':'FAIL','destination':str(DEST)}))
    else:print(json.dumps({'status':'PREPARED','files':len(names),'manifest_file_count':manifest['file_count']}))
if __name__=='__main__':main()
