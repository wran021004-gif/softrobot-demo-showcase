"""Read-only SHA-256 verification of the unified export snapshot."""
import argparse,hashlib,json
from pathlib import Path
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while block:=f.read(4*1024*1024):h.update(block)
    return h.hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();m=json.loads((a.root/'export_manifest_v2.json').read_text(encoding='utf8'));missing=[];changed=[]
    for name,expected in m['files'].items():
        f=a.root/name
        if not f.is_file():missing.append(name)
        elif digest(f)!=expected:changed.append(name)
    actual={f.relative_to(a.root).as_posix() for f in a.root.rglob('*') if f.is_file()};extra=sorted(actual-set(m['files'])-{'export_manifest_v2.json'})
    report={'status':'PASS' if not missing and not changed and not extra else 'FAIL','missing':missing,'changed':changed,'extra_since_snapshot':extra,'checked_files':len(m['files']),'physics_steps':0};print(json.dumps(report,ensure_ascii=False,indent=2))
    if report['status']!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
