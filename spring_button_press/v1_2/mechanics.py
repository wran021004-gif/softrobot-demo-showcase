"""Read-only reuse of v1.1 FamilySpace and guide generator; all writes local."""
import sys, importlib.util
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE.parent
spec=importlib.util.spec_from_file_location('v11_mechanics',BASE/'v1_1/mechanics.py')
v11=importlib.util.module_from_spec(spec);spec.loader.exec_module(v11)
read,write,LABEL=v11.read,v11.write,v11.LABEL
load_model,rgba,COLORS=v11.load_model,v11.rgba,v11.COLORS
S4=read(BASE/'v1_1/candidates/S4/mechanics.json')['candidate']
def assemble(candidate=None,folder=None):
    return v11.assemble(candidate or S4,folder=folder)
# Parent modules add their source directory. Restore local precedence explicitly.
sys.path.insert(0,str(HERE))
