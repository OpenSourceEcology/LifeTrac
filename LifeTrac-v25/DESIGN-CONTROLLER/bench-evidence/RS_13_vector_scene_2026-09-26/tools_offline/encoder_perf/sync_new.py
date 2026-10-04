"""Refresh the abold/ (194fa5b8) and abnew/ (worktree) package copies used by ab.py."""
import os
import shutil

WT = ("C:/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-1/"
      "LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/x8_image_pipeline")
here = os.path.dirname(os.path.abspath(__file__))
for pkg, src in (("abold", os.path.join(here, "old")), ("abnew", WT)):
    d = os.path.join(here, pkg)
    os.makedirs(d, exist_ok=True)
    for f in ("encode_vector.py", "vector_extract.py", "vs1_codec.py"):
        shutil.copyfile(os.path.join(src, f), os.path.join(d, f))
    open(os.path.join(d, "__init__.py"), "w").close()
    shutil.rmtree(os.path.join(d, "__pycache__"), ignore_errors=True)
print("synced")
