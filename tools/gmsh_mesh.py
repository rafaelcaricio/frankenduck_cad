# /// script
# dependencies = ["gmsh==4.15.2", "numpy==2.5.4"]
# ///
"""Isolated Gmsh fallback: preserve CAD surfaces, emit mesh and face grouping."""
import gmsh,json,sys
from pathlib import Path
source=Path(sys.argv[1]); output=Path(sys.argv[2]); maximum=float(sys.argv[3])
gmsh.initialize()
gmsh.option.setNumber('General.NumThreads',2)
gmsh.model.occ.importShapes(str(source))
gmsh.model.occ.synchronize()
print('VOLUME',sum(gmsh.model.occ.getMass(3,t) for d,t in gmsh.model.getEntities(3)),flush=True)
for k,v in [('Mesh.MeshSizeMin',.005),('Mesh.MeshSizeMax',maximum),('Mesh.MeshSizeFromCurvature',64),('Mesh.Algorithm',6),('Mesh.MeshSizeExtendFromBoundary',0),('Mesh.Binary',1)]:gmsh.option.setNumber(k,v)
gmsh.model.mesh.generate(2)

tags,coords,_=gmsh.model.mesh.getNodes();types,elemTags,nodes=gmsh.model.mesh.getElements(2)
groups=[]
for d,t in gmsh.model.getEntities(2):
 ty,et,en=gmsh.model.mesh.getElements(2,t)
 groups.append({'tag':t,'center':list(gmsh.model.occ.getCenterOfMass(2,t)),'area':gmsh.model.occ.getMass(2,t),'triangles':[a.reshape(-1,3).tolist() for k,a in zip(ty,en) if k==2]})
output.write_text(json.dumps({'gmsh_version':gmsh.__version__,'maximum_edge_mm':maximum,'groups':groups,'tags':tags.tolist(),'vertices':coords.reshape(-1,3).tolist(),'triangles':[n.reshape(-1,3).tolist() for ty,n in zip(types,nodes) if ty==2]}))
gmsh.finalize()
