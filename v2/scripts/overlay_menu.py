#!/usr/bin/env python3
"""Interactive selection of existing V2 trajectory sources."""
import sys
from pathlib import Path
from v2.scripts.isaac_trajectory import scan_final_p1
ROOT=Path(__file__).resolve().parents[2]
def options():
 result=[];seen=set()
 def add(label,kind,path):
  path=Path(path)
  if path.is_file():
   key=(kind,str(path.resolve()))
   if key not in seen:result.append((label,kind,str(path.resolve())));seen.add(key)
 for item in scan_final_p1():
  csv=Path(item['path']);name=item['name'];add(name+' P1/task','trajectory',csv)
  # For final P1, its segment or single task is the Parameterization ancestor.
  param=next((p for p in csv.parents if p.name in ('Parameterization','PRP')),None)
  if param:
   root=param.parent
   for label,kind,rel in [('P','P','ParentPath/parent_path.json'),('R','R','Reconstruction/geometry.json'),('P0','trajectory','Parameterization/P0/G/trajectory.csv'),('fast','trajectory','Parameterization/P1/fast/trajectory.csv')]:add(name+' '+label,kind,root/rel)
  for ancestor in csv.parents:
   if ancestor.name.startswith('V2-'):
    for f in sorted((ancestor/'Execution').glob('*/states.csv')):add(ancestor.name+' execution '+f.parent.name,'execution',f)
    break
 for f in sorted((ROOT/'v2/outputs').glob('SEQUENTIAL-*/final/trajectory.csv')):add(f.parents[1].name+' sequential','trajectory',f)
 return result

def main():
 entries=options()
 for i,(label,kind,path) in enumerate(entries,1):print('%d. %s | %s | %s'%(i,label,kind,path))
 print('输入编号如 1,3,5，可附 m:/绝对路径.csv；m 单独手动输入；q 退出')
 answer=input('选择 > ').strip()
 if answer.lower()=='q':return
 if answer.lower()=='m':
  path=Path(input('CSV 绝对路径 > ').strip());kind=input('类型 trajectory 或 execution > ').strip()
  if not path.is_absolute() or not path.is_file() or kind not in ('trajectory','execution'):raise ValueError('无效路径或类型')
  selected=[('manual',kind,str(path))]
 else:
  selected=[];seen=set()
  for token in answer.replace(',', ' ').split():
   if token.startswith('m:'):
    path=Path(token[2:]).expanduser()
    if not path.is_absolute() or not path.is_file():raise ValueError('手动 CSV 必须是现存绝对路径')
    item=('manual '+path.name,'trajectory',str(path.resolve()))
   else:
    i=int(token)
    if i<1 or i>len(entries):raise ValueError('编号超出范围')
    item=entries[i-1]
   if item[2] in seen:raise ValueError('请勿重复选择同一文件')
   seen.add(item[2]);selected.append(item)
  if len(selected)<2:raise ValueError('请选择至少两个数据源')
 from v2.scripts import trajectory_overlay
 sys.argv=['trajectory_overlay']+sum((['--source','%s:%s:%s'%('S%d'%(i+1),kind,path)] for i,(_,kind,path) in enumerate(selected)),[])
 for i,(name,kind,path) in enumerate(selected):print('图例 S%d: %s (%s)'%(i+1,name,path),flush=True)
 trajectory_overlay.main()
if __name__=='__main__':main()
