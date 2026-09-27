"""Create CPU precise/sampled export presets for two recorded thread IDs."""
from pathlib import Path
import argparse,copy,xml.etree.ElementTree as E
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('control_tid',type=int);p.add_argument('sensor_tid',type=int);p.add_argument('output',type=Path)
a=p.parse_args()
WPT=Path('C:/Program Files (x86)/Windows Kits/10/Windows Performance Toolkit')
NS='http://tempuri.org/SerializableElement.xsd';E.register_namespace('',NS)
tag=lambda n:'{'+NS+'}'+n
tree=E.parse(WPT/'Catalog/Mobile.Unlock.wpaprofile');root=tree.getroot();content=root.find(tag('Content'))
precise=copy.deepcopy(next(g for g in root.iter(tag('Graph')) if g.get('Guid')=='c58f5fea-0319-4046-932d-e695ebe20b47'))
other=E.parse(WPT/'Catalog/AppLaunch.wpaProfile')
sampled=copy.deepcopy(next(g for g in other.iter(tag('Graph')) if g.get('Guid')=='b855361e-7be0-4bc8-a754-3e8507715ca5'))
for ref in content.findall('.//'+tag('FileReferences')):ref.clear()
views=content.find(tag('Views'));view=views.find(tag('View'))
for child in list(views):views.remove(child)
views.append(view)
indices=view.find(tag('SessionIndices'))
if indices is None:indices=E.SubElement(view,tag('SessionIndices'))
indices.clear();E.SubElement(indices,tag('SessionIndex')).text='0'
for child in list(content):
    if child.tag not in (tag('Sessions'),tag('Views')):content.remove(child)
graphs=view.find(tag('Graphs'));graphs.clear()
def configure(graph,label,names,tid_name):
    graph.set('LayoutStyle','All');graphs.append(graph)
    preset=graph.find(tag('Preset'));cols=preset.find(tag('Columns'))
    byname={col.get('Name'):col for col in cols}
    cols.clear()
    for name in names:
        col=copy.deepcopy(byname[name]);col.set('IsVisible','true');col.set('SortPriority','0')
        if name in ('TimeStamp','Switch-In Time'):col.attrib.pop('AggregationMode',None)
        cols.append(col)
    preset.set('InitialFilterShouldKeep','true');preset.set('Name',label);preset.set('KeyColumnCount','3')
    preset.set('GraphColumnCount',str(len(names)));preset.set('LeftFrozenColumnCount','0');preset.set('RightFrozenColumnCount',str(len(names)-1))
    preset.attrib.pop('InitialSelectionQuery',None)
    preset.set('InitialFilterQuery',f'([{tid_name}]:={a.control_tid} OR [{tid_name}]:={a.sensor_tid})')
    preset.set('InitialExpansionQuery','[Series Depth]:<=100')
configure(precise,'Lifetime switches',['New Process','New Thread Id','Switch-In Time','Last Switch-Out Time','Next Switch-Out Time','Ready Time','Ready','Waits','New Prev State','New Prev Wait Reason','CPU','New In Pri','Count','CPU Usage (in view)','New Thread Stack','Ready Thread Stack'],'New Thread Id')
configure(sampled,'Lifetime samples',['Process','Thread ID','TimeStamp','Stack','Module','Function','Address','Weight (in view)','CPU','DPC/ISR','Count'],'Thread ID')
tree.write(a.output,encoding='utf-8',xml_declaration=True)
print(a.output)

