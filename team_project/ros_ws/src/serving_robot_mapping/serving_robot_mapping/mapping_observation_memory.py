"""Restore independently logged successful sweeps after validated map resume."""
import json,math,hashlib
from pathlib import Path


def record_observation(memory, xy, proposal=None):
    """Charge both requested and physically measured viewpoints once per sweep.

    Do not merge anchors using the larger candidate exclusion radius: a
    proposal can lie beyond that radius while its actual arrival lies inside.
    """
    memory=list(memory)
    points=[tuple(xy)]
    if proposal and math.dist(xy,proposal['xy'])>=.02:
        points.append(tuple(proposal['xy']))
    for x,y in points:
        near=next((i for i,p in enumerate(memory) if math.hypot(p[0]-x,p[1]-y)<.02),None)
        if near is None:memory.append((x,y,1))
        else:memory[near]=(memory[near][0],memory[near][1],memory[near][2]+1)
    return memory


def restore_observation_memory(report_file,_seen=None):
    report_file=Path(report_file).resolve();seen=set() if _seen is None else set(_seen)
    if str(report_file) in seen or len(seen)>20:raise ValueError('Invalid observation resume chain')
    seen.add(str(report_file));memory=[]
    seed=report_file.parent/'mapping_observation_resume.json'
    if seed.exists():
        provenance=json.loads(seed.read_text());source=Path(provenance['source_report'])
        if hashlib.sha256(source.read_bytes()).hexdigest()!=provenance['source_sha256']:raise ValueError('Observation source report changed')
        memory=restore_observation_memory(source,seen)
    path=report_file.parent/'mapping_observations.jsonl'
    if not path.exists():return memory
    for line in path.read_text().splitlines():
        record=json.loads(line)
        if record.get('success') is not True:continue
        xy=record.get('xy');events=record.get('events')
        if (not isinstance(xy,list) or len(xy)!=2 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in xy)
                or not isinstance(events,list) or len(events)!=3 or any(e.get('status')!='SUCCEEDED' for e in events)):
            raise ValueError('Invalid successful observation evidence')
        memory=record_observation(memory,xy,record.get('proposal'))
    return memory
