from pathlib import Path
import json,sys
import numpy as np
from scipy.spatial.transform import Rotation
import gzip,argparse
task=Path(__file__).resolve().parents[1]
from task_impl.grading import Grade
from task_impl.planar_clearance import PlanarClearance
def score(r):
 f=json.loads((task/'task_impl/fixture.json').read_bytes())
 actors={a['name']:a for a in f['actors']};gauges={s['name']:PlanarClearance(actors[s['a']],actors[s['b']]) for s in f['metrics'] if s['kind']=='planar_clearance'}
 grade=Grade(f);all_steps=[]
 for row in r:
  frames={'table':(np.zeros(3),np.eye(3),0)}
  for n,a in actors.items():
   z=row['poses'].get(n,a['position']+[0,0,0,1]);frames[n]=(np.asarray(z[:3]),Rotation.from_quat(z[3:]).as_matrix(),0)
  val={}
  for m in f['metrics']:
   n,k=m['name'],m['kind']
   if k=='food_guard':
    if n.startswith('impact_') and n not in row['guard']:
     # Older captures already recorded the *maximum* press timer at every
     # control step. All-zero maxima prove every sampled downward load was
     # below self-weight, hence every excess-load impulse was exactly zero.
     if any(x['guard']['press_'+n[7:]]>0 for x in r):
      raise ValueError('Missing impact evidence for a potentially loaded slice')
     v=0.
    else:v=row['guard'][n]
   elif k=='contact_n':
    key={'on_blade':'crust_blade','on_dish':'crust_plate','l_crust':'left_crust','l_filling':'left_filling','r_crust':'right_crust','r_filling':'right_filling'}[n];v=row['final_contact'][key]
   elif k=='planar_clearance':v=gauges[n].distance(frames)
   else:
    a,ra,_=frames[m['a']];b,rb,_=frames[m['b']];pa=ra@m['pa'];pb=rb@m['pb'];delta=a+pa-b-pb
    if k=='angle_deg':v=np.degrees(np.arccos(np.clip(pa@pb/np.linalg.norm(pa)/np.linalg.norm(pb),-1,1)))
    elif k=='distance':v=np.linalg.norm(delta)
    elif k=='planar':v=np.linalg.norm(delta[:2])
    elif k=='dy':v=delta[1]
    elif k=='dz':v=delta[2]
    else:raise ValueError(k)
   val[n]=float(v)
  all_steps.append({'step':row['step'],**grade.update(val)})
 summary={'steps':len(r),'firstStep':r[0]['step'],'final':all_steps[-1],'firstAt':{}}
 for t in (.25,.5,.75,1):
  summary['firstAt'][str(t)]=next((x['step'] for x in all_steps if x['score']>=t),None)
 summary['foodTouchMaxN']=max(v for x in r for k,v in x['guard'].items() if k.startswith(('left_','right_')))
 for prefix in ('lane','tilt','shift','press'):
  summary[prefix+'_max']=max(v for x in r for k,v in x['guard'].items() if k.startswith(prefix+'_'))
 return summary,all_steps

def main():
    ap=argparse.ArgumentParser(description="Replay observed physics rows through this task's grader.")
    ap.add_argument("--recording",choices=("reference","finger","lane","strike"),default="reference")
    ap.add_argument("--raw",type=Path,help="A fresh run directory made by evidence.local_reference")
    ap.add_argument("--write",type=Path,help="Write this control-step grade trace as JSON Lines")
    args=ap.parse_args()
    if args.raw:
        rows=json.loads((args.raw/"rollout.json").read_bytes())
    else:
        with gzip.open(Path(__file__).with_name(args.recording+".json.gz"),"rt",encoding="utf8") as stream:
            artifact=json.load(stream)
        rows=artifact["rows"]
    summary,steps=score(rows)
    if args.write:
        with args.write.open("w",encoding="utf8") as stream:
            for row in steps:stream.write(json.dumps(row)+"\n")
    peak_impact=max(float(x["guard"].get("impact_slice"+str(i),0.)) for x in rows for i in range(1,8))
    print(json.dumps({"peakImpactRatio":peak_impact,"steps":summary["steps"],"score":summary["final"]["score"],"invalid":summary["final"]["invalid"],
                      "milestones":summary["firstAt"],"foodTouchMaxN":summary["foodTouchMaxN"],
                      "maxShiftM":summary["shift_max"],"maxTiltDeg":summary["tilt_max"],
                      "maxLaneOpeningM":summary["lane_max"],"longestPressS":summary["press_max"]},indent=2))
if __name__=="__main__":main()
