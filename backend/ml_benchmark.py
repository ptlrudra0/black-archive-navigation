"""Drive-held-out ridge correction for short inertial outages on KITTI OXTS.

The paired baseline is open-loop vehicle-frame acceleration/yaw-rate integration,
initialized from the reference position, speed and yaw at each blackout start. This
is a controlled experiment, not independent ground truth or a live phone model.
"""
from pathlib import Path
from datetime import datetime
import json, math
import numpy as np
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import Ridge

ROOT=Path(__file__).resolve().parent
DATA=ROOT/'data/ml'
START_STEP_S=7
OUTAGE_S=6
# Drive IDs fixed in advance, all from the same KITTI date.
DRIVE_IDS=('0005','0009','0011','0013','0014','0015','0017','0018','0019','0020','0022','0023','0027','0028','0029','0032')
SEED=42

def load_drive(folder):
    files=sorted(p for p in (folder/'data').glob('*.txt') if p.stem.isdigit())
    stamps=(folder/'timestamps.txt').read_text().splitlines()
    raw=np.array([[float(v) for v in p.read_text().split()] for p in files])
    t=np.array([(datetime.fromisoformat(z[:26])-datetime.fromisoformat(stamps[0][:26])).total_seconds() for z in stamps])
    if len(t)!=len(raw) or len(t)<75: raise ValueError(f'{folder.name}: incomplete samples')
    lat0,lon0=raw[0,:2]
    truth=np.column_stack((np.radians(raw[:,1]-lon0)*6378137*math.cos(math.radians(lat0)),np.radians(raw[:,0]-lat0)*6378137))
    return t,raw,truth

def window(t,raw,truth,start):
    i=np.searchsorted(t,start);j=np.searchsorted(t,start+OUTAGE_S)
    if j-i<48 or t[j-1]-t[i]<5.7:return []
    v=float(raw[i,8]);yaw=float(raw[i,5]);origin_yaw=yaw;pos=truth[i].copy()
    acc=[];rates=[];dtlist=[];rows=[]
    for k in range(i+1,j):
        dt=float(t[k]-t[k-1])
        if not .03<dt<.2:return []
        a=float(raw[k,14]);w=float(raw[k,19]);mid=yaw+.5*w*dt;vm=v+.5*a*dt
        pos+=dt*vm*np.array([math.cos(mid),math.sin(mid)])
        yaw+=w*dt;v+=a*dt
        acc.append(a);rates.append(w);dtlist.append(dt)
        # Only the start speed/yaw and inertial samples through this instant enter features.
        age=t[k]-t[i];aa=np.asarray(acc);ww=np.asarray(rates)
        features=[raw[i,8],age,aa.mean(),aa.std(),ww.mean(),ww.std(),
                  np.sum(aa*np.asarray(dtlist)),np.sum(ww*np.asarray(dtlist)),
                  np.sum(np.abs(ww)*np.asarray(dtlist)),aa[-1],ww[-1]]
        error=truth[k]-pos;c,s=math.cos(origin_yaw),math.sin(origin_yaw)
        local=np.array([c*error[0]+s*error[1],-s*error[0]+c*error[1]])
        rows.append((features,local,origin_yaw,age))
    return rows

def build():
    samples=[];metadata={}
    for key in DRIVE_IDS:
        folder=DATA/key
        if not folder.is_dir():raise FileNotFoundError(f'Missing OXTS drive {key} in {DATA}')
        t,raw,truth=load_drive(folder);count=0
        for start in np.arange(1,max(1,t[-1]-OUTAGE_S),START_STEP_S):
            rows=window(t,raw,truth,start)
            if rows:
                count+=1
                for features,target,yaw,age in rows:samples.append((folder.name,count,features,target,yaw,age))
        metadata[folder.name]={'samples':len(t),'windows':count,'duration_s':round(float(t[-1]),2)}
    return samples,metadata

def evaluate():
    np.random.seed(SEED)
    samples,metadata=build()
    X=np.asarray([r[2] for r in samples]);Y=np.asarray([r[3] for r in samples]);ids=np.asarray([r[0] for r in samples]);ages=np.asarray([r[5] for r in samples]);
    pred=np.zeros_like(Y);selected={}
    candidate_alpha=[100.0,1000.0,10000.0]
    for held in sorted(set(ids)):
        train=ids!=held;test=~train
        # Tune solely on training drives, using whole-drive inner validation.
        losses={}
        for alpha in candidate_alpha:
            errors=[]
            for inner in sorted(set(ids[train])):
                fit=train&(ids!=inner);valid=ids==inner
                m=make_pipeline(StandardScaler(),Ridge(alpha=alpha))
                m.fit(X[fit],Y[fit]);errors.extend(np.sum((Y[valid]-m.predict(X[valid]))**2,axis=1))
            losses[alpha]=np.mean(errors)
        alpha=min(candidate_alpha,key=lambda v:(losses[v],v));selected[held]=alpha
        model=make_pipeline(StandardScaler(),Ridge(alpha=alpha))
        model.fit(X[train],Y[train]);pred[test]=model.predict(X[test])
    baseline=np.linalg.norm(Y,axis=1);corrected=np.linalg.norm(Y-pred,axis=1)
    rms=lambda a:round(float(np.sqrt(np.mean(a*a))),3)
    per_drive={key:{'windows':metadata[key]['windows'],'samples':int(sum(ids==key)),
                    'baseline_rmse_m':rms(baseline[ids==key]),'ml_rmse_m':rms(corrected[ids==key])}
               for key in sorted(set(ids))}
    # Last sample of each non-overlapping six-second outage.
    last=np.array([not (z+1<len(samples) and samples[z+1][:2]==r[:2]) for z,r in enumerate(samples)])
    result={'method':'Drive-held-out ridge regression predicts the 2D local position error of open-loop inertial propagation, using only initial OXTS forward speed and observed acceleration/yaw-rate through the prediction instant.',
            'protocol':{'source':'KITTI raw OXTS, 2011_09_26 synchronized drives','drive_ids':list(metadata),'drive_count':len(metadata),'stride_s':START_STEP_S,'outage_s':OUTAGE_S,'ridge_alpha_candidates':[100.0,1000.0,10000.0],'ridge_alpha_selection':'nested leave-one-drive-out on training drives only','chosen_alpha_by_heldout_drive':selected,'train_test':'leave-one-entire-drive-out; no drive crosses a split','reference':'OXTS GPS/INS-derived position; not independent ground truth','features':['initial forward speed','elapsed time','prefix acceleration mean/std/integral/latest','prefix yaw-rate mean/std/integral/absolute integral/latest']},
            'total_windows':sum(v['windows'] for v in metadata.values()),'total_prediction_samples':len(samples),
            'baseline_outage_rmse_m':rms(baseline),'ml_outage_rmse_m':rms(corrected),
            'baseline_endpoint_rmse_m':rms(baseline[last]),'ml_endpoint_rmse_m':rms(corrected[last]),
            'per_drive':per_drive,'drives':metadata,
            'limitations':'Synthetic position denial on KITTI OXTS; start speed/yaw/position and reference come from GPS/INS-derived OXTS, not independent survey truth. It is an offline vehicle-mounted-sensor experiment and does not establish real GNSS-outage accuracy or handheld-phone performance. The paired baseline is open-loop inertial propagation, not the separately published 1.371 m Kalman-filter run; its split and initialization differ. ML predictions are evaluated out of drive, but training and evaluation share the KITTI domain.'}
    return result

if __name__=='__main__':
    result=evaluate();out=ROOT.parent/'web/ml_benchmark.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k in ('total_windows','total_prediction_samples','baseline_outage_rmse_m','ml_outage_rmse_m','baseline_endpoint_rmse_m','ml_endpoint_rmse_m','per_drive')},indent=2))
