"""2D error-state INS/GNSS fusion against a held-out OXTS trajectory.
Not a production navigation filter: no attitude quaternion, time synchronization calibration,
or independent ground truth. KITTI OXTS is a GPS/INS reference stream.
"""
import json, math, pathlib, numpy as np
ROOT=pathlib.Path(__file__).resolve().parent
SAMPLES=sorted(f for f in (ROOT/'data/oxts/data').glob('*.txt') if f.stem.isdigit())
T=[x.strip() for x in (ROOT/'data/oxts/timestamps.txt').read_text().splitlines()]
from datetime import datetime
TS=np.array([(datetime.fromisoformat(t[:26])-datetime.fromisoformat(T[0][:26])).total_seconds() for t in T]); D=np.array([[float(v) for v in f.read_text().split()] for f in SAMPLES]);N=len(D)
assert N==len(TS) and N>100
lat0,lon0=D[0,:2]; R=6378137.; scale=math.cos(math.radians(lat0))
truth=np.column_stack(((np.radians(D[:,1]-lon0))*R*scale,np.radians(D[:,0]-lat0)*R))
# KITTI OXTS fields af (forward acceleration, index 14); wz (yaw rate, index 19);
# yaw at index 5; vf (forward speed, index 8). KITTI development kit documents fields.
# At 10 Hz, Position and speed updates are artificially downsampled to 1 Hz. Outage: t=[5,11)s.
# State [E, N, speed, yaw, accelerometer bias, yaw-rate bias].
# Prediction propagates real measured vehicle-frame acceleration and angular velocity.
# OXTS position + forward speed update at 1 Hz outside outage.
x=np.array([truth[0,0],truth[0,1],D[0,8],D[0,5],0.,0.],float)
P=np.diag([4.,4.,1.,math.radians(3)**2,.08,.002])**2
Q=np.diag([0.001,0.001,0.28**2,math.radians(1.5)**2,.02**2,math.radians(.06)**2])
H=np.zeros((3,6));H[0,0]=H[1,1]=H[2,2]=1
V=np.diag([1.3**2,1.3**2,.25**2]); estimated=[];cvs=[];raw=[];GNSS_INTERVAL=10
for k in range(N):
 if k:
  dt=TS[k]-TS[k-1]; assert 0<dt<.2
  a=float(D[k,14]); omega=float(D[k,19]); yaw=x[3]; speed=x[2]
  # midpoint integration avoids one-sample lag in headings
  midpoint=yaw+0.5*(omega-x[5])*dt
  midpoint_speed=speed+0.5*(a-x[4])*dt
  x[0]+=midpoint_speed*math.cos(midpoint)*dt;x[1]+=midpoint_speed*math.sin(midpoint)*dt
  x[2]+=(a-x[4])*dt;x[3]+=(omega-x[5])*dt
  F=np.eye(6);F[0,2]=math.cos(midpoint)*dt;F[1,2]=math.sin(midpoint)*dt
  F[0,3]=-midpoint_speed*math.sin(midpoint)*dt;F[1,3]=midpoint_speed*math.cos(midpoint)*dt
  F[2,4]=-dt;F[3,5]=-dt
  P=F@P@F.T+Q*dt
 outage=5<=TS[k]<11
 if k%GNSS_INTERVAL==0 and not outage:
  z=np.array([*truth[k],D[k,8]])
  innovation=z-H@x
  S=H@P@H.T+V; K=P@H.T@np.linalg.inv(S)
  x+=K@innovation
  I=np.eye(6);P=(I-K@H)@P@(I-K@H).T+K@V@K.T
 estimated.append(x[:2].copy());cvs.append(float(2.4477*math.sqrt(max(P[0,0],P[1,1]))))
 raw.append([float(truth[k,0]),float(truth[k,1])] if k%GNSS_INTERVAL==0 and not outage else None)
estimated=np.array(estimated);error=np.linalg.norm(estimated-truth,axis=1)
mask=(TS>=5)&(TS<11)
res={
 'source': 'KITTI raw data, 2011_09_26_drive_0005_sync, OXTS',
 'source_url':'https://www.cvlibs.net/datasets/kitti/raw_data.php',
 'archive_url':'https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/2011_09_26_drive_0005_sync.zip',
 'method':'2D inertial propagation from real KITTI OXTS forward acceleration and yaw rate; six-state EKF with position, speed, heading and two sensor biases. OXTS position and speed subsampled to 1 Hz for correction; position and speed updates withheld from 5 to 11 seconds.',
 'limitations':'OXTS reference is GPS/INS-derived and not independent survey ground truth. The same OXTS stream supplies input and reference; results measure artificial 1 Hz position denial, not real receiver outages, independent accuracy, 3D navigation, 99% accuracy, or an ML model.',
 'dataset_points':N,'outage_start_s':5,'outage_end_s':11,
 'metrics':{'outage_rmse_m':round(float(np.sqrt(np.mean(error[mask]**2))),3),'outage_median_m':round(float(np.median(error[mask])),3),'outage_max_m':round(float(np.max(error[mask])),3),'outage_end_m':round(float(error[np.where(mask)[0][-1]]),3),'full_rmse_m':round(float(np.sqrt(np.mean(error**2))),3)},
 'points':[{'t':round(float(TS[i]),2),'truth':[round(float(y),3) for y in truth[i]],'fused':[round(float(y),3) for y in estimated[i]],'gnss':raw[i],'error':round(float(error[i]),3),'radius95':round(float(cvs[i]),3),'outage':bool(mask[i])} for i in range(N)]}
if __name__=='__main__':
 out=ROOT.parent/'web'/'benchmark.json';out.write_text(json.dumps(res,separators=(',',':')))
 print(json.dumps({'output':str(out),'samples':N,'metrics':res['metrics']},indent=2))
