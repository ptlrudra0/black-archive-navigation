/* Browser-local GNSS / inertial research core. Coordinates: East, North. */
(function(root){'use strict';
const R=6378137,DEG=Math.PI/180;
const finite=v=>typeof v==='number'&&Number.isFinite(v);
function rotate(a,o){const A=o.alpha*DEG,B=o.beta*DEG,G=o.gamma*DEG,cA=Math.cos(A),sA=Math.sin(A),cB=Math.cos(B),sB=Math.sin(B),cG=Math.cos(G),sG=Math.sin(G);
return [ (cA*cG-sA*sB*sG)*a.x-sA*cB*a.y+(cA*sG+sA*sB*cG)*a.z,(sA*cG+cA*sB*sG)*a.x+cA*cB*a.y+(sA*sG-cA*sB*cG)*a.z ];}
class Axis{constructor(p,v,s){this.p=p;this.v=v;this.pp=s*s;this.pv=0;this.vv=9}predict(a,dt,q){this.p+=this.v*dt+.5*a*dt*dt;this.v+=a*dt;this.pp+=2*dt*this.pv+dt*dt*this.vv+.25*dt**4*q;this.pv+=dt*this.vv+.5*dt**3*q;this.vv+=dt*dt*q}update(z,r,velocity=false){const v=velocity?this.v:this.p,den=(velocity?this.vv:this.pp)+r,kp=(velocity?this.pv:this.pp)/den,kv=(velocity?this.vv:this.pv)/den,d=z-v,pp=this.pp,pv=this.pv,vv=this.vv;this.p+=kp*d;this.v+=kv*d;this.pp=pp-kp*(velocity?pv:pp);this.pv=pv-kp*(velocity?vv:pv);this.vv=vv-kv*(velocity?vv:pv)}}
class Core{
constructor(){this.reset()}
reset(){this.origin=null;this.axes=null;this.lastFix=0;this.lastPredict=0;this.lastMotion=0;this.orientation=null;this.acc=[0,0];this.bias=[0,0];this.biasSum=[0,0];this.biasWeight=0;this.learnedWindows=0;this.integral=[0,0];this.coverage=0;this.previousVelocity=null;this.previousFix=null;this.velocityAnchor=null;this.sensorFault=false;this.velocityKnown=false;this.mode='waiting';this.lastAccuracy=null;this.motionCount=0;this.gyro=0;this.hidden=false}
orient(o,time){if(![o.alpha,o.beta,o.gamma].every(finite))return false;let alpha=o.alpha,absolute=o.absolute===true;
// Safari compass reference is accepted only near level and with reported good compass accuracy.
if(!absolute&&finite(o.webkitCompassHeading)&&finite(o.webkitCompassAccuracy)&&o.webkitCompassAccuracy>=0&&o.webkitCompassAccuracy<=25&&Math.abs(o.beta)<25&&Math.abs(o.gamma)<25){alpha=(360-o.webkitCompassHeading)%360;absolute=true}
if(!absolute)return false;this.orientation={alpha,beta:o.beta,gamma:o.gamma,time};return true}
motion(a,r,time){this.motionCount++;if(!a||![a.x,a.y,a.z].every(finite)||!this.orientation||time-this.orientation.time>350){this.sensorFault=true;return false}const rate=r?Math.max(...['alpha','beta','gamma'].map(k=>finite(r[k])?Math.abs(r[k]):0)):0;this.gyro=rate;const z=rotate(a,this.orientation);if(rate>80||Math.hypot(a.x,a.y,a.z)>4){this.sensorFault=true;this.lastMotion=0;return false}this.acc=z;this.sensorFault=false;this.lastMotion=time;this.predict(time);return true}
predict(time){if(!this.axes||this.hidden)return;const dt=(time-this.lastPredict)/1000;if(dt<=0)return;this.lastPredict=time;if(dt>1.5){this.lastMotion=0;return}const age=(time-this.lastFix)/1000,ready=!this.sensorFault&&time-this.lastMotion<250&&this.orientation&&time-this.orientation.time<350;
if(age>8||!this.velocityKnown||this.lastAccuracy>100){this.mode='held';return}const a=ready?this.acc.map((v,i)=>Math.max(-3,Math.min(3,v-this.bias[i]))):[0,0];for(let i=0;i<2;i++)this.axes[i].predict(a[i],dt,ready?2.25:9);
if(ready){this.integral[0]+=this.acc[0]*dt;this.integral[1]+=this.acc[1]*dt;this.coverage+=dt}this.mode=age<3?'gnss':ready?'inertial':'coasting'}
fix(c,time){if(![c.latitude,c.longitude,c.accuracy].every(finite)||c.accuracy<=0||c.accuracy>10000||Math.abs(c.latitude)>90||Math.abs(c.longitude)>180)return false;if(this.lastFix&&time<=this.lastFix)return false;
if(!this.origin){this.origin=[c.latitude,c.longitude];this.axes=[new Axis(0,0,c.accuracy/2.448),new Axis(0,0,c.accuracy/2.448)];this.lastPredict=time}
this.predict(time);const p=[(c.longitude-this.origin[1])*DEG*R*Math.cos(this.origin[0]*DEG),(c.latitude-this.origin[0])*DEG*R],sigma=Math.max(3,c.accuracy/2.448);for(let i=0;i<2;i++)this.axes[i].update(p[i],sigma*sigma);
const stationary=finite(c.speed)&&c.speed===0;
const hasVelocity=finite(c.speed)&&c.speed>=0&&c.speed<70&&finite(c.heading)&&c.heading>=0&&c.heading<360;let v=null;
if(stationary||hasVelocity){v=stationary?[0,0]:[c.speed*Math.sin(c.heading*DEG),c.speed*Math.cos(c.heading*DEG)];for(let i=0;i<2;i++)this.axes[i].update(v[i],4,true);this.velocityKnown=true}
// Derive velocity only from clearly separated, recent, accurate observed fixes.
const pf=this.velocityAnchor,fixDt=pf?(time-pf.time)/1000:0;
if(!v&&pf&&fixDt>=1&&fixDt<=20&&c.accuracy<20&&pf.accuracy<20){const delta=p.map((x,i)=>x-pf.p[i]),dist=Math.hypot(...delta);if(dist>Math.max(8,c.accuracy,pf.accuracy)&&dist/fixDt<70){v=delta.map(x=>x/fixDt);for(let i=0;i<2;i++)this.axes[i].update(v[i],Math.max(4,(c.accuracy+pf.accuracy)**2/fixDt**2),true);this.velocityKnown=true}}
this.previousFix={p,time,accuracy:c.accuracy};
if(!this.velocityAnchor||v||fixDt>20)this.velocityAnchor=this.previousFix;
// With no dependable velocity, do not let near-zero covariance freeze fresh GPS positions.
if(!this.velocityKnown||c.accuracy>100){for(let i=0;i<2;i++){this.axes[i].p=p[i];this.axes[i].pp=sigma*sigma;this.axes[i].pv=0;this.axes[i].v=0}if(c.accuracy>100)this.velocityKnown=false}
// Fit a regularized constant sensor bias from prior interval GNSS velocity targets.
const prev=this.previousVelocity,elapsed=prev?(time-prev.time)/1000:0;
if(v&&prev&&c.accuracy<20&&prev.accuracy<20&&elapsed>=.7&&elapsed<=3&&this.coverage>=.8*elapsed&&this.gyro<20){const b=this.integral.map((x,i)=>(x-(v[i]-prev.v[i]))/elapsed);if(b.every(x=>Math.abs(x)<.8)){this.learnedWindows++;const w=Math.min(2,elapsed);this.biasWeight+=w;for(let i=0;i<2;i++){this.biasSum[i]+=b[i]*w;if(this.learnedWindows>=12)this.bias[i]=Math.max(-.35,Math.min(.35,this.biasSum[i]/(10+this.biasWeight)))}}}
this.previousVelocity=v?{v,time,accuracy:c.accuracy}:null;this.integral=[0,0];this.coverage=0;this.lastFix=time;this.lastAccuracy=c.accuracy;this.mode='gnss';return true}
position(time){if(!this.axes)return null;const age=Math.max(0,(time-this.lastFix)/1000);if(age>=3&&!this.velocityKnown||age>8||this.hidden)this.mode='held';const e=this.axes[0].p,n=this.axes[1].p;return {point:[this.origin[0]+n/R/DEG,this.origin[1]+e/(R*Math.cos(this.origin[0]*DEG))/DEG],speed:Math.hypot(...this.axes.map(a=>a.v)),mode:this.mode,age,radius:Math.max(this.lastAccuracy||10,2.448*Math.sqrt(Math.max(...this.axes.map(a=>a.pp))))+(age>=3?age*age:0),learnedWindows:this.learnedWindows,bias:this.bias.slice(),motionFresh:time-this.lastMotion<250,headingReady:!!this.orientation&&time-this.orientation.time<350}}
}
const api={Core,rotate,Axis};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.PhoneFusion=api;
})(typeof globalThis!=='undefined'?globalThis:this);
