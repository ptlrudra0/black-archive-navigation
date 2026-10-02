/* Black Archive site motion. Behaviour inspired by Componentry's kinetic-text-reveal and sticky-scroll-cards
   (https://componentry.dev, MIT, (c) Harsh Jadhav). Re-implemented in vanilla JS/CSS; no source copied. */
(function(){
const reduce=matchMedia('(prefers-reduced-motion: reduce)').matches;
function split(el){let i=0;(function walk(n){[...n.childNodes].forEach(c=>{if(c.nodeType===3){const parts=c.textContent.split(/(\s+)/),f=document.createDocumentFragment();parts.forEach(p=>{if(!p)return;if(/^\s+$/.test(p)){f.appendChild(document.createTextNode(' '));return}const m=document.createElement('span');m.className='kr-mask';const w=document.createElement('span');w.className='kr-word';w.style.setProperty('--i',i++);w.textContent=p;m.appendChild(w);f.appendChild(m)});c.replaceWith(f)}else if(c.nodeType===1&&c.tagName!=='BR')walk(c)})})(el);el.classList.add('kr')}
const targets=[...document.querySelectorAll('.hero h1,.section-title,.finale h2')];
if(!reduce){targets.forEach(split);
 const io=new IntersectionObserver(es=>es.forEach(e=>{if(e.isIntersecting){e.target.classList.add('kr-in');io.unobserve(e.target)}}),{threshold:.35});
 targets.forEach(t=>t.matches('.hero h1')?requestAnimationFrame(()=>requestAnimationFrame(()=>t.classList.add('kr-in'))):io.observe(t));
}
/* method: the left column stays while steps pass; the step in view is the lit one */
const steps=[...document.querySelectorAll('.method-step')];
if(steps.length&&!reduce){document.body.classList.add('has-steps');
 const so=new IntersectionObserver(es=>es.forEach(e=>e.target.classList.toggle('on',e.isIntersecting)),{rootMargin:'-38% 0px -42% 0px'});
 steps.forEach(s=>so.observe(s));}
})();
