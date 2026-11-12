// Deterministic scheduling regression against the actual dashboard tick().
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const page=fs.readFileSync(path.join(__dirname,'../main/dashboard.html'),'utf8');
const tick=page.slice(page.indexOf('async function tick(){'),page.indexOf("$('reset').addEventListener"));
let serial=0,requests=0,fail=false;
const pending=new Map();
const context={busy:false,document:{hidden:false,body:{}},lastSuccess:0,timer:0,AbortController,Date,
 setTimeout:(f,t)=>{pending.set(++serial,{f,t});return serial},clearTimeout:id=>pending.delete(id),
 fetch:async()=>{++requests;return {ok:!fail,json:async()=>({channels:[{},{}],history:[]})}},
 render:()=>{},put:()=>{},$:()=>({})};
vm.createContext(context);vm.runInContext(tick,context);
(async()=>{
 await context.tick();assert.equal(pending.size,1);
 for(let i=0;i<10;++i){await context.tick();assert.equal(pending.size,1,'manual refresh duplicated polling');}
 context.document.hidden=true;await context.tick();assert.equal(pending.size,0);
 context.document.hidden=false;await context.tick();assert.equal(pending.size,1);
 context.busy=true;await context.tick();assert.equal(pending.size,0);context.busy=false;
 fail=true;await context.tick();assert.equal(pending.size,1,'failure must retain exactly one retry');
 fail=false;await context.tick();assert.equal(pending.size,1);
 assert.equal(requests,14);
 console.log('Polling: repeated manual refresh, hidden tab, busy fetch, failure and recovery keep one timer');
})().catch(e=>{console.error(e);process.exitCode=1});
