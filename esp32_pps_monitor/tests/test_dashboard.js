// Real Chromium against local fixtures; never changes the physical monitor.
const assert=require('node:assert/strict'),fs=require('node:fs'),http=require('node:http');
const os=require('node:os'),path=require('node:path'),{spawn}=require('node:child_process');
const WebSocket=require('ws');
const page=fs.readFileSync(path.join(__dirname,'../main/dashboard.html'));
let fail=false,posts=0;
const channel=i=>({name:i?'UBX':'L76K',gpio:i?1:0,present:true,qualified:true,edges:120,glitches:0,missed:0,age_ms:5,period_us:1000000,width_us:100000});
let data={version:'1.0.0',device:'ESP32-C6 PPS alignment monitor',uptime_s:360,valid:true,resolution_hz:80000000,tick_ns:12.5,
 samples:60,total_pairs:60,unpaired:0,invalidations:0,queue_drops:0,offset_us:1.25,mean_us:1.23,median_us:1.25,min_us:1.2,max_us:1.3,
 stddev_us:.025,rms_us:1.23,peak_to_peak_us:.1,drift_ppm:.001,drift_samples:60,pair_age_ms:5,free_heap:140000,wifi_connected:true,wifi_disconnects:0,
 ip:'10.254.0.12',wifi_rssi:-65,channels:[channel(0),channel(1)],history:Array.from({length:60},(_,i)=>[59-i,1.25+Math.sin(i)*.025])};
const server=http.createServer((req,res)=>{
 if(req.url==='/api/health'){res.writeHead(fail?503:200,{'Content-Type':'application/json'});res.end(JSON.stringify(data));}
 else if(req.url==='/api/reset'){++posts;res.end('{"ok":true}');}
 else {res.writeHead(200,{'Content-Type':'text/html'});res.end(page);}
});
const folder=fs.mkdtempSync(path.join(os.tmpdir(),'pps-monitor-browser-'));let chrome,ws;
async function main(){
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const url='http://127.0.0.1:'+server.address().port;
 chrome=spawn('/opt/google/chrome/chrome',['--headless','--no-sandbox','--disable-gpu','--disable-background-networking','--no-first-run',
  '--remote-debugging-port=0','--user-data-dir='+folder+'/profile','about:blank'],{stdio:['ignore','ignore','pipe']});
 const endpoint=await new Promise((resolve,reject)=>{let log='';const timer=setTimeout(()=>reject(Error('Chrome startup timeout')),15000);
  chrome.stderr.on('data',b=>{log+=b;const m=log.match(/DevTools listening on (ws:\/\/\S+)/);if(m){clearTimeout(timer);resolve(m[1]);}});chrome.once('error',reject);});
 ws=new WebSocket(endpoint);await new Promise(r=>ws.once('open',r));let serial=0,session;const pending=new Map(),errors=[];
 ws.on('message',b=>{const m=JSON.parse(b);if(m.id){const p=pending.get(m.id);if(p){pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}}
  if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails.text);});
 const call=(method,params={},browser=false)=>new Promise((resolve,reject)=>{const id=++serial;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params,...(!browser&&session?{sessionId:session}:{})}));});
 const target=await call('Target.createTarget',{url:'about:blank'},true);session=(await call('Target.attachToTarget',{targetId:target.targetId,flatten:true},true)).sessionId;
 await call('Page.enable');await call('Runtime.enable');
 const evaluate=async expression=>{const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
 await call('Page.navigate',{url});
 for(let j=0;j<40;++j){if(await evaluate("document.getElementById('status')?.textContent==='Measuring'"))break;await new Promise(r=>setTimeout(r,100));}
 assert.equal(await evaluate("document.getElementById('offset').textContent"),'1.250');
 assert.equal(await evaluate("document.getElementById('pin0').textContent"),'GPIO0');
 assert.equal(await evaluate("document.getElementById('pin1').textContent"),'GPIO1');
 assert.equal(await evaluate("document.getElementById('direction').textContent"),'UBX arrives after L76K');
 for(const width of [1280,390,320]){
  await call('Emulation.setDeviceMetricsOverride',{width,height:1000,deviceScaleFactor:1,mobile:width<500});await evaluate('draw()');
  assert.equal(await evaluate('document.documentElement.scrollWidth<=window.innerWidth'),true,'horizontal overflow at '+width);
  const shot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true});fs.writeFileSync(folder+'/'+width+'.png',Buffer.from(shot.data,'base64'));
 }
 data.offset_us=-2.5;await evaluate('tick()');assert.equal(await evaluate("document.getElementById('offset').textContent"),'-2.500');
 assert.equal(await evaluate("document.getElementById('direction').textContent"),'UBX arrives before L76K');
 data.valid=false;data.offset_us=null;data.channels[1].present=false;data.channels[1].qualified=false;
 await evaluate('tick()');assert.equal(await evaluate("document.getElementById('offset').textContent"),'—');
 assert.equal(await evaluate("document.getElementById('status').textContent"),'Waiting for PPS');
 assert.equal(await evaluate("document.getElementById('signal1').textContent"),'No signal');
 assert.equal(await evaluate("document.getElementById('drift').textContent"),'—');
 assert.match(await evaluate("document.getElementById('historynote').textContent"),/Historical statistics/);
 data.channels[1].outage_ms=30000;data.channels[1].missed=30;await evaluate('tick()');
 assert.equal(await evaluate("document.getElementById('outage1').textContent"),'30.0 s');
 assert.equal(await evaluate("document.getElementById('missed1').textContent"),'30');
 assert.equal(await evaluate('startsNewLine([[2,1,1],[1,2,2]],1)'),true);
 assert.equal(await evaluate('startsNewLine([[5,1,1],[1,2,1]],1)'),true);
 assert.equal(await evaluate('startsNewLine([[2,1,1],[1,2,1]],1)'),false);
 data.channels[1].present=true;await evaluate('tick()');assert.equal(await evaluate("document.getElementById('status').textContent"),'Qualifying 1 Hz inputs');
 data.samples=0;data.history=[];data.drift_ppm=null;await evaluate('tick()');assert.equal(await evaluate("document.getElementById('empty').style.display"),'block');
 fail=true;await evaluate('tick()');assert.equal(await evaluate("document.getElementById('status').textContent"),'Monitor unavailable');
 assert.equal(await evaluate('document.body.className'),'stale');
 fail=false;data.valid=true;data.offset_us=0;await evaluate('tick()');assert.equal(await evaluate('document.body.className'),'');
 await evaluate("document.getElementById('reset').click()");await new Promise(r=>setTimeout(r,100));assert(posts>0);
 assert.deepEqual(errors,[]);console.log('Dashboard: desktop/mobile, offset signs, missing/qualifying inputs, empty history, offline/recovery and reset passed');
 console.log('Screenshots:',folder);
 await call('Browser.close',{},true);
}
main().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>{if(ws)ws.close();if(chrome)chrome.kill();server.close();});
