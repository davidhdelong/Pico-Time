"""Run the actual embedded dashboard script against healthy/stale fixtures."""
import ast
import json
import pathlib
import re
import subprocess

s = pathlib.Path("esp32_p4_ntp/main/main.c").read_text()
literal = s.split("static const char *dashboard =", 1)[1].split("\n\nstatic uint64_t now_ns", 1)[0]
page = "".join(ast.literal_eval(x) for x in re.findall(r'"(?:[^"\\]|\\.)*"', literal))
script = page.split("<script>", 1)[1].split("</script>", 1)[0]
ids = re.findall(r"id=([a-z]+)", page)
harness = r'''
const vm=require('node:vm'),assert=require('node:assert/strict');
const elements=Object.fromEntries(IDS.map(id=>[id,{textContent:''}]));
let fail=false,timers=new Map(),counter=0;
const data={clock_state:'LOCKED',gnss:'L76K',version:'0.1.0',utc:'2026-11-18T12:00:00',local_time:'2026-11-18T06:00:00',
 clock_uncertainty_us:105.3,fix:true,sats:12,pps:true,timezone_name:'America/Chicago',timezone_data_version:'2026e',
 timezone_boundary_version:'2026f',ethernet_link:true,ip:'10.0.0.2',ntp_requests:10,pps_capture_faults:0,queue_drops:0};
const context={document:{hidden:false,getElementById:id=>{assert(elements[id],id);return elements[id]}},AbortController,
 fetch:async()=>({ok:!fail,status:fail?503:200,json:async()=>data}),
 setTimeout:(f,t)=>{timers.set(++counter,{f,t});return counter},clearTimeout:id=>timers.delete(id)};
vm.createContext(context);vm.runInContext(SCRIPT,context);
(async()=>{
 await new Promise(r=>setImmediate(r));
 assert.equal(elements.utc.textContent,data.utc);assert.equal(elements.local.textContent,data.local_time);
 assert.match(elements.tz.textContent,/2026f/);assert.equal(timers.size,1);
 timers.clear();fail=true;await context.poll();assert.match(elements.status.textContent,/historical/);assert.equal(timers.size,1);
 timers.clear();fail=false;data.utc=null;data.local_time=null;data.clock_state='UNSYNCED';data.clock_uncertainty_us=null;
 data.ethernet_link=false;data.fix=false;data.pps=false;await context.poll();
 assert.equal(elements.utc.textContent,'Unqualified');assert.equal(elements.error.textContent,'Unqualified');
 assert.equal(elements.eth.textContent,'Link down');assert.equal(timers.size,1);
 console.log('P4 dashboard: actual embedded JS, healthy/unsynced/offline states, timezone and one polling timer passed');
})().catch(e=>{console.error(e);process.exitCode=1});
'''.replace("IDS", json.dumps(ids)).replace("SCRIPT", json.dumps(script))
subprocess.run(["node", "-e", harness], check=True)
