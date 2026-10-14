// Render the actual firmware page against local fixtures, never a live device.
// Requires Chrome and the host's node-ws package; screenshots stay in /tmp.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const http = require('node:http');
const {spawn, execFileSync} = require('node:child_process');
const WebSocket = require('ws');
const pages = [0, 1].map(v => execFileSync('python3', ['tools/render_dashboard.py', '--variant', String(v)], {encoding:'utf8'}));
const fields = [...new Set(pages[1].matchAll(/\bd\.([a-z_0-9]+)/g))].map(m => m[1]);
let data = Object.fromEntries(fields.map(k => [k, 0]));
Object.assign(data, {
    unix:1788890000, synced:true, src:'GPS', clock_state:'LOCKED', sntp_stratum:1, sntp_quality:'PPS',
    clock_uncertainty_valid:true, clock_uncertainty_us:2567.57, ntp_root_dispersion_us:2578.736,
    dispersion_source:'GPS ', dispersion_base_us:2460.02, dispersion_phase_us:.05, dispersion_frequency_us:7.5,
    dispersion_margin_us:100, fix:true, sats:17, pps:true, hdop:1.2, pdop:1.35,
    pps_placement_valid:true, pps_observation_window_us:20, pps_timestamp_offset_us:-2450,
    lat_ok:true, lon_ok:true, lat:30.0809, lon:-93.7938, timezone_mode:'AUTO',
    timezone_name:'America/Chicago', local_time:'2026-09-08T12:53:20', timezone_source:'GPS_POSITION',
    timezone_offset_text:'-05:00', timezone_is_dst:true, timezone_candidate:'America/Chicago',
    timezone_data_version:'2026c', timezone_boundary_version:'2026c', timezone_data_start_year:2020, timezone_data_end_year:2050,
    rtc_state:'LOCKED', rtc_status:true, rtc_sqw:true, rtc_sqw_time:true, rtc_phase_offset_ns:22200000,
    rtc_32k_enabled:true, rtc_32k_valid:true, rtc_32k_frequency_error_ppm:-.269, rtc_aging_valid:true,
    rtc_aging:-8, rtc_aging_enabled:true, rtc_calibration_status:'Learning', rtc_calibration_observed_s:1200,
    rtc_calibration_required_s:21600, rtc_temperature_valid:true, rtc_temperature_c:23.25, rtc_osf_boot_read:true,
    wifi_link:true, wifi_rssi:-73, ip:'10.254.0.11', uptime_s:1300, version:'0.4.0', build:'Sep 8 2026 12:43:18',
    discipline_mode:'TRACKING', ab_servo_valid:true, ab_period_ns:1000000650, ab_median_ns:1000000660,
    leap_loaded:true, leap_source:'eeprom', leap_expire_utc:'2027-06-28T00:00:00Z', leap_next_utc:'N/A',
    ubx_sat_total:40, ubx_sat_used:17, ubx_sat_gps:12, ubx_used_gps:8, ubx_sat_galileo:11, ubx_used_galileo:5,
    ubx_sat_beidou:14, ubx_used_beidou:4,
});
let fail = false, calls = 0, externalRequests = [];
const server = http.createServer((req,res) => {
    if(req.url === '/api/health') {
        calls++; res.writeHead(fail ? 503 : 200, {'Content-Type':'application/json'});
        res.end(JSON.stringify(data));
    } else {res.writeHead(200, {'Content-Type':'text/html'}); res.end(pages[req.url.includes('l76k') ? 0 : 1]);}
});
const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'pico-dashboard-browser-'));
let chrome, ws;
async function main() {
    await new Promise(r => server.listen(0, '127.0.0.1', r));
    const url = `http://127.0.0.1:${server.address().port}`;
    chrome = spawn(process.env.CHROME_BIN || '/opt/google/chrome/chrome', ['--headless', '--no-sandbox', '--disable-gpu',
        '--no-first-run', '--disable-background-networking', '--remote-debugging-port=0', `--user-data-dir=${temp}/profile`, 'about:blank'],
        {stdio:['ignore','ignore','pipe']});
    const endpoint = await new Promise((resolve,reject) => {
        let log=''; const timer=setTimeout(()=>reject(Error('Chrome startup timeout')),15000);
        chrome.once('error',reject);
        chrome.stderr.on('data',b=>{log+=b;const m=log.match(/DevTools listening on (ws:\/\/[^\s]+)/);if(m){clearTimeout(timer);resolve(m[1])}});
        chrome.once('exit',code=>reject(Error(`Chrome exited ${code}: ${log}`)));
    });
    ws = new WebSocket(endpoint); await new Promise(r=>ws.once('open',r));
    let serial=0, sessionId; const pending=new Map();
    ws.on('message',b=>{const m=JSON.parse(b);if(m.id){const p=pending.get(m.id);if(p){pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result)}}
        if(m.method==='Network.requestWillBeSent' && !m.params.request.url.startsWith(url) && !m.params.request.url.startsWith('about:')) externalRequests.push(m.params.request.url);
    });
    const call = (method,params={},browser=false)=>new Promise((resolve,reject)=>{
        const id=++serial;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params,...(!browser&&sessionId?{sessionId}:{})}));
    });
    const target = await call('Target.createTarget',{url:'about:blank'},true);
    sessionId=(await call('Target.attachToTarget',{targetId:target.targetId,flatten:true},true)).sessionId;
    await call('Page.enable');await call('Runtime.enable');await call('Network.enable');
    const evaluate = async expression => {
        const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});
        if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails)); return r.result.value;
    };
    const wait = async expression => evaluate(`new Promise((resolve,reject)=>{let n=0;const t=setInterval(()=>{if(${expression}){clearInterval(t);resolve(true)}else if(++n>100){clearInterval(t);reject(Error('UI timeout'))}},50)})`);
    for(const variant of ['l76k','ubx']) {
        await call('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
        await call('Page.navigate',{url:`${url}/${variant}`});
        await wait("document.getElementById('freshness')?.textContent==='Live snapshot'");
        assert.equal(await evaluate("document.querySelectorAll('details[open]').length"),0);
        for(const [name,width,height,expanded] of [['desktop',1280,1000,false],['mobile',390,844,false],['mobile-details',320,900,true],['desktop-details',1280,1000,true]]) {
            await call('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:width<600});
            if(expanded)await evaluate("document.querySelectorAll('details').forEach(d=>d.open=true)");
            await evaluate("new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))");
            assert.equal(await evaluate('document.documentElement.scrollWidth<=innerWidth'),true,`${variant} ${name} horizontal overflow`);
            assert.equal(await evaluate("[...document.querySelectorAll('dd')].filter(e=>e.getClientRects().length&&e.scrollWidth>e.clientWidth+1).length"),0,`${variant} ${name} clipped values`);
            if(expanded)await evaluate("document.querySelector('details').scrollIntoView()");
            const shot = await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
            fs.writeFileSync(`${temp}/${variant}-${name}.png`,Buffer.from(shot.data,'base64'));
        }
        Object.assign(data,{sntp_stratum:2,src:'RTC',clock_uncertainty_valid:true,rtc_holdover_active:true,rtc_estimated_error_ns:4000000,rtc_holdover_age_s:20});
        await evaluate('tick()'); assert.match(await evaluate("document.getElementById('status').textContent"),/RTC holdover/);
        Object.assign(data,{sntp_stratum:16,clock_uncertainty_valid:false,rtc_sqw_time:false,rtc_32k_valid:false,rtc_32k_sample_count:0});
        await evaluate('tick()'); assert.match(await evaluate("document.getElementById('status').textContent"),/Unqualified time/);
        fail=true;await evaluate('tick()');assert.equal(await evaluate('document.body.className'),'stale');
        fail=false;Object.assign(data,{sntp_stratum:1,src:'GPS',clock_uncertainty_valid:true,rtc_sqw_time:true,rtc_32k_valid:true,rtc_holdover_active:false});
        await evaluate('tick()');assert.equal(await evaluate('document.body.className'),'');
        console.log(`${variant}: desktop/mobile render, no overflow, holdover/unqualified/offline/recovery passed`);
    }
    assert.deepEqual(externalRequests,[]);
    console.log(`Screenshots: ${temp}; local health requests: ${calls}; external requests: 0`);
}
main().catch(e=>{console.error(e);process.exitCode=1}).finally(()=>{if(ws)ws.close();if(chrome)chrome.kill();server.close()});
