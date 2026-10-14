"""Exercise the actual embedded dashboard JS for both firmware variants.

This is a DOM-binding test, not a claim of hardware/browser rendering coverage.
Run from repository root: python3 tests/test_ntp_dashboard.py
"""
import ast
import json
import pathlib
import re
import subprocess

source = pathlib.Path("main.c").read_text()
dashboard = source.split("static const char http_dashboard[] =", 1)[1].split(
    "static int rtc_append_health_json", 1
)[0]
for variant in (0, 1):
    preprocessed = subprocess.run(
        ["cc", "-E", "-P", "-x", "c", f"-DGPS_PROTOCOL_UBX={variant}", "-"],
        input=dashboard, text=True, capture_output=True, check=True,
    ).stdout
    html = "".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', preprocessed))
    ids = re.findall(r'\bid=(?:"([^"]+)"|([^\s>]+))', html)
    ids = [a or b for a, b in ids]
    assert len(ids) == len(set(ids)), "duplicate DOM IDs"
    script = html.split("<script>", 1)[1].split("</script>", 1)[0]
    bound_ids = set(re.findall(r"(?:v|\$)\('([^']+)'", script))
    assert bound_ids <= set(ids), f"missing DOM elements: {bound_ids-set(ids)}"
    assert "Rephases" not in html
    assert "compensated in software" in html
    assert "<details>" in html
    for qualified in (True, False):
        data = dict(synced=True, unix=1788890000, sntp_stratum=1, clock_uncertainty_valid=qualified,
                    rtc_sqw_time=True, rtc_aging_valid=True, rtc_aging=-8, rtc_aging_enabled=True,
                    rtc_phase_offset_ns=22000000, rtc_calibration_status="Learning",
                    rtc_calibration_observed_s=3600, rtc_calibration_required_s=21600,
                    rtc_calendar_write_count=2,
                    pps_placement_valid=True, pps_observation_window_us=20,
                    pps_timestamp_offset_us=-2450,
                    dispersion_source="GPS ", dispersion_base_us=5,
                    dispersion_phase_us=20, dispersion_frequency_us=1.5,
                    dispersion_margin_us=100, dispersion_slew_us=0,
                    clock_uncertainty_us=126.5, ntp_root_dispersion_us=137.33)
        program = "const ids=" + json.dumps(ids) + ";\n"
        program += "const data=" + json.dumps(data) + ";\n"
        program += r"""
const assert=require('node:assert/strict');
const elements=Object.fromEntries(ids.map(id=>[id,{textContent:'-',className:''}]));
const events={};
const document={hidden:false,body:{className:'stale'},getElementById:id=>elements[id],addEventListener:(n,f)=>events[n]=f};
let fetchCount=0,fetchMode='ok';
const fetch=async()=>{fetchCount++;if(fetchMode==='network')throw Error('offline');return {ok:fetchMode!=='http',status:503,json:async()=>{if(fetchMode==='json')throw Error('bad JSON');return new Proxy(data,{get:(o,k)=>k in o?o[k]:0})}}};
const scheduled=[];
const setTimeout=(fn,ms)=>{if(ms>=2000){scheduled.push(ms);return 0}return global.setTimeout(fn,ms)};
const clearTimeout=global.clearTimeout;
"""
        program += script
        program += r"""
setTimeout(async()=>{
  assert.equal(elements.status.textContent,data.clock_uncertainty_valid?'GPS disciplined / UTC synchronized':'Unqualified time / do not use as a precision reference');
  assert.equal(elements.disp_source.textContent,'GPS ');
  assert.equal(elements.disp_base.textContent,'5.0 us');
  assert.equal(elements.disp_phase.textContent,'20.0 us');
  assert.equal(elements.disp_growth.textContent,'1.5 us');
  assert.equal(elements.disp_margin.textContent,'100.0 us');
  assert.equal(elements.disp_slew.textContent,'0.0 us');
  assert.equal(elements.disp_total.textContent,data.clock_uncertainty_valid?'126.5 us':'Unqualified');
  assert.equal(elements.disp_wire.textContent,'137.3 us');
  assert.equal(elements.quality_total.textContent,elements.disp_total.textContent);
  assert.equal(elements.pps_window.textContent,'20.0 us');
  assert.equal(elements.pps_offset.textContent,'-2450.0 us');
  data.pps_timestamp_offset_us=2450;await tick();
  assert.equal(elements.pps_offset.textContent,'+2450.0 us');
  data.pps_placement_valid=false;await tick();
  assert.equal(elements.pps_window.textContent,'Unavailable');
  assert.equal(elements.pps_offset.textContent,'Unavailable');
  assert.equal(elements.phase.textContent,'22.000 ms');
  assert.equal(elements.rtc_aging.textContent,'-8 / automatic');
  assert.equal(elements.rtc_learning.textContent,'1.0 / 6.0 h');
  assert.equal(elements.rtc_error.textContent,'Not in holdover');
  assert.equal(elements.rtc_writes.textContent,2);
  assert.equal(document.body.className,'');
  assert.equal(scheduled.at(-1),2000);
  data.sntp_stratum=2;data.clock_uncertainty_valid=true;data.rtc_holdover_active=true;
  data.rtc_estimated_error_ns=4000000;data.rtc_holdover_age_s=5;
  await tick();
  assert.equal(elements.status.textContent,'RTC holdover / GPS precision unavailable');
  assert.equal(elements.rtc_error.textContent,'4.00 ms');
  data.sntp_stratum=16;data.clock_uncertainty_valid=false;data.rtc_sqw_time=false;data.pps=true;
  await tick();
  assert.equal(elements.pps.textContent,'Present');
  assert.equal(elements.phase.textContent,'Unqualified');
  assert.equal(elements.quality_total.textContent,'Unqualified');
  data.rtc_32k_enabled=true;data.rtc_32k_valid=false;data.rtc_32k_age_ms=-1;
  await tick();assert.equal(elements.rtc_32k.textContent,'No signal');
  data.rtc_32k_age_ms=100;data.rtc_32k_sample_count=2;
  await tick();assert.equal(elements.rtc_32k.textContent,'Learning / 2 accepted');
  data.rtc_32k_enabled=false;await tick();assert.equal(elements.rtc_32k.textContent,'Disabled');
  for(const mode of ['network','http','json']){
    fetchMode=mode;await tick();assert.equal(document.body.className,'stale');
    assert.equal(elements.status.textContent,'Device unavailable / displayed values are stale');
  }
  assert.equal(scheduled.at(-1),16000);
  for(let i=0;i<6;i++)await tick();
  assert.equal(scheduled.at(-1),30000);
  document.hidden=true;events.visibilitychange();const count=fetchCount;
  await tick();assert.equal(fetchCount,count);assert.equal(elements.freshness.textContent,'Paused / background tab');
  document.hidden=false;fetchMode='ok';await tick();
  assert.equal(document.body.className,'');assert.equal(scheduled.at(-1),2000);
},0);
"""
        subprocess.run(["node", "-e", program], check=True)
    print(f"Dashboard bindings passed: GPS_PROTOCOL_UBX={variant}, HTML bytes={len(html.encode())}")
    health = source.split("static int rtc_append_health_json", 1)[1].split("typedef struct {", 1)[0]
    health = subprocess.run(
        ["cc", "-E", "-P", "-x", "c", f"-DGPS_PROTOCOL_UBX={variant}", "-"],
        input=health, text=True, capture_output=True, check=True,
    ).stdout
    formats = re.findall(
        r'snprintf\(out(?:\s*\+\s*offset)?,\s*out_len(?:\s*-\s*offset)?,\s*((?:"(?:[^"\\]|\\.)*"\s*)+),',
        health,
    )
    assert len(formats) == 3, "health format extraction changed; inspect capacity"
    bound = 1  # NUL
    for literals in formats:
        fmt = "".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', literals))
        # Conservative shipped-configuration widths: strings <=39 bytes;
        # booleans <=5; floats <=17 integer digits + sign/decimal/fraction;
        # integers use their complete target-type range, even for satellites.
        def width(match):
            token = match.group()
            if token.endswith("s"):
                if "." in token:
                    return "x" * int(token.split(".")[1][:-1])
                return "x" * (39 if fmt[match.start()-1] == '"' else 5)
            if token.endswith("f"):
                precision = int(token.split(".")[1][:-1]) if "." in token else 6
                return "x" * (19 + precision)
            return "x" * (20 if "ll" in token else 11)
        bound += len(re.sub(r'%[-+0-9.]*[l]*[sduf]', width, fmt))
    capacity = int(re.search(r'#define HTTP_HEALTH_CAPACITY (\d+)', source).group(1))
    assert bound <= capacity, (bound, capacity)
    print(f"Health format bound: {bound}/{capacity} bytes (shipped value ranges)")
