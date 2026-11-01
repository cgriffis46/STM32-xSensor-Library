// Node test of gw121-protocol.js: it decodes what our C++ firmware sends
// (packets.txt from staging/boards/gw121/sim/test: `make packets`), its own
// encoder round-trips, and its key and clock messages match the format.
//   node test/protocol.test.js ../../boards/gw121/sim/test/packets.txt
'use strict';
const fs = require('fs');
const G = require('../gw121-protocol.js');
let failures = 0;
const check = (c, what) => { if (!c) { failures++; console.log('  FAIL ' + what); } };

console.log('decodes the C++ simulator\'s packets');
const file = process.argv[2];
const lines = fs.readFileSync(file, 'utf8').trim().split('\n');
const packets = lines.map((l) => Uint8Array.from(l.match(/../g).map((h) => parseInt(h, 16))));
check(packets.length > 10, 'packets read');
const seen = new Set();
for (const p of packets) {
  const f = G.decode(p);
  check(f.ok, 'decoded: ' + f.error);
  if (!f.ok) continue;
  check(f.serial === 12345 && f.calYear === 2026 && f.calMonth === 11, 'serial and date');
  seen.add(f.mode);
}
check(seen.has(1) && seen.has(2) && seen.has(3) && seen.has(9), 'DCV, ACV, DCmV and ohms all seen: ' + [...seen]);

// The first DC V reading with a value: 1.2345 V +- 0.05 on 5.0000 V.
const dcv = packets.map(G.decode).find((f) => f.ok && f.mode === 1 && f.digits > 0);
const m = G.main(dcv);
check(m.unit === 'V' && m.decimals === 4 && Math.abs(m.value - 1.2345) < 0.06, 'DC V: ' + m.text + ' ' + m.unit);
const ohm = packets.map(G.decode).find((f) => f.ok && f.mode === 9 && f.range === 2);
const mo = G.main(ohm);
check(mo.unit === 'kΩ' && Math.abs(mo.value - 3300) < 10, 'ohms: ' + mo.text + ' ' + mo.unit);
const acv = packets.map(G.decode).find((f) => f.ok && f.mode === 2 && f.digits > 0);
check(G.main(acv).text.match(/^23\d\.\d\d$/) && acv.status.acdc === 'AC', 'AC V: ' + G.main(acv).text);

console.log('stream: pieces of 20 bytes, junk, a bad checksum, an echo');
const got = [], echoes = [], bad = [];
const sp = new G.StreamParser({ onPacket: (f) => got.push(f), onEcho: (e) => echoes.push(e), onBad: (e) => bad.push(e) });
const corrupt = Uint8Array.from(packets[3]); corrupt[15] ^= 1;
const stream = [0x00, 0x41, ...packets[1], ...corrupt, ...G.keyMessage('HOLD'), ...packets[2]];
for (let i = 0; i < stream.length; i += 20) sp.push(stream.slice(i, i + 20));
check(got.length === 2 && echoes.length === 1 && bad.length >= 1, `packets ${got.length}, echoes ${echoes.length}, bad ${bad.length}`);

console.log('encoder round trip, keys, clock');
const f0 = { mode: 9, range: 5, negative: false, overload: false, digits: 12345, serial: 777, calYear: 2025, calMonth: 3,
  sub1: { mode: 100, flags: 1, value: 234 }, bar: { on: true, negative: true, range: 2, segments: 13 },
  status: { auto: true, apo: true, acdc: 'DC', rel: true, bluetooth: true, hold: 1, minmax: 2 }, sub2: null, sub3: null };
const f1 = G.decode(G.encode(f0));
check(f1.ok && f1.mode === 9 && f1.range === 5 && f1.digits === 12345 && f1.serial === 777, 'round trip');
check(f1.bar.on && f1.bar.negative && f1.bar.segments === 13 && f1.status.rel && f1.status.hold === 1 && f1.status.minmax === 2, 'flags');
check(G.main(f1).text === '1.2345' && G.main(f1).unit === 'MΩ', 'main: ' + G.main(f1).text + G.main(f1).unit);
check(G.subText(f1.sub1) === '23.4 °C', 'sub: ' + G.subText(f1.sub1));
check(Buffer.from(G.keyMessage('MODE')).toString('latin1') === '\xF40505', 'MODE key');
check(Buffer.from(G.keyMessage('RANGE', true)).toString('latin1') === '\xF48181', 'long RANGE');
const cm = G.clockMessage(new Date(2026, 10, 1, 15, 42, 7));
check(Buffer.from(cm).toString('latin1') === '\xF81A0B010F2A07' + ((26 ^ 11 ^ 1 ^ 15 ^ 42 ^ 7).toString(16).toUpperCase().padStart(2, '0')), 'clock');
const ol = G.main({ mode: 1, range: 0, overload: true, digits: 0 });
check(ol.text === 'OL', 'OL');

if (failures) { console.log(failures + ' FAILED'); process.exit(1); }
console.log('all passed');
