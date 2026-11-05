/*
 * gw121-protocol.js
 *
 * The EEVblog 121GW's Bluetooth protocol, for the browser (and Node, for
 * tests). Written from the format as staging/boards/gw121/inc/Gw121Bluetooth.h
 * documents it (the stock firmware's packet builder, read in 121gw-re's
 * decompiled firmware), not from anyone's code.
 *
 *   Meter -> us: 0xF2, then 55 ASCII bytes: calibration date and serial
 *                (BCD), mode, range, value, sub fields, bargraph, status,
 *                an XOR checksum, CR LF.
 *   Us -> meter: 0xF4 kk kk          a key (01 RANGE .. 08 SETUP, + 0x80 long)
 *                0xF8 YY MM DD hh mm ss cc   set the clock
 *
 * Decimals per mode and range: the stock firmware's
 * calc_decimal_place_for_mode_range(). Units: the meter's own labels.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.GW121 = factory();
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  const START = 0xF2, KEY = 0xF4, TIME = 0xF8, PACKET_LEN = 56;

  // mode -> name, unit, and per range [decimals, prefix]
  const MODES = {
    0:  { name: 'Low Z', unit: 'V', ranges: [[1, '']] },
    1:  { name: 'DC V', unit: 'V', ac: false, ranges: [[4, ''], [3, ''], [2, ''], [1, '']] },
    2:  { name: 'AC V', unit: 'V', ranges: [[4, ''], [3, ''], [2, ''], [1, '']] },
    3:  { name: 'DC mV', unit: 'V', ranges: [[3, 'm'], [2, 'm']] },
    4:  { name: 'AC mV', unit: 'V', ranges: [[3, 'm'], [2, 'm']] },
    5:  { name: 'Temp', unit: '°C', ranges: [[2, '']] },
    6:  { name: 'Hz', unit: 'Hz', ranges: [[3, ''], [2, ''], [4, 'k'], [3, 'k'], [2, 'k']] },
    7:  { name: 'Period', unit: 's', ranges: [[4, 'm'], [3, 'm'], [2, 'm'], [1, 'm'], [2, 'm']] },
    8:  { name: 'Duty', unit: '%', ranges: [[2, '']] },
    9:  { name: 'Ohms', unit: 'Ω', ranges: [[3, ''], [2, ''], [4, 'k'], [3, 'k'], [2, 'k'], [4, 'M'], [3, 'M']] },
    10: { name: 'Continuity', unit: 'Ω', ranges: [[3, ''], [2, '']] },
    11: { name: 'Diode', unit: 'V', ranges: [[4, ''], [3, '']] },
    12: { name: 'Capacitance', unit: 'F', ranges: [[2, 'n'], [1, 'n'], [3, 'µ'], [2, 'µ'], [1, 'µ'], [0, 'µ']] },
    13: { name: 'AC µVA', unit: 'VA', ranges: [[2, 'µ'], [1, 'µ'], [1, 'µ'], [0, 'µ']] },
    14: { name: 'AC mVA', unit: 'VA', ranges: [[3, 'm'], [2, 'm'], [2, 'm'], [1, 'm']] },
    15: { name: 'AC VA', unit: 'VA', ranges: [[1, ''], [0, ''], [3, ''], [2, '']] },
    16: { name: 'AC µA', unit: 'A', ranges: [[3, 'µ'], [2, 'µ']] },
    17: { name: 'DC µA', unit: 'A', ranges: [[3, 'µ'], [2, 'µ']] },
    18: { name: 'AC mA', unit: 'A', ranges: [[4, 'm'], [3, 'm']] },
    19: { name: 'DC mA', unit: 'A', ranges: [[4, 'm'], [3, 'm']] },
    20: { name: 'AC A', unit: 'A', ranges: [[2, 'm'], [4, ''], [3, '']] },
    21: { name: 'DC A', unit: 'A', ranges: [[2, 'm'], [4, ''], [3, '']] },
    22: { name: 'DC µVA', unit: 'VA', ranges: [[2, 'µ'], [1, 'µ'], [1, 'µ'], [0, 'µ']] },
    23: { name: 'DC mVA', unit: 'VA', ranges: [[3, 'm'], [2, 'm'], [2, 'm'], [1, 'm']] },
    24: { name: 'DC VA', unit: 'VA', ranges: [[1, ''], [0, ''], [3, ''], [2, '']] },
  };
  const PREFIX = { '': 1, 'k': 1e3, 'M': 1e6, 'm': 1e-3, 'µ': 1e-6, 'n': 1e-9 };

  const KEYS = { RANGE: 1, HOLD: 2, REL: 3, PEAK: 4, MODE: 5, MINMAX: 6, MEM: 7, SETUP: 8 };

  const hex = (n) => (n & 0x0F).toString(16).toUpperCase();
  function unhex(c) {
    if (c >= 0x30 && c <= 0x39) return c - 0x30;
    if (c >= 0x41 && c <= 0x46) return c - 0x41 + 10;
    return -1;
  }
  function byteAt(b, i) {
    const h = unhex(b[i]), l = unhex(b[i + 1]);
    return (h < 0 || l < 0) ? -1 : (h << 4) | l;
  }
  function bcd(b, from, n) {
    let v = 0;
    for (let i = from; i < from + n; i++) {
      if (b[i] < 0x30 || b[i] > 0x39) return -1;
      v = v * 10 + (b[i] - 0x30);
    }
    return v;
  }

  // 56 bytes -> { ok, error?, ...fields }
  function decode(b) {
    if (!b || b.length < PACKET_LEN - 2) return { ok: false, error: 'short' };
    if (b[0] !== START) return { ok: false, error: 'start' };
    const v = [];
    let ck = 0;
    for (let i = 0; i < 21; i++) {
      const x = byteAt(b, 10 + 2 * i);
      if (x < 0) return { ok: false, error: 'not hex' };
      v.push(x);
      ck ^= x;
    }
    const sent = byteAt(b, 52);
    if (sent !== ck) return { ok: false, error: 'checksum' };
    const ym = bcd(b, 1, 4), sn = bcd(b, 5, 5);
    const sub = (o) => ({ mode: v[o], flags: v[o + 1], value: (v[o + 2] << 8) | v[o + 3] });
    return {
      ok: true,
      calYear: ym >= 0 ? 2000 + Math.floor(ym / 100) : null, calMonth: ym >= 0 ? ym % 100 : null,
      serial: sn,
      mode: v[0],
      range: v[1] & 0x3F, overload: !!(v[1] & 0x80), negative: !!(v[1] & 0x40),
      digits: (v[2] << 8) | v[3],
      sub1: sub(4),
      bar: { on: !(v[8] & 0x10), negative: !!(v[8] & 0x04), range: (v[8] & 0x08) ? 3 : (v[8] & 0x03), segments: v[9] & 0x1F },
      status: {
        lpf: !!(v[10] & 0x40), peak1ms: !!(v[10] & 0x20),
        acdc: ['', 'DC', 'AC', 'AC+DC'][(v[10] >> 3) & 3],
        auto: !!(v[10] & 0x04), apo: !!(v[10] & 0x02), battLow: !!(v[10] & 0x01),
        bluetooth: !!(v[11] & 0x40), danger: !!(v[11] & 0x20), rel: !!(v[11] & 0x10), dbm: !!(v[11] & 0x08),
        minmax: v[11] & 0x07,
        logging: !!(v[12] & 0x10), hold: (v[12] >> 2) & 0x03,
      },
      sub2: sub(13), sub3: sub(17),
    };
  }

  // The main display: text as the meter shows it, the value in SI, the unit.
  function main(f) {
    const m = MODES[f.mode];
    const rg = m && m.ranges[f.range] ? m.ranges[f.range] : [0, ''];
    const dec = rg[0], prefix = rg[1];
    const unit = prefix + (m ? m.unit : '?');
    if (f.overload) return { text: 'OL', value: NaN, unit, mode: m ? m.name : 'mode ' + f.mode, decimals: dec };
    const shown = f.digits / Math.pow(10, dec);
    const text = (f.negative ? '-' : '') + shown.toFixed(dec);
    const value = (f.negative ? -shown : shown) * PREFIX[prefix];
    return { text, value, unit, mode: m ? m.name : 'mode ' + f.mode, decimals: dec, shown: f.negative ? -shown : shown };
  }

  // A sub field as text, or '' for none.
  function subText(s) {
    if (!s || (s.mode === 0 && s.flags === 0 && s.value === 0)) return '';
    const dec = s.flags & 0x0F, neg = (s.flags & 0x40) ? '-' : '';
    const num = () => (s.flags & 0x80) ? 'OL' : neg + (s.value / Math.pow(10, dec)).toFixed(dec);
    switch (s.mode) {
      case 100: return num() + ' °C';
      case 105: return num() + ' °F';
      case 110: return 'BAT ' + num();
      case 120: return 'APO on';
      case 125: return 'APO off';
      case 130: return 'Year ' + s.value;
      case 135: return 'Date ' + (s.value >> 8) + '/' + (s.value & 0xFF);
      case 140: return 'Time ' + String(s.value >> 8).padStart(2, '0') + ':' + String(s.value & 0xFF).padStart(2, '0');
      case 150: return 'Burden ' + num() + ' mV';
      case 155: return 'Burden on';
      case 156: return 'Burden off';
      case 160: return 'LCD ' + s.value;
      case 180: return num() + ' dBm';
      case 190: return 'Interval ' + s.value;
      case 6:   return num() + ((s.flags & 0x20) ? ' kHz' : ' Hz');
      default: {
        const m = MODES[s.mode];
        return m ? num() + ' ' + m.unit + ' (' + m.name + ')' : 'sub ' + s.mode + ': ' + s.value;
      }
    }
  }

  // Bytes from the link (BLE notifications arrive in pieces of 20 or so)
  // into packets: onPacket(frame, bytes), onEcho(bytes), onBad(error, bytes).
  class StreamParser {
    constructor(handlers) { this.h = handlers || {}; this.buf = []; this.good = 0; this.bad = 0; this.skipped = 0; }
    push(bytes) {
      for (const x of bytes) this.buf.push(x);
      for (;;) {
        while (this.buf.length && this.buf[0] !== START && this.buf[0] !== KEY && this.buf[0] !== TIME) { this.buf.shift(); this.skipped++; }
        if (!this.buf.length) return;
        const need = this.buf[0] === START ? PACKET_LEN : this.buf[0] === KEY ? 5 : 15;
        if (this.buf.length < need) return;
        const pkt = this.buf.slice(0, need);
        if (pkt[0] === START) {
          const f = decode(pkt);
          if (f.ok) { this.good++; this.buf.splice(0, need); this.h.onPacket && this.h.onPacket(f, pkt); }
          else { this.bad++; this.buf.shift(); this.h.onBad && this.h.onBad(f.error, pkt); }
        } else {
          this.buf.splice(0, need);
          this.h.onEcho && this.h.onEcho(pkt);
        }
      }
    }
  }

  function keyMessage(key, longPress) {
    const c = (KEYS[key] || key) | (longPress ? 0x80 : 0);
    const a = hex(c >> 4).charCodeAt(0), b = hex(c).charCodeAt(0);
    return new Uint8Array([KEY, a, b, a, b]);
  }

  function clockMessage(d) {
    const v = [d.getFullYear() % 100, d.getMonth() + 1, d.getDate(), d.getHours(), d.getMinutes(), d.getSeconds()];
    const out = [TIME];
    let ck = 0;
    for (const x of v) { out.push(hex(x >> 4).charCodeAt(0), hex(x).charCodeAt(0)); ck ^= x; }
    out.push(hex(ck >> 4).charCodeAt(0), hex(ck).charCodeAt(0));
    return new Uint8Array(out);
  }

  // A packet from fields: for the page's demo mode, and tests.
  function encode(f) {
    const out = new Uint8Array(PACKET_LEN);
    out[0] = START;
    const ym = String(((f.calYear || 2026) % 100) * 100 + (f.calMonth || 1)).padStart(4, '0');
    const sn = String(f.serial || 0).padStart(5, '0');
    for (let i = 0; i < 4; i++) out[1 + i] = ym.charCodeAt(i);
    for (let i = 0; i < 5; i++) out[5 + i] = sn.charCodeAt(i);
    const s = f.status || {}, bar = f.bar || { on: false };
    const sub = (x) => x ? [x.mode, x.flags, x.value >> 8, x.value & 0xFF] : [0, 0, 0, 0];
    const acdc = { '': 0, DC: 1, AC: 2, 'AC+DC': 3 }[s.acdc || ''] || 0;
    const v = [
      f.mode, (f.range & 0x3F) | (f.overload ? 0x80 : 0) | (f.negative ? 0x40 : 0), f.digits >> 8, f.digits & 0xFF,
      ...sub(f.sub1),
      (bar.on ? 0 : 0x10) | (bar.negative ? 0x04 : 0) | (bar.range === 3 ? 0x08 : (bar.range || 0) & 3), bar.segments || 0,
      (s.lpf ? 0x40 : 0) | (s.peak1ms ? 0x20 : 0) | (acdc << 3) | (s.auto ? 4 : 0) | (s.apo ? 2 : 0) | (s.battLow ? 1 : 0),
      (s.bluetooth ? 0x40 : 0) | (s.danger ? 0x20 : 0) | (s.rel ? 0x10 : 0) | (s.dbm ? 0x08 : 0) | (s.minmax || 0),
      (s.logging ? 0x10 : 0) | ((s.hold || 0) << 2),
      ...sub(f.sub2), ...sub(f.sub3),
    ];
    let ck = 0;
    v.forEach((x, i) => { out[10 + 2 * i] = hex(x >> 4).charCodeAt(0); out[11 + 2 * i] = hex(x).charCodeAt(0); ck ^= x; });
    out[52] = hex(ck >> 4).charCodeAt(0); out[53] = hex(ck).charCodeAt(0);
    out[54] = 13; out[55] = 10;
    return out;
  }

  return { START, KEY, TIME, PACKET_LEN, MODES, KEYS, decode, main, subText, StreamParser, keyMessage, clockMessage, encode };
}));
