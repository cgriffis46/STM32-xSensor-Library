# 121GW in the browser (Web Bluetooth)

`index.html` connects straight to a 121GW, or to a stand-in meter (below),
over Bluetooth LE. There's no app and nothing to install. It shows:

* the display (value, unit, AC/DC, annunciators, bargraph, sub display);
* the 8 keys (click: short press; hold 0.7 s: long press);
* a chart with min / max / average;
* CSV logging;
* the raw packets.

**Demo** makes readings up in the page, so it can be tried with no meter at
all.

The protocol is in `gw121-protocol.js`. It is written from the format in
`staging/boards/gw121/inc/Gw121Bluetooth.h`, and is the same packets and
key messages EEVblog's app uses.

## Opening it

Web Bluetooth only works on a secure page, and only in Chrome or Edge
(Windows, macOS, Linux, Android, ChromeOS). Safari (iPhone, iPad) and
Firefox don't have it.

* **Locally:**

  ```bash
  cd staging/tools/web-bluetooth && python3 -m http.server 8000
  ```

  Then open http://localhost:8000 in Chrome.
* **Online:** any https host, e.g. GitHub Pages. It's two static files.

Connect lists every Bluetooth device nearby. The page then asks the chosen
device for each service in its list (under "Bluetooth services") and uses
the first one it finds. In that service, it takes the first notifying
characteristic for data and the first writable one for keys. The services
it tries by default:

| UUID | What |
|------|------|
| `0bd51666-e7cb-469b-8e4d-2742f1ba77cc` | Bluegiga's serial service. **A guess** for the 121GW's BLE112, not yet checked on a meter |
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | Nordic UART, which Adafruit's Bluefruit modules use |

If a real 121GW turns out to use another UUID, read it with nRF Connect (a
free phone app) and add it under "Bluetooth services".

## Tests

```bash
make -C staging/boards/gw121/sim/test packets
```

```bash
node staging/tools/web-bluetooth/test/protocol.test.js staging/boards/gw121/sim/test/packets.txt
```

The first command makes packets.txt from our C++ firmware stack (the
simulator). The second checks that the page's decoder reads them, along
with a stream cut into BLE-sized pieces, a bad checksum, an echo, the
encoder round trip, and the key and clock messages.
