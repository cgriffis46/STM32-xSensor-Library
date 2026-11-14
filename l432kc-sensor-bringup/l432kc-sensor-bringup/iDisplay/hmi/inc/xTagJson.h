#pragma once
#include <stddef.h>
#include "iTagSource.h"

namespace idisplay {

// The answer to GET /api/tags?names=a,b,c as the HMI's preview page reads
// it in Live mode (hmi/tools/preview_template.html, poll(), and
// hmi_render.js's tagValue()), from any iTagSource:
//
//   {"tags":[{"name":"tank.level","value":72.46},
//            {"name":"pump1.run","value":true},
//            {"name":"probe.ph","value":null},
//            {"name":"nope","error":"not found"}]}
//
// value: a number (6 significant digits, as PLCTransport's API sends REAL
// values; the page rounds it to the item's decimals), true or false, or
// null when the tag can't be read now or isn't a finite number (the page
// shows "--"). error: the source has no such tag (the page lists it as
// "Not on the device").
//
// So a device without PLCTransport (an ESP32 with its own variables in an
// xTagTable, say) can serve the page itself: its web server routes the
// request here and sends the buffer with Content-Type application/json.
// No iNetTransport dependency: the caller hands over the query value and
// a buffer.
//
// names: the query parameter's value as it arrived (comma separated, each
// percent-encoded the way the page's encodeURIComponent() does it).
// Names longer than kMaxName bytes once decoded are answered "not found";
// at most kMaxNames are answered, the rest ignored.
//
// Returns the length written (without the terminating 0), or 0 if it
// didn't fit in cap (nothing usable is left in out) or out is null. No
// allocation, no printf. Reads each tag once, through the source, from
// the caller's thread (an iTagSource's read() never waits on a bus).
size_t writeTagsJson(iTagSource& source, const char* names, char* out, size_t cap);

static constexpr size_t kTagJsonMaxName = 64;
static constexpr size_t kTagJsonMaxNames = 64;

// A number as the API writes it: up to 6 significant digits, no trailing
// zeros, "null" for NaN, infinity or a magnitude the formatter can't take
// (1e15 or more). Returns its length, 0 if it didn't fit. Public for tests.
size_t formatJsonNumber(double v, char* out, size_t cap);

} // namespace idisplay
