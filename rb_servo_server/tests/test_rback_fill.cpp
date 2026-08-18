#include <iostream>
#include <string>
#include <vector>

#include "rb_servo/robot/rback_fill.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::RbackScan scanBatch(const std::vector<std::string>& responses) {
    rb_servo::RbackScan scan;
    for (const auto& line : responses) {
        rb_servo::scanRbackFillLine(line, scan);
    }
    return scan;
}

// The nominal case: one ACK per streamed command.
bool testSingleTokenParses() {
    const auto scan = scanBatch({"RBACK[5]"});
    RB_CHECK(scan.last_fill.has_value());
    RB_CHECK(*scan.last_fill == 5);
    RB_CHECK(scan.samples == 1);
    RB_CHECK(scan.unparsed == 0);
    return true;
}

// Occupancy zero is a real reading (queue empty), not "no observation". It is
// also the underrun case a cadence controller must react to, so it must never
// collapse into nullopt.
bool testZeroFillIsAnObservation() {
    const auto scan = scanBatch({"RBACK[0]"});
    RB_CHECK(scan.last_fill.has_value());
    RB_CHECK(*scan.last_fill == 0);
    RB_CHECK(scan.samples == 1);
    return true;
}

// Several ACKs can land in one drain when the loop runs late. The newest value
// is the one that describes the queue now, so the last token must win -- taking
// the first would report a stale (smaller) occupancy and under-correct.
bool testLastTokenWinsWithinAndAcrossLines() {
    const auto within = scanBatch({"RBACK[3]RBACK[7]"});
    RB_CHECK(within.last_fill.has_value());
    RB_CHECK(*within.last_fill == 7);
    RB_CHECK(within.samples == 2);
    RB_CHECK(within.unparsed == 0);

    const auto across = scanBatch({"RBACK[3]", "RBACK[7]", "RBACK[4]"});
    RB_CHECK(across.last_fill.has_value());
    RB_CHECK(*across.last_fill == 4);
    RB_CHECK(across.samples == 3);
    return true;
}

// The SDK splits the response stream on '\n' with no carry buffer, so a token
// straddling a TCP chunk boundary arrives as two fragments. Both halves must be
// dropped (counted as unparsed) rather than parsed into a wrong number -- a
// bogus occupancy would drive a cadence controller the wrong way.
bool testSplitTokenIsDroppedNotGuessed() {
    const auto scan = scanBatch({"RBAC", "K[7]"});
    RB_CHECK(!scan.last_fill.has_value());
    RB_CHECK(scan.samples == 0);
    RB_CHECK(scan.unparsed == 2);

    // A token truncated at the closing bracket is equally unusable.
    const auto truncated = scanBatch({"RBACK[12"});
    RB_CHECK(!truncated.last_fill.has_value());
    RB_CHECK(truncated.unparsed == 1);
    return true;
}

// Non-RBACK traffic (info/warn/error popups) shares the channel and must be
// counted as unparsed without disturbing the reading.
bool testUnrelatedTrafficIsCountedNotParsed() {
    const auto scan = scanBatch({
        "info[system][the command was executed]",
        "RBACK[6]",
        "warn[collision][something]",
    });
    RB_CHECK(scan.last_fill.has_value());
    RB_CHECK(*scan.last_fill == 6);
    RB_CHECK(scan.samples == 1);
    RB_CHECK(scan.unparsed == 2);
    return true;
}

// Garbled fragments must never throw out of the servo send path and must never
// be mistaken for a reading.
bool testMalformedTokensAreRejected() {
    const auto scan = scanBatch({
        "RBACK[]",            // no digits
        "RBACK[-3]",          // sign is not part of the wire format
        "RBACK[ 4]",          // whitespace
        "RBACK[4x]",          // trailing garbage
        "RBACK[1234567890]",  // absurd digit run, would overflow a narrow parse
    });
    RB_CHECK(!scan.last_fill.has_value());
    RB_CHECK(scan.samples == 0);
    RB_CHECK(scan.unparsed == 5);
    return true;
}

// An empty drain is the common case when the box has nothing queued to say; it
// must report "no observation", not a stale or zero reading.
bool testEmptyBatchYieldsNoObservation() {
    const auto scan = scanBatch({});
    RB_CHECK(!scan.last_fill.has_value());
    RB_CHECK(scan.samples == 0);
    RB_CHECK(scan.unparsed == 0);
    return true;
}

}  // namespace

int main() {
    if (!testSingleTokenParses()) return 1;
    if (!testZeroFillIsAnObservation()) return 1;
    if (!testLastTokenWinsWithinAndAcrossLines()) return 1;
    if (!testSplitTokenIsDroppedNotGuessed()) return 1;
    if (!testUnrelatedTrafficIsCountedNotParsed()) return 1;
    if (!testMalformedTokensAreRejected()) return 1;
    if (!testEmptyBatchYieldsNoObservation()) return 1;
    return 0;
}
