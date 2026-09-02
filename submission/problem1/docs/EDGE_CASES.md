# Edge Cases

## Impossible Deadline

The dispatcher computes the fastest theoretical delivery from base before scoring drones. Requests that cannot be delivered even by the fastest model are rejected immediately.

## Overweight Request

Payload above `2.5 kg` is rejected with an explicit reason.

## Pad Gridlock

The pad manager reserves free pads, otherwise queues returning drones by risk. Drones below `15%` SoC enter the front of the queue and may preempt non-occupied reservations.

## Fleet Saturation

When all drones are committed, all candidates are marked ineligible and the request remains queued. The queue is reevaluated on every simulation tick.

## Competing Urgent Requests

Requests are sorted by deadline when reprocessed. Candidate costs include deadline urgency and workload balance, so a scarce available drone is assigned to the request with the strongest utility.

## Diversion Cancel / Pad Loss

`PadManager.release_reservation()` frees a drone reservation when a routing decision changes. A later request can then reserve that pad.

## Reassignment Mid-Flight

The dispatcher model keeps route and active-request state separate. A reassignment should clear the old request, write a failure/reassignment reason, release any pad reservation, and install the new route.

## Telemetry Anomaly

The telemetry filter rejects out-of-range SoC, abrupt SoC jumps, impossible coordinates, and physically impossible implied speeds.

## Wind / Slowdown

Telemetry carries `speed_factor`. If wind reduces it to `0.8`, subsequent ETA and safety calculations become stricter. Slow drones can be marked at risk, queued for mitigation, or reassigned.

## Communication Failure

The server marks airborne drones `Lost` after heartbeat timeout. The client state machine enters hold and then return-home mode when server communication remains unavailable.
