# Algorithm Note

Score every drone for each request, pick the cheapest feasible one. Every decision is logged.

Eligible only if: payload ≤ 2.5 kg, status is Idle/Returning/Holding, SoC ≥ 15%,
`outbound + return + 12% reserve` fits in degraded capacity, arrival is before deadline.

```text
usable = 520Wh * health% / 100;  need = energy(base->dst, m) + energy(dst->base, 0) + reserve
v(m) = 12.0 * (1 - 0.35 * m / 2.5)
cost = deadline + energy + workload + pad_wait + low_battery
```

`GET /audit` returns the picked drone plus per-drone ETA/energy/margin/cost.
