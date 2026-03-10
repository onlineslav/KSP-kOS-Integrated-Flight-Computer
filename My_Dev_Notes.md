Would be good if the FC could automatically pull the AC config from the name of the aircraft?

Is there a way we could limit bank angle during all phases of flight (except maybe in space but we can patch that in later)? AA doesn't have a bank limiter

Add these to cfg files?
Vs: stall speed in the configuration you mean

Vs0: stall speed in landing configuration
Usually: gear down, landing flaps, final landing mass

Vref: reference approach speed on final
Usually based on 1.3 × Vs0

Vapp: actual flown approach speed
Often Vref plus a small additive for gusts / handling margin

Vtd: target touchdown speed
Usually a bit below Vref, but still safely above stall

Vfe: maximum flap extension speed

