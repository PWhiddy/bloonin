"""Callsign families and frame order shared by collection and decoding."""

# Same order as wspr_beacon_message(); each sequence uses one frozen fix.
FRAMES_BY_VARIANT = {
    variant: tuple((name, f"ZL3X{variant}{suffix}", maximum)
                   for name, suffix, maximum in (
                       ("coarse", "Z", 32399),
                       ("fine", "F", 32399),
                       ("altitude", "A", 32767),
                       ("extra", "T", 28799),
                   ))
    for variant in ("X", "Y")
}
CALLSIGNS = tuple(call for frames in FRAMES_BY_VARIANT.values() for _, call, _ in frames)
