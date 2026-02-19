# Shared HaLow mesh + overlay (distance-vector routing over HaLow Ethernet).
# Used by gateway (main) and sensor_unit.
idf_component_register(
    SRCS "halow_mesh.c" "halow_mesh_overlay.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES morselib mm_shims mmipal heap
)
