# HaLow host tuning (Linux / Rockchip)

These tunings apply to the **Linux host** that runs the HaLow AP and talks to the Morse Micro HaLow chip over SPI (e.g. Rockchip SoC). They are not part of the ESP32 bridge firmware in this repo.

---

## First tranche

Target: reduce SPI/CS and scheduling jitter. Typical result **16–18 Mbps** (occasional 20 Mbps) before tuning; iperf UDP TX/RX similar.

| System | Improvement | Action |
|--------|-------------|--------|
| **SPI driver** | Disable all power management | Turn off runtime PM / autosuspend on the SPI controller so clocks and CS are not delayed by power management. |
| **SPI driver** | Enable `rockchip,rt` DT parameter | Set `rockchip,rt` in the SPI node in the device tree so the controller runs in real-time mode (reduces extra delays around CS and clock). |
| **DMAC driver** | Disable all power management | Disable runtime PM on the DMA controller to avoid latency spikes when DMA is used. |
| **Morse driver** | Align SPI transactions that use DMA | Ensure buffers and transaction sizes used for DMA SPI transfers are properly aligned (e.g. cache-line / DMA alignment) to avoid extra copies or stalls. |
| **Performance governor** | Use `performance` instead of `schedutil` | Set CPU frequency governor to `performance` on the cores used for SPI/DMA so frequency doesn’t drop and cause transaction delays. |

Main issues addressed: spikes in transaction delay, and unnecessary delays before/after CS active/inactive relative to clock start/end and between CS active periods (scheduling related).

---

## Second tranche

Target: consistent **~21 Mbps** on SPI bus tests and **~20 Mbps** TX/RX on iperf.

| System | Improvement | Action |
|--------|-------------|--------|
| **IRQ pinning** | Pin DMA and SPI to a dedicated big core each | Move the DMA and SPI IRQ handlers to run on a dedicated big core (e.g. one core for DMA, one for SPI) so they don’t compete with other work. |
| **IRQ pinning** | Move other IRQs off cores 4 and 5 | Ensure other device IRQs are not affined to cores 4 and 5 so SPI/DMA have exclusive use of those cores. |
| **CPU states** | Disable sleep on CPUs 4 and 5 | Disable idle/sleep states (e.g. C-states) on cores 4 and 5 so they don’t enter deep sleep and add wake-up latency to SPI/DMA handling. |

---

## Summary

- **First tranche**: SPI/DMAC power management off, `rockchip,rt` for SPI, DMA-aligned Morse SPI transactions, `performance` governor.
- **Second tranche**: DMA and SPI on dedicated big cores (4 and 5), other IRQs off those cores, no sleep states on 4 and 5.

Apply these on the HaLow AP host (device tree, kernel config or module options, and userspace governor/IRQ affinity scripts) as appropriate for your BSP.
