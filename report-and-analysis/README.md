# Report and Analysis

Project report and analysis for the **AXI4-Stream to PCIe 6.0-style link** (RTL bridge with FLIT framing, CRC/FEC protection and replay).

## Contents

| File | Description |
|------|-------------|
| [`REPORT_FILE.pdf`](REPORT_FILE.pdf) | Full project report (architecture, design details, verification plan, synthesis results) |
| `README.md` | This summary |

Related folders in the repository:

| Folder | Content |
|--------|---------|
| `rtl/` | Synthesizable Verilog source |
| `tb/` | Testbenches |
| `sim/` | Simulation scripts and outputs |
| `synthesis/` | Genus synthesis reports and the mapped netlist |

## What the report covers

1. **Problem statement and use cases:** network packet processing (SmartNIC) and ML-inference accelerators.
2. **Architecture:** three layers: interconnect, link layer, protocol-to-PHY.
3. **Design details:**
   - Packet-atomic N:1 QoS arbitration (tqos, packet locking, round-robin among equal priorities)
   - 24-byte record header (TLP-style) carrying all AXI-Stream sideband signals
   - 6-byte DLP with type, sequence number, record count and used bytes
   - 256-byte FLIT: 236 B payload + 6 B DLP + 8 B CRC + 6 B FEC
   - RS-style CRC over GF(2^8), 32 bytes per clock (3-stage pipeline)
   - 3-way interleaved FEC with single-symbol correction per group
   - Sequence numbers, ACK/NAK and go-back-N replay (8-entry window)
   - Four-lane PHY mapping: byte striping, scrambling, Gray coding, PAM4 precoding
4. **Verification:** two-instance back-to-back testbench with scoreboard, error injection and a 12-test plan.
5. **Logic synthesis:** Cadence Genus, area / power / timing / design-check results.

## Synthesis results (summary)

Tool: Cadence Genus 20.11-s111_1 | Library: `tsl18fs120_scl_ss` | Top: `pcie6_top` | Clock: 10 ns (100 MHz)

| Metric | Result |
|--------|--------|
| Setup timing | WNS 0 ps, TNS 0, 0 violating paths (all 40 reported paths met) |
| Cell area | 6,505,672.955 (library area units) |
| Instances | 203,069 total (50,042 sequential, 153,027 combinational) |
| Power | 542.259 mW (register 359.088 mW, logic 183.171 mW) |
| Design check | No unresolved references, multi-driven nets or undriven ports; 3 unused undriven hierarchical pins (`u_packer/s_record[514:512]`) |

Area of the blocks kept as separate hierarchy:

| Block | Cells | Area | Share |
|-------|------:|-----:|------:|
| Packer | 13,458 | 470,968.722 | 7.24% |
| Tx CRC pipeline | 5,759 | 150,179.091 | 2.31% |
| Rx CRC pipeline | 5,756 | 150,157.151 | 2.31% |
| PHY Tx | 3,903 | 99,921.841 | 1.54% |
| PHY Rx | 2,881 | 79,160.130 | 1.22% |

The remaining 85.39% of the area was flattened into the top module.

### Key observations

- The **PHY scrambler/precoder is the timing-critical block**: 8 paths have 5 ps of slack or less, and the worst paths have 33 to 53 cells. The Rx FEC/CRC check has 181 ps or more of slack and the replay pointer 408 ps.
- The CRC pipelines are about 2.3% of the area each, so the CRC is not a dominant cost even at 32 bytes per clock.
- Registers account for 66% of the power. The FLIT-sized buffers (replay buffer, receive slots, depacketizer register) are the main targets for area and power reduction.
- At 100 MHz the raw line rate is 25.6 Gb/s (6.4 Gb/s per lane), about one tenth of the full PCIe 6.0 per-lane rate.

## Limitations

- **Synthesis numbers are pre-layout:** default wireload, ideal clock, setup checks only. Place-and-route, clock-tree synthesis, hold analysis and GDSII have not been run.
- Synthesis used Cadence Genus; the OpenLane/OpenROAD flow named in the problem statement is future work.
- The CRC first-consecutive-root (`FCR = 0`) is a placeholder; the CRC/FEC structure follows public descriptions of PCIe 6.0, not the licensed specification.
- There is no LTSSM, link training, credit flow control or analog/equalization modelling.
- The synthesized netlist has not been gate-level simulated yet.

## Future work

- OpenLane/OpenROAD flow through to GDSII, with post-layout timing
- Pipeline the PHY precoder/scrambler to raise the clock rate
- Reduce buffer sizes and add clock gating
- Gate-level simulation, SVA assertions and a Python golden model
- FPGA bring-up

## Rebuilding the report (optional)

If you add the LaTeX source (`report_final.tex`) and your block-diagram image (`image.png`) to this folder:

```bash
pdflatex report_final.tex
pdflatex report_final.tex   # second pass resolves cross-references
```
