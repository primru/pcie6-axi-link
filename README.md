# AXI4-Stream to PCIe 6 style link (SARCathon 2026)

Layout: rtl/ (design), tb/ (self-checking testbench), sim/ (run script + log).
Run: `sh sim/run.sh` (Icarus Verilog 12, needs -g2012 for the testbench). Result: `ALL TESTS PASSED`.

## Data path
AXI4-Stream slave IFs (skid, address travels with each beat) -> N:1 crossbar (tqos priority at packet
boundaries, round-robin ties, locked until tlast) -> link layer -> 4-lane PHY layer, and the full reverse path.

Link layer TX: record builder -> flit packer -> ACK/NAK insertion -> flit_protect (RS-style CRC + 3-way FEC) ->
replay buffer -> transmitter. RX: flit receiver -> flit_check (FEC syndrome + single-symbol correction per group,
then CRC) -> rx_link_ctl (sequence, ACK/NAK) -> record depacketizer -> AXI-S style output.

## What was added or fixed vs the previous version
1. Arbiter locks to a master until tlast (no packet interleaving), combinational zero-latency mux.
2. tkeep, tstrb, tid, tdest, tuser, tlast, address and rw travel in a 24 B record header; reads carry no data.
3. Rx path with FEC correction, CRC check, sequence numbers, cumulative ACK, NAK, go-back-N replay buffer,
   replay timeout, duplicate handling.
4. PHY: byte striping across lanes (lane0 gets the earliest bytes), scrambler, Gray, mod-4 precoder, and the inverse on Rx.
5. Bug fix: address was sampled when the beat was delivered, not accepted; it now travels with the beat.
6. Bug fix: flit_protect FIFO-full test compared a wrapped 32-bit difference and could overwrite a slot.
7. Testbench: two links back to back, scoreboard, 12 tests, error injection (see below).

## Assumptions (all small)
- A record is 24 B header + 64 B data field (88 B write, 24 B read). Records never straddle FLITs
  (about 2 writes per FLIT). A partial FLIT is sent after 16 idle clocks (FLUSH_TMO) or when nothing more fits.
- Packets must end with tlast; the crossbar stays locked until then. Strict tqos priority can starve low-QoS masters.
- m_drw = 1 means a data beat (write or completion data), 0 means a read request. Completions are ordinary data
  records matched by tid at application level.
- DLP layout, 10-bit sequence number, ACK/NAK control FLITs are custom, not the licensed PCIe 6.0 format.
- CRC is the existing RS-over-GF(2^8) generator with placeholder FCR=0. FEC is 3-way byte interleave with 2 check
  bytes per group (single-symbol correction per group). Both are concept-level, not spec-exact.
- Scrambler polynomial, seeds and the mod-4 precoder are not claimed to match the spec. Scrambler and precoder
  restart at each FLIT start. No training, ordered sets, lane deskew or clock recovery: Rx assumes the 8 beats
  of a FLIT arrive contiguously and aligned.
- Rx output has no flow control toward the wire: if the sink stalls and the Rx buffers fill, FLITs are dropped
  and recovered by NAK/replay.
- Both ends use the same parameters. Replay window = 2^REPLAY_LOG2 = 8 FLITs by default (16 kbit buffer; lower it
  if area is a problem).

## Tests (tb/tb_pcie6_link.v)
T1 reset/idle, T2 single write, T3 read request, T4 packet locking + QoS, T5 partial tkeep/tstrb,
T6 network workload (4 sources, random lengths, both directions), T7 ML workload (64-beat tensor, read requests,
completions), T8 single symbol error corrected by FEC, T9 uncorrectable error -> CRC fail, NAK, replay,
T10 ACK path corrupted -> window stall, replay timeout, recovery, T11 random sink backpressure, T12 reset.
A mutation check (FEC correction disabled) makes T8 fail, so the checks are live.

## Not covered
- The drop-on-overflow path (st_drop) is not hit by T11; add a longer stall to exercise it.
- No SVA properties, no gate-level or timing results. Not run through Yosys/OpenLane by me.
- Reset in the middle of active traffic is not tested (T12 resets an idle link).
