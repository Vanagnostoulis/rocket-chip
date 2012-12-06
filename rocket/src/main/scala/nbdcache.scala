package rocket

import Chisel._
import Node._
import Constants._
import uncore._
import Util._

case class DCacheConfig(sets: Int, ways: Int, co: CoherencePolicy,
                        nmshr: Int, nrpq: Int, nsdq: Int, ntlb: Int,
                        reqtagbits: Int = -1, databits: Int = -1,
                        narrowRead: Boolean = true)
{
  require(isPow2(sets))
  require(isPow2(ways)) // TODO: relax this
  def lines = sets*ways
  def dm = ways == 1
  def ppnbits = PADDR_BITS - PGIDX_BITS
  def vpnbits = VADDR_BITS - PGIDX_BITS
  def pgidxbits = PGIDX_BITS
  def offbits = OFFSET_BITS
  def paddrbits = ppnbits + pgidxbits
  def lineaddrbits = paddrbits - offbits
  def idxbits = log2Up(sets)
  def waybits = log2Up(ways)
  def untagbits = offbits + idxbits
  def tagbits = lineaddrbits - idxbits
  def ramoffbits = log2Up(MEM_DATA_BITS/8)
  def databytes = databits/8
  def wordoffbits = log2Up(databytes)
  def isNarrowRead = narrowRead && databits*ways % MEM_DATA_BITS == 0
}

abstract class ReplacementPolicy
{
  def way: UFix
  def miss: Unit
  def hit: Unit
}

class RandomReplacement(implicit conf: DCacheConfig) extends ReplacementPolicy
{
  private val replace = Bool()
  replace := Bool(false)
  val lfsr = LFSR16(replace)

  def way = if (conf.dm) UFix(0) else lfsr(conf.waybits-1,0)
  def miss = replace := Bool(true)
  def hit = {}
}

object StoreGen
{
  def apply(r: HellaCacheReq) = new StoreGen(r.typ, r.addr, r.data)
  def apply(r: hwacha.io_dmem_req_bundle) = new StoreGen(r.typ, r.addr, r.data)
  def apply(typ: Bits, addr: Bits, data: Bits = Bits(0)) = new StoreGen(typ, addr, data)
}

class StoreGen(typ: Bits, addr: Bits, dat: Bits)
{
  val byte = typ === MT_B || typ === MT_BU
  val half = typ === MT_H || typ === MT_HU
  val word = typ === MT_W || typ === MT_WU
  def mask =
    Mux(byte, Bits(  1) <<     addr(2,0),
    Mux(half, Bits(  3) << Cat(addr(2,1), Bits(0,1)),
    Mux(word, Bits( 15) << Cat(addr(2),   Bits(0,2)),
              Bits(255))))
  def data =
    Mux(byte, Fill(8, dat( 7,0)),
    Mux(half, Fill(4, dat(15,0)),
    Mux(word, Fill(2, dat(31,0)),
                      dat)))
}

class LoadGen(typ: Bits, addr: Bits, dat: Bits)
{
  val t = StoreGen(typ, addr, dat)
  val sign = typ === MT_B || typ === MT_H || typ === MT_W || typ === MT_D

  val wordShift = Mux(addr(2), dat(63,32), dat(31,0))
  val word = Cat(Mux(t.word, Fill(32, sign && wordShift(31)), dat(63,32)), wordShift)
  val halfShift = Mux(addr(1), word(31,16), word(15,0))
  val half = Cat(Mux(t.half, Fill(48, sign && halfShift(15)), word(63,16)), halfShift)
  val byteShift = Mux(addr(0), half(15,8), half(7,0))
  val byte = Cat(Mux(t.byte, Fill(56, sign && byteShift(7)), half(63,8)), byteShift)
}

class MSHRReq(implicit conf: DCacheConfig) extends HellaCacheReq {
  val tag_match = Bool()
  val old_meta = new MetaData
  val way_en = Bits(width = conf.ways)

  override def clone = new MSHRReq().asInstanceOf[this.type]
}

class Replay(implicit conf: DCacheConfig) extends HellaCacheReq {
  val sdq_id = UFix(width = log2Up(conf.nsdq))

  override def clone = new Replay().asInstanceOf[this.type]
}

class DataReadReq(implicit conf: DCacheConfig) extends Bundle {
  val way_en = Bits(width = conf.ways)
  val addr   = Bits(width = conf.untagbits)

  override def clone = new DataReadReq().asInstanceOf[this.type]
}

class DataWriteReq(implicit conf: DCacheConfig) extends Bundle {
  val way_en = Bits(width = conf.ways)
  val addr   = Bits(width = conf.untagbits)
  val wmask  = Bits(width = MEM_DATA_BITS/conf.databits)
  val data   = Bits(width = MEM_DATA_BITS)

  override def clone = new DataWriteReq().asInstanceOf[this.type]
}

class WritebackReq(implicit conf: DCacheConfig) extends Bundle {
  val tag = Bits(width = conf.tagbits)
  val idx = Bits(width = conf.idxbits)
  val way_en = Bits(width = conf.ways)
  val tile_xact_id = Bits(width = TILE_XACT_ID_BITS)

  override def clone = new WritebackReq().asInstanceOf[this.type]
}

class MetaData(implicit conf: DCacheConfig) extends Bundle {
  val state = UFix(width = 2)
  val tag = Bits(width = conf.tagbits)

  override def clone = new MetaData().asInstanceOf[this.type]
}

class MetaReadReq(implicit conf: DCacheConfig) extends Bundle {
  val addr  = UFix(width = conf.paddrbits)

  override def clone = new MetaReadReq().asInstanceOf[this.type]
}

class MetaWriteReq(implicit conf: DCacheConfig) extends Bundle {
  val way_en = Bits(width = conf.ways)
  val idx  = Bits(width = conf.idxbits)
  val data = new MetaData()

  override def clone = new MetaWriteReq().asInstanceOf[this.type]
}

class MSHR(id: Int)(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val req_pri_val    = Bool(INPUT)
    val req_pri_rdy    = Bool(OUTPUT)
    val req_sec_val    = Bool(INPUT)
    val req_sec_rdy    = Bool(OUTPUT)
    val req_bits       = new MSHRReq().asInput
    val req_sdq_id     = UFix(INPUT, log2Up(conf.nsdq))

    val idx_match      = Bool(OUTPUT)
    val tag            = Bits(OUTPUT, conf.tagbits)

    val mem_req  = (new FIFOIO) { new TransactionInit }
    val mem_resp = new DataWriteReq().asOutput
    val meta_read = (new FIFOIO) { new MetaReadReq }
    val meta_write = (new FIFOIO) { new MetaWriteReq }
    val replay = (new FIFOIO) { new Replay() }
    val mem_abort = (new PipeIO) { new TransactionAbort }.flip
    val mem_rep = (new PipeIO) { new TransactionReply }.flip
    val mem_finish = (new FIFOIO) { new TransactionFinish }
    val wb_req = (new FIFOIO) { new WritebackReq }
    val probe_writeback = (new FIFOIO) { Bool() }.flip
    val probe_refill = (new FIFOIO) { Bool() }.flip
  }

  val s_invalid :: s_wb_req :: s_wb_resp :: s_meta_clear :: s_refill_req :: s_refill_resp :: s_meta_write :: s_drain_rpq :: Nil = Enum(8) { UFix() }
  val state = Reg(resetVal = s_invalid)

  val xacx_type = Reg { UFix() }
  val line_state = Reg { UFix() }
  val refill_count = Reg { UFix(width = log2Up(REFILL_CYCLES)) }
  val req = Reg { new MSHRReq() }

  val req_cmd = io.req_bits.cmd
  val req_idx = req.addr(conf.untagbits-1,conf.offbits)
  val idx_match = req_idx === io.req_bits.addr(conf.untagbits-1,conf.offbits)
  val sec_rdy = idx_match && (state === s_wb_req || state === s_wb_resp || state === s_meta_clear || (state === s_refill_req || state === s_refill_resp) && !conf.co.needsTransactionOnSecondaryMiss(req_cmd, io.mem_req.bits))

  val rpq = (new Queue(conf.nrpq)) { new Replay }
  rpq.io.enq.valid := (io.req_pri_val && io.req_pri_rdy || io.req_sec_val && sec_rdy) && !isPrefetch(req_cmd)
  rpq.io.enq.bits := io.req_bits
  rpq.io.enq.bits.sdq_id := io.req_sdq_id
  rpq.io.deq.ready := io.replay.ready && state === s_drain_rpq || state === s_invalid

  val abort = io.mem_abort.valid && io.mem_abort.bits.tile_xact_id === UFix(id)
  val reply = io.mem_rep.valid && io.mem_rep.bits.tile_xact_id === UFix(id)
  val refill_done = reply && refill_count.andR
  val wb_done = reply && (state === s_wb_resp)

  val finish_q = (new Queue(2 /* wb + refill */)) { new TransactionFinish }
  finish_q.io.enq.valid := wb_done || refill_done
  finish_q.io.enq.bits.global_xact_id := io.mem_rep.bits.global_xact_id
  io.wb_req.valid := Bool(false)

  when (state === s_drain_rpq && !rpq.io.deq.valid && !finish_q.io.deq.valid) {
    state := s_invalid
  }
  when (state === s_meta_write && io.meta_write.ready) {
    state := s_drain_rpq
  }
  when (state === s_refill_resp) {
    when (refill_done) { state := s_meta_write }
    when (reply) {
      refill_count := refill_count + UFix(1)
      line_state := conf.co.newStateOnTransactionReply(io.mem_rep.bits, io.mem_req.bits)
    }
    when (abort) { state := s_refill_req }
  }
  when (state === s_refill_req) {
    when (abort) { state := s_refill_req }
    .elsewhen (io.mem_req.ready) { state := s_refill_resp }
  }
  when (state === s_meta_clear && io.meta_write.ready) {
    state := s_refill_req
  }
  when (state === s_wb_resp) {
    when (reply) { state := s_meta_clear }
    when (abort) { state := s_wb_req }
  }
  when (state === s_wb_req) {
    io.wb_req.valid := Bool(true)
    when (io.probe_writeback.valid && idx_match) {
      io.wb_req.valid := Bool(false)
      when (io.probe_writeback.bits) { state := s_refill_req }
    }.elsewhen (io.wb_req.ready) { state := s_wb_resp }
  }

  when (io.req_sec_val && io.req_sec_rdy) { // s_wb_req, s_wb_resp, s_refill_req
    xacx_type := conf.co.getTransactionInitTypeOnSecondaryMiss(req_cmd, conf.co.newStateOnFlush(), io.mem_req.bits)
  }
  when ((state === s_invalid) && io.req_pri_val) {
    line_state := conf.co.newStateOnFlush()
    refill_count := UFix(0)
    xacx_type := conf.co.getTransactionInitTypeOnPrimaryMiss(req_cmd, conf.co.newStateOnFlush())
    req := io.req_bits

    state := Mux(conf.co.needsWriteback(io.req_bits.old_meta.state), s_wb_req, s_refill_req)
    when (io.req_bits.tag_match) {
      when (conf.co.isHit(req_cmd, io.req_bits.old_meta.state)) { // set dirty bit
        state := s_meta_write
        line_state := conf.co.newStateOnHit(req_cmd, io.req_bits.old_meta.state)
      }.otherwise { // upgrade permissions
        state := s_refill_req
      }
    }
  }

  io.idx_match := (state != s_invalid) && idx_match
  io.mem_resp := req
  io.mem_resp.addr := Cat(req_idx, refill_count) << conf.ramoffbits
  io.tag := req.addr >> conf.untagbits
  io.req_pri_rdy := (state === s_invalid)
  io.req_sec_rdy := sec_rdy && rpq.io.enq.ready

  io.meta_write.valid := state === s_meta_write || state === s_meta_clear
  io.meta_write.bits.idx := req_idx
  io.meta_write.bits.data.state := Mux(state === s_meta_clear, conf.co.newStateOnFlush(), line_state)
  io.meta_write.bits.data.tag := io.tag
  io.meta_write.bits.way_en := req.way_en

  io.wb_req.bits.tag := req.old_meta.tag
  io.wb_req.bits.idx := req_idx
  io.wb_req.bits.way_en := req.way_en
  io.wb_req.bits.tile_xact_id := Bits(id)

  io.probe_writeback.ready := (state != s_wb_resp && state != s_meta_clear && state != s_drain_rpq) || !idx_match
  io.probe_refill.ready := (state != s_refill_resp && state != s_drain_rpq) || !idx_match

  io.mem_req.valid := state === s_refill_req
  io.mem_req.bits.x_type := xacx_type
  io.mem_req.bits.addr := Cat(io.tag, req_idx).toUFix
  io.mem_req.bits.tile_xact_id := Bits(id)
  io.mem_finish <> finish_q.io.deq

  io.meta_read.valid := state === s_drain_rpq
  io.meta_read.bits.addr := io.mem_req.bits.addr << conf.offbits

  io.replay.valid := state === s_drain_rpq && rpq.io.deq.valid
  io.replay.bits := rpq.io.deq.bits
  io.replay.bits.phys := Bool(true)
  io.replay.bits.addr := Cat(io.tag, req_idx, rpq.io.deq.bits.addr(conf.offbits-1,0)).toUFix

  when (!io.meta_read.ready) {
    rpq.io.deq.ready := Bool(false)
    io.replay.bits.cmd := M_FENCE // NOP
  }
}

class MSHRFile(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val req = (new FIFOIO) { new MSHRReq }.flip
    val secondary_miss = Bool(OUTPUT)

    val mem_req  = (new FIFOIO) { new TransactionInit }
    val mem_resp = new DataWriteReq().asOutput
    val meta_read = (new FIFOIO) { new MetaReadReq }
    val meta_write = (new FIFOIO) { new MetaWriteReq }
    val replay = (new FIFOIO) { new Replay }
    val mem_abort = (new PipeIO) { new TransactionAbort }.flip
    val mem_rep = (new PipeIO) { new TransactionReply }.flip
    val mem_finish = (new FIFOIO) { new TransactionFinish }
    val wb_req = (new FIFOIO) { new WritebackReq }
    val probe = (new FIFOIO) { Bool() }.flip

    val fence_rdy = Bool(OUTPUT)
  }

  val sdq_val = Reg(resetVal = Bits(0, conf.nsdq))
  val sdq_alloc_id = PriorityEncoder(~sdq_val(conf.nsdq-1,0))
  val sdq_rdy = !sdq_val.andR
  val sdq_enq = io.req.valid && io.req.ready && isWrite(io.req.bits.cmd)
  val sdq = Mem(conf.nsdq) { io.req.bits.data.clone }
  when (sdq_enq) { sdq(sdq_alloc_id) := io.req.bits.data }

  val idxMatch = Vec(conf.nmshr) { Bool() }
  val tagList = Vec(conf.nmshr) { Bits() }
  val wbTagList = Vec(conf.nmshr) { Bits() }
  val memRespMux = Vec(conf.nmshr) { new DataWriteReq }
  val meta_read_arb = (new Arbiter(conf.nmshr)) { new MetaReadReq }
  val meta_write_arb = (new Arbiter(conf.nmshr)) { new MetaWriteReq }
  val mem_req_arb = (new Arbiter(conf.nmshr)) { new TransactionInit }
  val mem_finish_arb = (new Arbiter(conf.nmshr)) { new TransactionFinish }
  val wb_req_arb = (new Arbiter(conf.nmshr)) { new WritebackReq }
  val replay_arb = (new Arbiter(conf.nmshr)) { new Replay() }
  val alloc_arb = (new Arbiter(conf.nmshr)) { Bool() }

  val tag_match = Mux1H(idxMatch, tagList) === io.req.bits.addr >> conf.untagbits
  val wb_probe_match = Mux1H(idxMatch, wbTagList) === io.req.bits.addr >> conf.untagbits

  var idx_match = Bool(false)
  var pri_rdy = Bool(false)
  var fence = Bool(false)
  var sec_rdy = Bool(false)
  var writeback_probe_rdy = Bool(true)
  var refill_probe_rdy = Bool(true)

  for (i <- 0 to conf.nmshr-1) {
    val mshr = new MSHR(i)

    idxMatch(i) := mshr.io.idx_match
    tagList(i) := mshr.io.tag
    wbTagList(i) := mshr.io.wb_req.bits.tag

    alloc_arb.io.in(i).valid := mshr.io.req_pri_rdy
    mshr.io.req_pri_val := alloc_arb.io.in(i).ready

    mshr.io.req_sec_val := io.req.valid && sdq_rdy && tag_match
    mshr.io.req_bits := io.req.bits
    mshr.io.req_sdq_id := sdq_alloc_id

    mshr.io.meta_read <> meta_read_arb.io.in(i)
    mshr.io.meta_write <> meta_write_arb.io.in(i)
    mshr.io.mem_req <> mem_req_arb.io.in(i)
    mshr.io.mem_finish <> mem_finish_arb.io.in(i)
    mshr.io.wb_req <> wb_req_arb.io.in(i)
    mshr.io.replay <> replay_arb.io.in(i)
    mshr.io.probe_refill.valid := io.probe.valid && tag_match
    mshr.io.probe_writeback.valid := io.probe.valid
    mshr.io.probe_writeback.bits := wb_probe_match

    mshr.io.mem_abort <> io.mem_abort
    mshr.io.mem_rep <> io.mem_rep
    memRespMux(i) := mshr.io.mem_resp

    pri_rdy = pri_rdy || mshr.io.req_pri_rdy
    sec_rdy = sec_rdy || mshr.io.req_sec_rdy
    fence = fence || !mshr.io.req_pri_rdy
    idx_match = idx_match || mshr.io.idx_match
    refill_probe_rdy = refill_probe_rdy && mshr.io.probe_refill.ready
    writeback_probe_rdy = writeback_probe_rdy && mshr.io.probe_writeback.ready
  }

  alloc_arb.io.out.ready := io.req.valid && sdq_rdy && !idx_match

  meta_read_arb.io.out <> io.meta_read
  meta_write_arb.io.out <> io.meta_write
  mem_req_arb.io.out <> io.mem_req
  mem_finish_arb.io.out <> io.mem_finish
  wb_req_arb.io.out <> io.wb_req

  io.req.ready := Mux(idx_match, tag_match && sec_rdy, pri_rdy) && sdq_rdy
  io.secondary_miss := idx_match
  io.mem_resp := memRespMux(io.mem_rep.bits.tile_xact_id)
  io.fence_rdy := !fence
  io.probe.ready := (refill_probe_rdy || !tag_match) && (writeback_probe_rdy || !wb_probe_match)

  val free_sdq = io.replay.fire() && isWrite(io.replay.bits.cmd)
  io.replay.bits.data := sdq(RegEn(replay_arb.io.out.bits.sdq_id, free_sdq))
  io.replay <> replay_arb.io.out

  when (io.replay.valid || sdq_enq) {
    sdq_val := sdq_val & ~(UFixToOH(io.replay.bits.sdq_id) & Fill(conf.nsdq, free_sdq)) | 
               PriorityEncoderOH(~sdq_val(conf.nsdq-1,0)) & Fill(conf.nsdq, sdq_enq)
  }
}


class WritebackUnit(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val req = (new FIFOIO) { new WritebackReq() }.flip
    val probe = (new FIFOIO) { new WritebackReq() }.flip
    val meta_read = (new FIFOIO) { new MetaReadReq }
    val data_req = (new FIFOIO) { new DataReadReq() }
    val data_resp = Bits(INPUT, MEM_DATA_BITS)
    val mem_req = (new FIFOIO) { new TransactionInit }
    val mem_req_data = (new FIFOIO) { new TransactionInitData }
    val probe_rep_data = (new FIFOIO) { new ProbeReplyData }
  }

  val valid = Reg(resetVal = Bool(false))
  val is_probe = Reg{Bool()}
  val r1_data_req_fired = Reg(resetVal = Bool(false))
  val r2_data_req_fired = Reg(resetVal = Bool(false))
  val cmd_sent = Reg{Bool()}
  val cnt = Reg{UFix(width = log2Up(REFILL_CYCLES+1))}
  val req = Reg{new WritebackReq}

  when (valid) {
    r1_data_req_fired := false
    r2_data_req_fired := r1_data_req_fired
    when (io.data_req.fire()) {
      r1_data_req_fired := true
      cnt := cnt + 1
    }

    when (r2_data_req_fired && !Mux(is_probe, io.probe_rep_data.ready, io.mem_req_data.ready)) {
      r1_data_req_fired := false
      r2_data_req_fired := false
      cnt := cnt - Mux[UFix](r1_data_req_fired, 2, 1)
    }

    when (!r1_data_req_fired && !r2_data_req_fired && cmd_sent && cnt === REFILL_CYCLES) {
      valid := false
    }

    when (valid && io.mem_req.ready) {
      cmd_sent := true
    }
  }
  when (io.probe.fire()) {
    valid := true
    is_probe := true
    cmd_sent := true
    cnt := 0
    req := io.probe.bits
  }
  when (io.req.fire()) {
    valid := true
    is_probe := false
    cmd_sent := false
    cnt := 0
    req := io.req.bits
  }

  val fire = valid && cnt < UFix(REFILL_CYCLES)
  io.req.ready := !valid && !io.probe.valid
  io.probe.ready := !valid
  io.data_req.valid := fire && io.meta_read.ready
  io.data_req.bits.way_en := req.way_en
  io.data_req.bits.addr := Cat(req.idx, cnt(log2Up(REFILL_CYCLES)-1,0)) << conf.ramoffbits

  io.mem_req.valid := valid && !cmd_sent
  io.mem_req.bits.x_type := conf.co.getTransactionInitTypeOnWriteback()
  io.mem_req.bits.addr := Cat(req.tag, req.idx).toUFix
  io.mem_req.bits.tile_xact_id := req.tile_xact_id
  io.mem_req_data.valid := r2_data_req_fired && !is_probe
  io.mem_req_data.bits.data := io.data_resp
  io.probe_rep_data.valid := r2_data_req_fired && is_probe
  io.probe_rep_data.bits.data := io.data_resp

  io.meta_read.valid := fire && io.data_req.ready
  io.meta_read.bits.addr := io.mem_req.bits.addr << conf.offbits
}

class ProbeUnit(implicit conf: DCacheConfig)  extends Component {
  val io = new Bundle {
    val req = (new FIFOIO) { new ProbeRequest }.flip
    val rep = (new FIFOIO) { new ProbeReply }
    val meta_read = (new FIFOIO) { new MetaReadReq }
    val meta_write = (new FIFOIO) { new MetaWriteReq }
    val mshr_req = (new FIFOIO) { Bool() }
    val wb_req = (new FIFOIO) { new WritebackReq }
    val way_en = Bits(INPUT, conf.ways)
    val line_state = UFix(INPUT, 2)
  }

  val s_invalid :: s_meta_read :: s_meta_resp :: s_mshr_req :: s_probe_rep :: s_writeback_req :: s_writeback_resp :: s_meta_write :: Nil = Enum(8) { UFix() }
  val state = Reg(resetVal = s_invalid)
  val line_state = Reg() { UFix() }
  val way_en = Reg() { Bits() }
  val req = Reg() { new ProbeRequest() }
  val hit = way_en.orR

  when (state === s_meta_write && io.meta_write.ready) {
    state := s_invalid
  }
  when (state === s_writeback_resp && io.wb_req.ready) {
    state := s_meta_write
  }
  when (state === s_writeback_req && io.wb_req.ready) {
    state := s_writeback_resp
  }
  when (state === s_probe_rep && io.rep.ready) {
    state := s_invalid
    when (hit) {
      state := Mux(conf.co.needsWriteback(line_state), s_writeback_req, s_meta_write)
    }
  }
  when (state === s_mshr_req) {
    state := s_probe_rep
    line_state := io.line_state
    way_en := io.way_en
    when (!io.mshr_req.ready) { state := s_meta_read }
  }
  when (state === s_meta_resp) {
    state := s_mshr_req
  }
  when (state === s_meta_read && io.meta_read.ready) {
    state := s_meta_resp
  }
  when (state === s_invalid && io.req.valid) {
    state := s_meta_read
    req := io.req.bits
  }

  io.req.ready := state === s_invalid && !reset
  io.rep.valid := state === s_probe_rep
  io.rep.bits := conf.co.newProbeReply(req, Mux(hit, line_state, conf.co.newStateOnFlush))

  io.meta_read.valid := state === s_meta_read
  io.meta_read.bits.addr := req.addr << UFix(conf.offbits)

  io.meta_write.valid := state === s_meta_write
  io.meta_write.bits.way_en := way_en
  io.meta_write.bits.idx := req.addr
  io.meta_write.bits.data.state := conf.co.newStateOnProbeRequest(req, line_state)
  io.meta_write.bits.data.tag := req.addr >> UFix(conf.idxbits)

  io.mshr_req.valid := state === s_mshr_req
  io.wb_req.valid := state === s_writeback_req
  io.wb_req.bits.way_en := way_en
  io.wb_req.bits.idx := req.addr
  io.wb_req.bits.tag := req.addr >> UFix(conf.idxbits)
}

class MetaDataArray(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val read = (new FIFOIO) { new MetaReadReq }.flip
    val write = (new FIFOIO) { new MetaWriteReq }.flip
    val resp = Vec(conf.ways){ (new MetaData).asOutput }
  }

  val rst_cnt = Reg(resetVal = UFix(0, log2Up(conf.sets+1)))
  val rst = rst_cnt < conf.sets
  when (rst) { rst_cnt := rst_cnt+1 }

  val metabits = io.write.bits.data.state.width + conf.tagbits
  val tags = Mem(conf.sets, seqRead = true) { UFix(width = metabits*conf.ways) }
  val tag = Reg{UFix()}

  when (io.read.valid) {
    tag := tags(io.read.bits.addr(conf.untagbits-1,conf.offbits))
  }
  when (rst || io.write.valid) {
    val addr = Mux(rst, rst_cnt, io.write.bits.idx)
    val data = Cat(Mux(rst, conf.co.newStateOnFlush, io.write.bits.data.state), io.write.bits.data.tag)
    val mask = Mux(rst, Fix(-1), io.write.bits.way_en)
    tags.write(addr, Fill(conf.ways, data), FillInterleaved(metabits, mask))
  }

  for (w <- 0 until conf.ways) {
    val m = tag(metabits*(w+1)-1, metabits*w)
    io.resp(w).state := m >> conf.tagbits
    io.resp(w).tag := m
  }

  io.read.ready := Bool(true)
  io.write.ready := !rst
}

class DataArray(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val read = new FIFOIO()(new DataReadReq).flip
    val write = new FIFOIO()(new DataWriteReq).flip
    val resp = Vec(conf.ways){ Bits(OUTPUT, MEM_DATA_BITS) }
  }

  val waddr = io.write.bits.addr >> conf.ramoffbits
  val raddr = io.read.bits.addr >> conf.ramoffbits

  if (conf.isNarrowRead) {
    val waysPerMem = MEM_DATA_BITS/conf.databits
    for (w <- 0 until conf.ways by waysPerMem) {
      val wway_en = io.write.bits.way_en(w+waysPerMem-1,w)
      val rway_en = io.read.bits.way_en(w+waysPerMem-1,w)
      val resp = Vec(MEM_DATA_BITS/conf.databits){Reg{Bits(width = MEM_DATA_BITS)}}
      val r_raddr = RegEn(io.read.bits.addr, io.read.valid)
      for (p <- 0 until resp.size) {
        val array = Mem(conf.sets*REFILL_CYCLES, seqRead = true){ Bits(width=MEM_DATA_BITS) }
        when (wway_en.orR && io.write.valid && io.write.bits.wmask(p)) {
          val data = Fill(waysPerMem, io.write.bits.data(conf.databits*(p+1)-1,conf.databits*p))
          val mask = FillInterleaved(conf.databits, wway_en)
          array.write(waddr, data, mask)
        }
        when (rway_en.orR && io.read.valid) {
          resp(p) := array(raddr)
        }
      }
      for (dw <- 0 until waysPerMem) {
        val r = AVec(resp.map(_(conf.databits*(dw+1)-1,conf.databits*dw)))
        val resp_mux =
          if (r.size == 1) r
          else AVec(r(r_raddr(conf.ramoffbits-1,conf.wordoffbits)), r.tail:_*)
        io.resp(w+dw) := resp_mux.toBits
      }
    }
  } else {
    val wmask = FillInterleaved(conf.databits, io.write.bits.wmask)
    for (w <- 0 until conf.ways) {
      val rdata = Reg() { Bits() }
      val array = Mem(conf.sets*REFILL_CYCLES, seqRead = true){ Bits(width=MEM_DATA_BITS) }
      when (io.write.bits.way_en(w) && io.write.valid) {
        array.write(waddr, io.write.bits.data, wmask)
      }
      when (io.read.bits.way_en(w) && io.read.valid) {
        rdata := array(raddr)
      }
      io.resp(w) := rdata
    }
  }

  io.read.ready := Bool(true)
  io.write.ready := Bool(true)
}

class AMOALU(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val addr = Bits(INPUT, conf.offbits)
    val cmd = Bits(INPUT, 4)
    val typ = Bits(INPUT, 3)
    val lhs = Bits(INPUT, conf.databits)
    val rhs = Bits(INPUT, conf.databits)
    val out = Bits(OUTPUT, conf.databits)
  }

  require(conf.databits == 64)
  
  val sgned = io.cmd === M_XA_MIN || io.cmd === M_XA_MAX
  val max = io.cmd === M_XA_MAX || io.cmd === M_XA_MAXU
  val min = io.cmd === M_XA_MIN || io.cmd === M_XA_MINU
  val word = io.typ === MT_W || io.typ === MT_WU || io.typ === MT_B || io.typ === MT_BU

  val mask = Fix(-1,64) ^ (io.addr(2) << 31)
  val adder_out = (io.lhs & mask) + (io.rhs & mask)

  val cmp_lhs  = Mux(word && !io.addr(2), io.lhs(31), io.lhs(63))
  val cmp_rhs  = Mux(word && !io.addr(2), io.rhs(31), io.rhs(63))
  val lt_lo = io.lhs(31,0) < io.rhs(31,0)
  val lt_hi = io.lhs(63,32) < io.rhs(63,32)
  val eq_hi = io.lhs(63,32) === io.rhs(63,32)
  val lt = Mux(word, Mux(io.addr(2), lt_hi, lt_lo), lt_hi || eq_hi && lt_lo)
  val less = Mux(cmp_lhs === cmp_rhs, lt, Mux(sgned, cmp_lhs, cmp_rhs))

  val out = Mux(io.cmd === M_XA_ADD, adder_out,
            Mux(io.cmd === M_XA_AND, io.lhs & io.rhs,
            Mux(io.cmd === M_XA_OR,  io.lhs | io.rhs,
            Mux(Mux(less, min, max), io.lhs,
            io.rhs))))

  val wmask = FillInterleaved(8, StoreGen(io.typ, io.addr).mask)
  io.out := wmask & out | ~wmask & io.lhs
}

class HellaCacheReq(implicit conf: DCacheConfig) extends Bundle {
  val kill = Bool()
  val typ  = Bits(width = 3)
  val phys = Bool()
  val addr = UFix(width = conf.ppnbits.max(conf.vpnbits+1) + conf.pgidxbits)
  val data = Bits(width = conf.databits)
  val tag  = Bits(width = conf.reqtagbits)
  val cmd  = Bits(width = 4)

  override def clone = new HellaCacheReq().asInstanceOf[this.type]
}

class HellaCacheResp(implicit conf: DCacheConfig) extends Bundle {
  val nack = Bool() // comes 2 cycles after req.fire
  val replay = Bool()
  val typ = Bits(width = 3)
  val data = Bits(width = conf.databits)
  val data_subword = Bits(width = conf.databits)
  val tag = Bits(width = conf.reqtagbits)
  val cmd  = Bits(width = 4)
  val addr = UFix(width = conf.ppnbits.max(conf.vpnbits+1) + conf.pgidxbits)
  val store_data = Bits(width = conf.databits)

  override def clone = new HellaCacheResp().asInstanceOf[this.type]
}

class AlignmentExceptions extends Bundle {
  val ld = Bool()
  val st = Bool()
}

class HellaCacheExceptions extends Bundle {
  val ma = new AlignmentExceptions
  val pf = new AlignmentExceptions
}

// interface between D$ and processor/DTLB
class ioHellaCache(implicit conf: DCacheConfig) extends Bundle {
  val req = (new FIFOIO){ new HellaCacheReq }
  val resp = (new PipeIO){ new HellaCacheResp }.flip
  val xcpt = (new HellaCacheExceptions).asInput
  val ptw = new IOTLBPTW().flip
}

class HellaCache(implicit conf: DCacheConfig) extends Component {
  val io = new Bundle {
    val cpu = (new ioHellaCache).flip
    val mem = new ioTileLink
  }
 
  val indexmsb    = conf.untagbits-1
  val indexlsb    = conf.offbits
  val offsetmsb   = indexlsb-1
  val offsetlsb   = log2Up(conf.databytes)

  val wb = new WritebackUnit
  val prober = new ProbeUnit
  val mshr = new MSHRFile

  io.cpu.req.ready := Bool(true)
  val s1_valid = Reg(io.cpu.req.fire(), resetVal = Bool(false))
  val s1_req = Reg{io.cpu.req.bits.clone}
  val s1_valid_masked = s1_valid && !io.cpu.req.bits.kill
  val s1_replay = Reg(resetVal = Bool(false))
  val s1_clk_en = Reg{Bool()}

  val s2_valid = Reg(s1_valid_masked, resetVal = Bool(false))
  val s2_req = Reg{io.cpu.req.bits.clone}
  val s2_replay = Reg(s1_replay, resetVal = Bool(false))
  val s2_valid_masked = Bool()

  val s3_valid = Reg(resetVal = Bool(false))
  val s3_req = Reg{io.cpu.req.bits.clone}
  val s3_way = Reg{Bits()}

  val s1_read  = isRead(s1_req.cmd)
  val s1_write = isWrite(s1_req.cmd)
  val s1_readwrite = s1_read || s1_write || isPrefetch(s1_req.cmd)

  val dtlb = new TLB(8)
  dtlb.io.ptw <> io.cpu.ptw
  dtlb.io.req.valid := s1_valid_masked && s1_readwrite && !s1_req.phys
  dtlb.io.req.bits.passthrough := s1_req.phys
  dtlb.io.req.bits.asid := UFix(0)
  dtlb.io.req.bits.vpn := s1_req.addr >> conf.pgidxbits
  dtlb.io.req.bits.instruction := Bool(false)
  when (!dtlb.io.req.ready && !io.cpu.req.bits.phys) { io.cpu.req.ready := Bool(false) }
  
  when (io.cpu.req.valid) {
    s1_req := io.cpu.req.bits
  }
  when (wb.io.meta_read.valid) {
    s1_req := wb.io.meta_read.bits
    s1_req.phys := Bool(true)
  }
  when (prober.io.meta_read.valid) {
    s1_req := prober.io.meta_read.bits
    s1_req.phys := Bool(true)
  }
  when (mshr.io.replay.valid) {
    s1_req := mshr.io.replay.bits
  }
  val s1_addr = Cat(dtlb.io.resp.ppn, s1_req.addr(conf.pgidxbits-1,0))

  when (s1_clk_en) {
    s2_req.addr := s1_addr
    s2_req.typ := s1_req.typ
    s2_req.cmd := s1_req.cmd
    s2_req.tag := s1_req.tag
    when (s1_write) {
      s2_req.data := Mux(s1_replay, mshr.io.replay.bits.data, io.cpu.req.bits.data)
    }
  }

  val misaligned =
    (((s1_req.typ === MT_H) || (s1_req.typ === MT_HU)) && (s1_req.addr(0) != Bits(0))) ||
    (((s1_req.typ === MT_W) || (s1_req.typ === MT_WU)) && (s1_req.addr(1,0) != Bits(0))) ||
    ((s1_req.typ === MT_D) && (s1_req.addr(2,0) != Bits(0)));
    
  io.cpu.xcpt.ma.ld := s1_read && misaligned
  io.cpu.xcpt.ma.st := s1_write && misaligned
  io.cpu.xcpt.pf.ld := s1_read && dtlb.io.resp.xcpt_ld
  io.cpu.xcpt.pf.st := s1_write && dtlb.io.resp.xcpt_st

  // tags
  val meta = new MetaDataArray
  val metaReadArb = (new Arbiter(4)) { new MetaReadReq }
  val metaWriteArb = (new Arbiter(2)) { new MetaWriteReq }
  metaReadArb.io.out <> meta.io.read
  metaWriteArb.io.out <> meta.io.write

  // data
  val data = new DataArray
  val readArb = new Arbiter(3)(new DataReadReq)
  val writeArb = new Arbiter(2)(new DataWriteReq)
  readArb.io.out.ready := !io.mem.xact_rep.valid || io.mem.xact_rep.ready // insert bubble if refill gets blocked
  readArb.io.out <> data.io.read
  writeArb.io.out <> data.io.write

  // tag read for new requests
  metaReadArb.io.in(3).valid := io.cpu.req.valid
  metaReadArb.io.in(3).bits.addr := io.cpu.req.bits.addr
  when (!metaReadArb.io.in(3).ready) { io.cpu.req.ready := Bool(false) }

  // data read for new requests
  readArb.io.in(2).bits.addr := io.cpu.req.bits.addr
  readArb.io.in(2).valid := io.cpu.req.valid
  readArb.io.in(2).bits.way_en := Fix(-1)
  when (!readArb.io.in(2).ready) { io.cpu.req.ready := Bool(false) }

  // tag check and way muxing
  def wayMap[T <: Data](f: Int => T)(gen: => T) = Vec((0 until conf.ways).map(i => f(i))){gen}
  val s1_tag_eq_way = wayMap((w: Int) => meta.io.resp(w).tag === (s1_addr >> conf.untagbits)){Bits()}.toBits
  val s1_tag_match_way = wayMap((w: Int) => s1_tag_eq_way(w) && conf.co.isValid(meta.io.resp(w).state)){Bits()}.toBits
  s1_clk_en := metaReadArb.io.out.valid
  val s1_writeback = s1_clk_en && !s1_valid && !s1_replay
  val s2_tag_match_way = RegEn(s1_tag_match_way, s1_clk_en)
  val s2_tag_match = s2_tag_match_way.orR
  val s2_hit_state = Mux1H(s2_tag_match_way, wayMap((w: Int) => RegEn(meta.io.resp(w).state, s1_clk_en)){Bits()})
  val s2_hit = s2_tag_match && conf.co.isHit(s2_req.cmd, s2_hit_state) && s2_hit_state === conf.co.newStateOnHit(s2_req.cmd, s2_hit_state)

  val s2_data = Vec(conf.ways){Bits(width = MEM_DATA_BITS)}
  for (w <- 0 until conf.ways) {
    val regs = Vec(MEM_DATA_BITS/conf.databits){Reg{Bits(width = conf.databits)}}
    val en1 = s1_clk_en && s1_tag_eq_way(w)
    for (i <- 0 until regs.size) {
      val en = en1 && (Bool(i == 0 || !conf.isNarrowRead) || s1_writeback)
      when (en) { regs(i) := data.io.resp(w) >> conf.databits*i }
    }
    s2_data(w) := Cat(regs.last, regs.init.reverse:_*)
  }
  val data_resp_mux = Mux1H(s2_tag_match_way, s2_data)
  
  // store/amo hits
  s3_valid := (s2_valid_masked && s2_hit || s2_replay) && isWrite(s2_req.cmd)
  val amoalu = new AMOALU
  when ((s2_valid || s2_replay) && isWrite(s2_req.cmd)) {
    s3_req := s2_req
    s3_req.data := amoalu.io.out
    s3_way := s2_tag_match_way
  }

  writeArb.io.in(0).bits.addr := s3_req.addr
  writeArb.io.in(0).bits.wmask := UFix(1) << s3_req.addr(conf.ramoffbits-1,offsetlsb).toUFix
  writeArb.io.in(0).bits.data := Fill(MEM_DATA_BITS/conf.databits, s3_req.data)
  writeArb.io.in(0).valid := s3_valid
  writeArb.io.in(0).bits.way_en :=  s3_way

  // replacement policy
  val replacer = new RandomReplacement
  val s1_replaced_way_en = UFixToOH(replacer.way)
  val s2_replaced_way_en = UFixToOH(RegEn(replacer.way, s1_clk_en))
  val s2_repl_state = Mux1H(s2_replaced_way_en, wayMap((w: Int) => RegEn(meta.io.resp(w).state, s1_clk_en && s1_replaced_way_en(w))){Bits()})
  val s2_repl_tag = Mux1H(s2_replaced_way_en, wayMap((w: Int) => RegEn(meta.io.resp(w).tag, s1_clk_en && s1_replaced_way_en(w))){Bits()})

  // miss handling
  mshr.io.req.valid := s2_valid_masked && !s2_hit && (isPrefetch(s2_req.cmd) || isRead(s2_req.cmd) || isWrite(s2_req.cmd))
  mshr.io.req.bits := s2_req
  mshr.io.req.bits.tag_match := s2_tag_match
  mshr.io.req.bits.old_meta.state := s2_repl_state
  mshr.io.req.bits.old_meta.tag := s2_repl_tag
  mshr.io.req.bits.way_en := Mux(s2_tag_match, s2_tag_match_way, s2_replaced_way_en)
  mshr.io.req.bits.data := s2_req.data

  mshr.io.mem_rep.valid := io.mem.xact_rep.fire()
  mshr.io.mem_rep.bits := io.mem.xact_rep.bits
  mshr.io.mem_abort.valid := io.mem.xact_abort.valid
  mshr.io.mem_abort.bits := io.mem.xact_abort.bits
  io.mem.xact_abort.ready := Bool(true)
  when (mshr.io.req.fire()) { replacer.miss }

  // replays
  readArb.io.in(0).valid := mshr.io.replay.valid
  readArb.io.in(0).bits := mshr.io.replay.bits
  readArb.io.in(0).bits.way_en := Fix(-1)
  mshr.io.replay.ready := Bool(true)
  s1_replay := mshr.io.replay.fire()
  metaReadArb.io.in(0) <> mshr.io.meta_read
  metaWriteArb.io.in(0) <> mshr.io.meta_write

  // probes
  prober.io.req <> io.mem.probe_req
  prober.io.rep <> io.mem.probe_rep
  prober.io.mshr_req <> mshr.io.probe
  prober.io.wb_req <> wb.io.probe
  prober.io.way_en := s2_tag_match_way
  prober.io.line_state := s2_hit_state
  prober.io.meta_read <> metaReadArb.io.in(1)
  prober.io.meta_write <> metaWriteArb.io.in(1)

  // refills
  val refill = conf.co.messageUpdatesDataArray(io.mem.xact_rep.bits)
  writeArb.io.in(1).valid := io.mem.xact_rep.valid && refill
  io.mem.xact_rep.ready := writeArb.io.in(1).ready || !refill
  writeArb.io.in(1).bits := mshr.io.mem_resp
  writeArb.io.in(1).bits.wmask := Fix(-1)
  writeArb.io.in(1).bits.data := io.mem.xact_rep.bits.data

  // writebacks
  wb.io.req <> mshr.io.wb_req
  wb.io.meta_read <> metaReadArb.io.in(2)
  wb.io.data_req <> readArb.io.in(1)
  wb.io.data_resp := data_resp_mux
  wb.io.probe_rep_data <> io.mem.probe_rep_data

  // store->load bypassing
  val s4_valid = Reg(s3_valid, resetVal = Bool(false))
  val s4_req = RegEn(s3_req, s3_valid && metaReadArb.io.out.valid)
  val bypasses = List(
    (s2_valid_masked || s2_replay, s2_req, amoalu.io.out),
    (s3_valid, s3_req, s3_req.data),
    (s4_valid, s4_req, s4_req.data)
  ).map(r => (r._1 && (s1_addr >> conf.wordoffbits === r._2.addr >> conf.wordoffbits) && isWrite(r._2.cmd), r._3))
  val s2_store_bypass_data = Reg{Bits(width = conf.databits)}
  val s2_store_bypass = Reg{Bool()}
  when (s1_clk_en) {
    s2_store_bypass := false
    when (bypasses.map(_._1).reduce(_||_)) {
      s2_store_bypass_data := PriorityMux(bypasses.map(x => (x._1, x._2)))
      s2_store_bypass := true
    }
  }

  // load data subword mux/sign extension
  val s2_data_word_prebypass =
    if (conf.isNarrowRead) data_resp_mux(conf.databits-1,0)
    else data_resp_mux >> Cat(s2_req.addr(log2Up(MEM_DATA_BITS/8)-1,3), Bits(0,log2Up(conf.databits)))
  val s2_data_word = Mux(s2_store_bypass, s2_store_bypass_data, s2_data_word_prebypass)
  val loadgen = new LoadGen(s2_req.typ, s2_req.addr, s2_data_word)

  amoalu.io := s2_req
  amoalu.io.lhs := s2_data_word
  amoalu.io.rhs := s2_req.data

  // nack it like it's hot
  val s1_nack = dtlb.io.req.valid && dtlb.io.resp.miss ||
                s1_req.addr(indexmsb,indexlsb) === prober.io.meta_write.bits.idx && !prober.io.req.ready
  val s2_nack_hit = RegEn(s1_nack, s1_valid || s1_replay)
  when (s2_nack_hit) { mshr.io.req.valid := Bool(false) }
  val s2_nack_victim = s2_hit && mshr.io.secondary_miss
  val s2_nack_miss = !s2_hit && !mshr.io.req.ready
  val s2_nack_fence = s2_req.cmd === M_FENCE && !mshr.io.fence_rdy
  val s2_nack = s2_nack_hit || s2_nack_victim || s2_nack_miss || s2_nack_fence
  s2_valid_masked := s2_valid && !s2_nack

  // after a nack, block until nack condition resolves to save energy
  val block_fence = Reg(resetVal = Bool(false))
  block_fence := (s2_valid && s2_req.cmd === M_FENCE || block_fence) && !mshr.io.fence_rdy
  val block_miss = Reg(resetVal = Bool(false))
  block_miss := (s2_valid || block_miss) && s2_nack_miss
  when (block_fence || block_miss) {
    io.cpu.req.ready := Bool(false)
  }

  val s2_read = isRead(s2_req.cmd)
  io.cpu.resp.valid  := s2_read && (s2_replay || s2_valid_masked && s2_hit)
  io.cpu.resp.bits.nack := s2_valid && s2_nack
  io.cpu.resp.bits := s2_req
  io.cpu.resp.bits.replay := s2_replay && s2_read
  io.cpu.resp.bits.data := loadgen.word
  io.cpu.resp.bits.data_subword := loadgen.byte
  io.cpu.resp.bits.store_data := s2_req.data
  
  val xact_init_arb = (new Arbiter(2)) { new TransactionInit }
  xact_init_arb.io.in(0) <> wb.io.mem_req
  xact_init_arb.io.in(1).valid := mshr.io.mem_req.valid && prober.io.req.ready
  mshr.io.mem_req.ready := xact_init_arb.io.in(1).ready && prober.io.req.ready
  xact_init_arb.io.in(1).bits := mshr.io.mem_req.bits
  io.mem.xact_init <> xact_init_arb.io.out

  io.mem.xact_init_data <> wb.io.mem_req_data
  io.mem.xact_finish <> mshr.io.mem_finish
}
