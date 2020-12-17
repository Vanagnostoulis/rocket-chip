// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.tile

import chisel3._
import chisel3.util._
import chisel3.util.HasBlackBoxResource
import chisel3.experimental.IntParam
import chisel3.experimental.chiselName
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.InOrderArbiter

case object BuildRoCC extends Field[Seq[Parameters => LazyRoCC]](Nil)

class RoCCInstruction extends Bundle {
  val funct = Bits(7.W)
  val rs2 = Bits(5.W)
  val rs1 = Bits(5.W)
  val xd = Bool()
  val xs1 = Bool()
  val xs2 = Bool()
  val rd = Bits(5.W)
  val opcode = Bits(7.W)
}

class RoCCCommand(implicit p: Parameters) extends CoreBundle()(p) {
  val inst = new RoCCInstruction
  val rs1 = Bits(xLen.W)
  val rs2 = Bits(xLen.W)
  val status = new MStatus
}

class RoCCResponse(implicit p: Parameters) extends CoreBundle()(p) {
  val rd = Bits(5.W)
  val data = Bits(xLen.W)
}

class RoCCCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val cmd = Flipped(Decoupled(new RoCCCommand))
  val resp = Decoupled(new RoCCResponse)
  val mem = new HellaCacheIO
  val busy = Output(Bool())
  val interrupt = Output(Bool())
  val exception = Input(Bool())
}

class RoCCIO(val nPTWPorts: Int)(implicit p: Parameters) extends RoCCCoreIO()(p) {
  val ptw = Vec(nPTWPorts, new TLBPTWIO)
  val fpu_req = Decoupled(new FPInput)
  val fpu_resp = Flipped(Decoupled(new FPResult))
}

/** Base classes for Diplomatic TL2 RoCC units **/
abstract class LazyRoCC(
      val opcodes: OpcodeSet,
      val nPTWPorts: Int = 0,
      val usesFPU: Boolean = false
    )(implicit p: Parameters) extends LazyModule {
  val module: LazyRoCCModuleImp
  val atlNode: TLNode = TLIdentityNode()
  val tlNode: TLNode = TLIdentityNode()
}

class LazyRoCCModuleImp(outer: LazyRoCC) extends LazyModuleImp(outer) {
  val io = IO(new RoCCIO(outer.nPTWPorts))
}

/** Mixins for including RoCC **/

trait HasLazyRoCC extends CanHavePTW { this: BaseTile =>
  val roccs = p(BuildRoCC).map(_(p))

  roccs.map(_.atlNode).foreach { atl => tlMasterXbar.node :=* atl }
  roccs.map(_.tlNode).foreach { tl => tlOtherMastersNode :=* tl }

  nPTWPorts += roccs.map(_.nPTWPorts).sum
  nDCachePorts += roccs.size
}


trait HasLazyRoCCModule extends CanHavePTWModule
    with HasCoreParameters { this: RocketTileModuleImp with HasFpuOpt =>

  val (respArb, cmdRouter) = if(outer.roccs.nonEmpty) {
    val respArb = Module(new RRArbiter(new RoCCResponse()(outer.p), outer.roccs.size))
    val cmdRouter = Module(new RoccCommandRouter(outer.roccs.map(_.opcodes))(outer.p))
    outer.roccs.zipWithIndex.foreach { case (rocc, i) =>
      rocc.module.io.ptw ++=: ptwPorts
      rocc.module.io.cmd <> cmdRouter.io.out(i)
      val dcIF = Module(new SimpleHellaCacheIF()(outer.p))
      dcIF.io.requestor <> rocc.module.io.mem
      dcachePorts += dcIF.io.cache
      respArb.io.in(i) <> Queue(rocc.module.io.resp)
    }

    fpuOpt foreach { fpu =>
      val nFPUPorts = outer.roccs.count(_.usesFPU)
      if (usingFPU && nFPUPorts > 0) {
        val fpArb = Module(new InOrderArbiter(new FPInput()(outer.p), new FPResult()(outer.p), nFPUPorts))
        val fp_rocc_ios = outer.roccs.filter(_.usesFPU).map(_.module.io)
        fpArb.io.in_req <> fp_rocc_ios.map(_.fpu_req)
        fp_rocc_ios.zip(fpArb.io.in_resp).foreach {
          case (rocc, arb) => rocc.fpu_resp <> arb
        }
        fpu.io.cp_req <> fpArb.io.out_req
        fpArb.io.out_resp <> fpu.io.cp_resp
      } else {
        fpu.io.cp_req.valid := false.B
        fpu.io.cp_resp.ready := false.B
      }
    }
    (Some(respArb), Some(cmdRouter))
  } else {
    (None, None)
  }
}

class ExampleAccel (opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new ExampleAccelModuleImp(this)
}

@chiselName
class ExampleAccelModuleImp(outer: ExampleAccel)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
with HasCoreParameters {
  val rs1_reg     = Reg(chiselTypeOf(io.cmd.bits.rs1))
  val rs2_reg     = Reg(chiselTypeOf(io.cmd.bits.rs2))
  val rd_reg      = Reg(chiselTypeOf(io.cmd.bits.inst.rd))
  val mem_data    = Reg(chiselTypeOf(io.mem.resp.bits.data))
  val next_addr   = Reg(chiselTypeOf(io.mem.req.bits.addr))
  
  // fix width of inner_addr and size of Mem to support more
  val inner_addr  = RegInit(0.U(7.W))
  val inner_mem   = Mem(256, UInt(xLen.W)) // i think SyncReadMem needs 1cc delay to get the output in mem_data

  val s_idle :: s_mem_req_load :: s_mem_resp_load :: s_mem_req_store :: s_mem_resp_store :: s_resp :: s_read_inner :: Nil = Enum(7)
  val state = RegInit(s_idle)
  
  // eimai ready na dextw entoli mono stin idle
  io.cmd.ready := (state === s_idle)

  // otan erthei entoli pare auta pou xreiazomai 
  when (io.cmd.fire()){
    rs1_reg   := io.cmd.bits.rs1
    rs2_reg   := io.cmd.bits.rs2
    rd_reg    := io.cmd.bits.inst.rd
    next_addr := io.cmd.bits.rs1(39,0) //first address is in rs1. rs1->64, mem.addr-> 40 bits 
   
    inner_addr := io.cmd.bits.rs1(9,3)

    when(io.cmd.bits.inst.funct === 0.U){
      state := s_mem_req_load
    }.elsewhen(io.cmd.bits.inst.funct === 1.U){
      state := s_mem_req_store
    }.elsewhen(io.cmd.bits.inst.funct === 2.U){
      state := s_read_inner
    }
  }

  when(state === s_read_inner){
    //printf(p"INFO pame gia read stin inner_addr $inner_addr me ${inner_mem(inner_addr)} read ${inner_mem.read(inner_addr)} ${inner_mem(106)} \n")
    mem_data := inner_mem.read(inner_addr)
    state    := s_resp
  }

  when(io.resp.fire()) { state := s_idle }
  
  // response from a request, must care of OOO responses
  when(io.mem.resp.valid){
    when (state === s_mem_resp_load){
      inner_mem.write( (next_addr(9,3)) , io.mem.resp.bits.data)
      state       :=   s_mem_req_load
      printf(p"INFO:load  resp address 0x${Hexadecimal(next_addr)} | ${next_addr(9,3)} state $state data ${io.mem.resp.bits.data}\n")
      next_addr   := next_addr + 8.U
    }
    when (state === s_mem_resp_store){
      state    := s_mem_req_store
      next_addr:= next_addr + 8.U //go to the next element and change it
     //printf(p"INFO:store resp address 0x${Hexadecimal(next_addr)}  state $state data ${io.mem.resp.bits.data}\n")
    }
    when (next_addr === rs2_reg ) { state := s_resp }  //end of matrix give a respone to let him know we are done 
  }

  // when I send a request to memory wait for response 
  when(io.mem.req.fire()){ 
    when (state === s_mem_req_load){
      state := s_mem_resp_load
      //printf(p"INFO:load  req  address 0x${Hexadecimal(io.mem.req.bits.addr)}        state $state data ${io.mem.req.bits.data}\n")
    }
    when (state === s_mem_req_store){
      state := s_mem_resp_store    
      printf(p"INFO:store req  address 0x${Hexadecimal(io.mem.req.bits.addr)}  state $state data ${io.mem.req.bits.data}\n")
    }
  }
 
  io.resp.valid     := (state === s_resp) 
  io.resp.bits.data := mem_data 
  io.resp.bits.rd   := rd_reg

  io.mem.req.valid      := ((state === s_mem_req_load) || (state === s_mem_req_store))
  io.mem.req.bits.addr  := next_addr
  io.mem.req.bits.tag   := Mux( state === s_mem_req_load, 0.U , 1.U) //maybe find another way for tagging
  io.mem.req.bits.cmd   := Mux( state === s_mem_req_load, M_XRD, M_XWR) 
  io.mem.req.bits.size  := log2Ceil(8).U
  io.mem.req.bits.signed:= false.B 
  io.mem.req.bits.phys  := false.B
  io.mem.req.bits.data  := Mux( state === s_mem_req_load, 0.U, (inner_mem(next_addr(9,3)) + 4.U) )

  io.busy := (state =/= s_idle)
  io.interrupt := false.B
 }

class AccumulatorExample(opcodes: OpcodeSet, val n: Int = 4)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new AccumulatorExampleModuleImp(this)
}

@chiselName
class AccumulatorExampleModuleImp(outer: AccumulatorExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  val regfile = Mem(outer.n, UInt(xLen.W))
  val busy = RegInit(VecInit(Seq.fill(outer.n){false.B}))

  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct
  val addr = cmd.bits.rs2(log2Up(outer.n)-1,0)
  val doWrite = funct === 0.U
  val doRead = funct === 1.U
  val doLoad = funct === 2.U
  val doAccum = funct === 3.U
  val memRespTag = io.mem.resp.bits.tag(log2Up(outer.n)-1,0)

  // datapath
  val addend = cmd.bits.rs1
  val accum = regfile(addr)
  val wdata = Mux(doWrite, addend, accum + addend)

  when (cmd.fire() && (doWrite || doAccum)) {
    regfile(addr) := wdata
  }

  when (io.mem.resp.valid) {
    regfile(memRespTag) := io.mem.resp.bits.data
    busy(memRespTag) := false.B
  }

  // control
  when (io.mem.req.fire()) {
    busy(addr) := true.B
  }

  val doResp = cmd.bits.inst.xd
  val stallReg = busy(addr)
  val stallLoad = doLoad && !io.mem.req.ready
  val stallResp = doResp && !io.resp.ready

  cmd.ready := !stallReg && !stallLoad && !stallResp
    // command resolved if no stalls AND not issuing a load that will need a request

  // PROC RESPONSE INTERFACE
  io.resp.valid := cmd.valid && doResp && !stallReg && !stallLoad
    // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := cmd.bits.inst.rd
    // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := accum
    // Semantics is to always send out prior accumulator register value

  io.busy := cmd.valid || busy.reduce(_||_)
    // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := false.B
    // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := cmd.valid && doLoad && !stallReg && !stallResp
  io.mem.req.bits.addr := addend
  io.mem.req.bits.tag := addr
  io.mem.req.bits.cmd := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.size := log2Ceil(8).U
  io.mem.req.bits.signed := false.B
  io.mem.req.bits.data := 0.U // we're not performing any stores...
  io.mem.req.bits.phys := false.B
  io.mem.req.bits.dprv := cmd.bits.status.dprv
}

class  TranslatorExample(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes, nPTWPorts = 1) {
  override lazy val module = new TranslatorExampleModuleImp(this)
}

@chiselName
class TranslatorExampleModuleImp(outer: TranslatorExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  val req_addr = Reg(UInt(coreMaxAddrBits.W))
  val req_rd = Reg(chiselTypeOf(io.resp.bits.rd))
  val req_offset = req_addr(pgIdxBits - 1, 0)
  val req_vpn = req_addr(coreMaxAddrBits - 1, pgIdxBits)
  val pte = Reg(new PTE)

  val s_idle :: s_ptw_req :: s_ptw_resp :: s_resp :: Nil = Enum(4)
  val state = RegInit(s_idle)

  io.cmd.ready := (state === s_idle)

  when (io.cmd.fire()) {
    req_rd := io.cmd.bits.inst.rd
    req_addr := io.cmd.bits.rs1
    state := s_ptw_req
  }

  private val ptw = io.ptw(0)

  when (ptw.req.fire()) { state := s_ptw_resp }

  when (state === s_ptw_resp && ptw.resp.valid) {
    pte := ptw.resp.bits.pte
    state := s_resp
  }

  when (io.resp.fire()) { state := s_idle }

  ptw.req.valid := (state === s_ptw_req)
  ptw.req.bits.valid := true.B
  ptw.req.bits.bits.addr := req_vpn

  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := req_rd
  io.resp.bits.data := Mux(pte.leaf(), Cat(pte.ppn, req_offset), -1.S(xLen.W).asUInt)

  io.busy := (state =/= s_idle)
  io.interrupt := false.B
  io.mem.req.valid := false.B
}

class  CharacterCountExample(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new CharacterCountExampleModuleImp(this)
  override val atlNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1("CharacterCountRoCC")))))
}

@chiselName
class CharacterCountExampleModuleImp(outer: CharacterCountExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
  with HasCoreParameters
  with HasL1CacheParameters {
  val cacheParams = tileParams.icache.get
  private val blockOffset = blockOffBits
  // blockoffbits = lgCacheBlockBytes = 6
  // cacheDataBits = beatbytes * 8 = 64 
  // beatOffset = 3 (?)
  private val beatOffset = log2Up(cacheDataBits/8)
  val needle = Reg(UInt(8.W))
  val addr = Reg(UInt(coreMaxAddrBits.W))
  val count = Reg(UInt(xLen.W))
  val resp_rd = Reg(chiselTypeOf(io.resp.bits.rd))

  // coreMaxAddrBits = 40
  val addr_block = addr(coreMaxAddrBits - 1, blockOffset)
  // addr_block = addr(39,6)  
  // offset = addr(5,0)
  // next_addr = ((addr(55,6)) +1) << 6 
  val offset = addr(blockOffset - 1, 0)
  val next_addr = (addr_block + 1.U) << blockOffset.U

  val s_idle :: s_acq :: s_gnt :: s_check :: s_resp :: Nil = Enum(5)
  val state = RegInit(s_idle)

  // 6.8 and 9.1 in chipyard  
  //Inside your lazy module implementation, you can call node.out to get a list
  //of bundle/edge pairs. If you used the TLHelper, you only specified a single
  //client edge, so this list will only have one pair.
  val (tl_out, edgesOut) = outer.atlNode.out(0)
  
  // The tl bundle is a Chisel hardware bundle that connects to the IO of this
  // module. It contains two (in the case of TL-UL and TL-UH) or five (in the
  // case of TL-C) decoupled bundles corresponding to the TileLink channels.
  // This is what you should connect your hardware logic to in order to actually
  // send/receive TileLink messages.
  
  /* 
  An AcquireBlock message is a request message type used by a Master Agent with a cache to
  obtain a copy of a block of data that it plans to cache locally

  A Grant message is both a response and a request message used by a Slave Agent to acknowledge 
  the receipt of a Acquire and provide permissions to access the cache block to the original requesting Master Agent
  */
  val gnt = tl_out.d.bits
  // recv_data = 64.W
  // recv_beat = log2(1+1) = 1.W 
  val recv_data = Reg(UInt(cacheDataBits.W))
  val recv_beat = RegInit(0.U(log2Up(cacheDataBeats+1).W))
  // data bytes (recdata(7,0),recdata(15,8),recdata(23,16),...,recdata(63,56))
  val data_bytes = VecInit(Seq.tabulate(cacheDataBits/8) { i => recv_data(8 * (i + 1) - 1, 8 * i) })
  val zero_match = data_bytes.map(_ === 0.U)
  val needle_match = data_bytes.map(_ === needle)
  val first_zero = PriorityEncoder(zero_match)
  //  Returns the bit position of the least-significant high bit of the input bitvector. "b0110"esults in 1.U



  val chars_found = PopCount(needle_match.zipWithIndex.map {
    case (matches, i) =>
      val idx = Cat(recv_beat - 1.U, i.U(beatOffset.W))
      matches && idx >= offset && i.U <= first_zero
  })
  val zero_found = zero_match.reduce(_ || _)
  val finished = Reg(Bool())

  io.cmd.ready := (state === s_idle)
  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := resp_rd
  io.resp.bits.data := count
  tl_out.a.valid := (state === s_acq)
  tl_out.a.bits := edgesOut.Get(
                       fromSource = 0.U,
                       toAddress = addr_block << blockOffset,
                       lgSize = lgCacheBlockBytes.U)._2
  tl_out.d.ready := (state === s_gnt)

  when (io.cmd.fire()) {
//    printf(p"INFO: blockOffset $blockOffset beatOffset $beatOffset cacheDataBits $cacheDataBits coreMaxAddrBits $coreMaxAddrBits , \n")
    //printf(p"INFO: addr_block $addr_block next_addr $next_addr offset $offset cacheDataBeats $cacheDataBeats\n")
//    printf(p"INFO: data_bytes $data_bytes  \n")
 //   printf(p"INFO: chars_found $chars_found  \n")
    addr := io.cmd.bits.rs1
    needle := io.cmd.bits.rs2
    resp_rd := io.cmd.bits.inst.rd
    count := 0.U
    finished := false.B
    state := s_acq
  }

  // otan steilw ACQUIRE message sto tilelink perimenw gia GRANT sto D channel
  when (tl_out.a.fire()) { state := s_gnt }
  
  // otan parw apantisi GRANT apo to D channel kanw save ta data
  when (tl_out.d.fire()) {
    recv_beat := recv_beat + 1.U
    recv_data := gnt.data
    state := s_check
  }
  // chekarw an exw diavasei mideniko opou teleiwnw to treksimo
  // alliws 
  when (state === s_check) {
//	printf(p"CHECK: data_bytes $data_bytes  \n")
//    printf(p"CHECK: chars_found $chars_found  \n")
    when (!finished) {
      count := count + chars_found
    }
    when (zero_found) { finished := true.B }
    when (recv_beat === cacheDataBeats.U) {
      addr := next_addr
      state := Mux(zero_found || finished, s_resp, s_acq)
    } .otherwise {
      state := s_gnt
    }
  }

  when (io.resp.fire()) { state := s_idle }

  io.busy := (state =/= s_idle)
  io.interrupt := false.B
  io.mem.req.valid := false.B
  // Tie off unused channels
  tl_out.b.ready := true.B
  tl_out.c.valid := false.B
  tl_out.e.valid := false.B
}

class BlackBoxExample(opcodes: OpcodeSet, blackBoxFile: String)(implicit p: Parameters)
    extends LazyRoCC(opcodes) {
  override lazy val module = new BlackBoxExampleModuleImp(this, blackBoxFile)
}

class BlackBoxExampleModuleImp(outer: BlackBoxExample, blackBoxFile: String)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer)
    with RequireSyncReset
    with HasCoreParameters {

  val blackbox = {
    val roccIo = io
    Module(
      new BlackBox( Map( "xLen" -> IntParam(xLen),
                         "PRV_SZ" -> IntParam(PRV.SZ),
                         "coreMaxAddrBits" -> IntParam(coreMaxAddrBits),
                         "dcacheReqTagBits" -> IntParam(roccIo.mem.req.bits.tag.getWidth),
                         "M_SZ" -> IntParam(M_SZ),
                         "mem_req_bits_size_width" -> IntParam(roccIo.mem.req.bits.size.getWidth),
                         "coreDataBits" -> IntParam(coreDataBits),
                         "coreDataBytes" -> IntParam(coreDataBytes),
                         "paddrBits" -> IntParam(paddrBits),
                         "FPConstants_RM_SZ" -> IntParam(FPConstants.RM_SZ),
                         "fLen" -> IntParam(fLen),
                         "FPConstants_FLAGS_SZ" -> IntParam(FPConstants.FLAGS_SZ)
                   ) ) with HasBlackBoxResource {
        val io = IO( new Bundle {
                      val clock = Input(Clock())
                      val reset = Input(Reset())
                      val rocc = chiselTypeOf(roccIo)
                    })
        override def desiredName: String = blackBoxFile
        addResource(s"/vsrc/$blackBoxFile.v")
      }
    )
  }

  blackbox.io.clock := clock
  blackbox.io.reset := reset
  blackbox.io.rocc.cmd <> io.cmd
  io.resp <> blackbox.io.rocc.resp
  io.mem <> blackbox.io.rocc.mem
  io.busy := blackbox.io.rocc.busy
  io.interrupt := blackbox.io.rocc.interrupt
  blackbox.io.rocc.exception := io.exception
  io.ptw <> blackbox.io.rocc.ptw
  io.fpu_req <> blackbox.io.rocc.fpu_req
  blackbox.io.rocc.fpu_resp <> io.fpu_resp

}

class OpcodeSet(val opcodes: Seq[UInt]) {
  def |(set: OpcodeSet) =
    new OpcodeSet(this.opcodes ++ set.opcodes)

  def matches(oc: UInt) = opcodes.map(_ === oc).reduce(_ || _)
}

object OpcodeSet {
  def custom0 = new OpcodeSet(Seq("b0001011".U))
  def custom1 = new OpcodeSet(Seq("b0101011".U))
  def custom2 = new OpcodeSet(Seq("b1011011".U))
  def custom3 = new OpcodeSet(Seq("b1111011".U))
  def all = custom0 | custom1 | custom2 | custom3
}

class RoccCommandRouter(opcodes: Seq[OpcodeSet])(implicit p: Parameters)
    extends CoreModule()(p) {
  val io = new Bundle {
    val in = Flipped(Decoupled(new RoCCCommand))
    val out = Vec(opcodes.size, Decoupled(new RoCCCommand))
    val busy = Output(Bool())
  }

  val cmd = Queue(io.in)
  val cmdReadys = io.out.zip(opcodes).map { case (out, opcode) =>
    val me = opcode.matches(cmd.bits.inst.opcode)
    out.valid := cmd.valid && me
    out.bits := cmd.bits
    out.ready && me
  }
  cmd.ready := cmdReadys.reduce(_ || _)
  io.busy := cmd.valid

  assert(PopCount(cmdReadys) <= 1.U,
    "Custom opcode matched for more than one accelerator")
}
